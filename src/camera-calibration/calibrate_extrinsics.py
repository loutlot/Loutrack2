#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

SRC_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parent
for path in (SRC_ROOT, MODULE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from host.geo import CalibrationLoader, CameraParams
from extrinsics_capture import load_wand_metric_observations
from extrinsics_scale import VALID_WAND_FACE_CONSTRAINTS, WAND_FACE_FRONT_UP, apply_wand_metric_alignment
from extrinsics_validate import validate_wand_metric_extrinsics
from calibration.targets.wand import WAND_POINTS_MM


DEFAULT_OUTPUT = "calibration/extrinsics_pose_v2.json"
DEFAULT_PAIR_WINDOW_US = 4166
DEFAULT_WAND_PAIR_WINDOW_US = 4166
DEFAULT_MIN_PAIRS = 8
_LARGE_RESIDUAL_PX = 50.0


@dataclass(frozen=True)
class PoseObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    image_point: np.ndarray


@dataclass(frozen=True)
class CameraRow:
    anchor_timestamp: int
    anchor_frame_index: int
    image_points: List[np.ndarray | None]
    matched_delta_us: List[int | None]


def _sanitize_path_for_payload(path_value: str | Path) -> str:
    path = Path(path_value)
    if not path.is_absolute():
        return path.as_posix()
    cwd = Path.cwd().resolve()
    try:
        return path.resolve().relative_to(cwd).as_posix()
    except ValueError:
        # Keep payload portable and avoid leaking machine-local absolute directories.
        return path.name


def _extract_frame_payload(entry: Dict[str, Any]) -> Dict[str, Any] | None:
    if entry.get("_type") == "frame" and isinstance(entry.get("data"), dict):
        return entry["data"]
    if "camera_id" in entry and "timestamp" in entry and "blobs" in entry:
        return entry
    return None


def load_pose_capture_rows(path: str | Path) -> Dict[str, List[PoseObservation]]:
    rows: Dict[str, List[PoseObservation]] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            entry = json.loads(line)
            payload = _extract_frame_payload(entry)
            if payload is None:
                continue
            camera_id = str(payload.get("camera_id", "")).strip()
            blobs = payload.get("blobs", [])
            if not camera_id or not isinstance(blobs, list):
                continue
            payload_blob_count = payload.get("blob_count")
            blob_count = int(payload_blob_count) if isinstance(payload_blob_count, (int, float)) else len(blobs)
            if blob_count != 1 or len(blobs) != 1:
                continue
            blob = blobs[0]
            rows.setdefault(camera_id, []).append(
                PoseObservation(
                    camera_id=camera_id,
                    timestamp=int(payload.get("timestamp", 0)),
                    frame_index=int(payload.get("frame_index", 0)),
                    image_point=np.array([float(blob.get("x", 0.0)), float(blob.get("y", 0.0))], dtype=np.float64),
                )
            )
    for camera_id in rows:
        rows[camera_id].sort(key=lambda item: item.timestamp)
    return rows


def _nearest_within_window(
    observations: Sequence[PoseObservation],
    timestamp: int,
    pair_window_us: int,
) -> PoseObservation | None:
    if not observations:
        return None
    lo = 0
    hi = len(observations)
    while lo < hi:
        mid = (lo + hi) // 2
        if observations[mid].timestamp < timestamp:
            lo = mid + 1
        else:
            hi = mid
    candidates: List[PoseObservation] = []
    if lo < len(observations):
        candidates.append(observations[lo])
    if lo > 0:
        candidates.append(observations[lo - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda item: abs(item.timestamp - timestamp))
    if abs(best.timestamp - timestamp) > pair_window_us:
        return None
    return best


def _pair_overlap_count(
    observations_a: Sequence[PoseObservation],
    observations_b: Sequence[PoseObservation],
    pair_window_us: int,
) -> int:
    count = 0
    for obs in observations_a:
        if _nearest_within_window(observations_b, obs.timestamp, pair_window_us) is not None:
            count += 1
    return count


def _pair_overlap_key(camera_a: str, camera_b: str) -> Tuple[str, str]:
    return tuple(sorted((camera_a, camera_b)))


def _build_camera_order(
    observations_by_camera: Dict[str, List[PoseObservation]],
    pair_window_us: int,
) -> tuple[List[str], Dict[Tuple[str, str], int]]:
    camera_ids = sorted(camera_id for camera_id, rows in observations_by_camera.items() if rows)
    if len(camera_ids) < 2:
        raise ValueError("At least two cameras with single-blob pose capture observations are required")

    overlaps: Dict[Tuple[str, str], int] = {}
    for camera_a, camera_b in combinations(camera_ids, 2):
        overlaps[_pair_overlap_key(camera_a, camera_b)] = _pair_overlap_count(
            observations_by_camera[camera_a],
            observations_by_camera[camera_b],
            pair_window_us,
        )

    strongest_pair = max(
        overlaps.items(),
        key=lambda item: (item[1], item[0][0], item[0][1]),
    )[0]
    ordered: List[str] = [strongest_pair[0], strongest_pair[1]]
    remaining = set(camera_ids) - set(ordered)

    while remaining:
        left = ordered[0]
        right = ordered[-1]
        best_camera = None
        best_side = "right"
        best_score = -1
        for camera_id in sorted(remaining):
            score_left = overlaps.get(_pair_overlap_key(camera_id, left), 0)
            score_right = overlaps.get(_pair_overlap_key(camera_id, right), 0)
            side = "left" if score_left >= score_right else "right"
            score = max(score_left, score_right)
            candidate = (score, camera_id, side)
            incumbent = (best_score, best_camera or "", best_side)
            if candidate > incumbent:
                best_score = score
                best_camera = camera_id
                best_side = side
        assert best_camera is not None
        if best_side == "left":
            ordered.insert(0, best_camera)
        else:
            ordered.append(best_camera)
        remaining.remove(best_camera)

    return ordered, overlaps


def build_camera_order_and_rows(
    observations_by_camera: Dict[str, List[PoseObservation]],
    pair_window_us: int,
) -> tuple[List[str], List[CameraRow], Dict[str, int]]:
    camera_order, overlaps = _build_camera_order(observations_by_camera, pair_window_us)
    anchor_camera = camera_order[0]
    rows: List[CameraRow] = []
    complete_rows = 0
    for obs in observations_by_camera[anchor_camera]:
        image_points: List[np.ndarray | None] = [obs.image_point.astype(np.float64)]
        matched_delta_us: List[int | None] = [None]
        for camera_id in camera_order[1:]:
            match = _nearest_within_window(observations_by_camera[camera_id], obs.timestamp, pair_window_us)
            image_points.append(None if match is None else match.image_point.astype(np.float64))
            matched_delta_us.append(None if match is None else int(abs(match.timestamp - obs.timestamp)))
        if sum(point is not None for point in image_points) == len(camera_order):
            complete_rows += 1
        rows.append(
            CameraRow(
                anchor_timestamp=obs.timestamp,
                anchor_frame_index=obs.frame_index,
                image_points=image_points,
                matched_delta_us=matched_delta_us,
            )
        )

    pair_overlaps = {f"{a}|{b}": int(count) for (a, b), count in overlaps.items()}
    return camera_order, rows, {
        "captured_rows": len(rows),
        "complete_rows": complete_rows,
        "pair_overlaps": pair_overlaps,
    }


def _scaled_intrinsic_matrix(camera: CameraParams, focal_scale: float) -> np.ndarray:
    intrinsic = np.array(camera.intrinsic_matrix, dtype=np.float64, copy=True)
    intrinsic[0, 0] *= focal_scale
    intrinsic[1, 1] *= focal_scale
    return intrinsic


def _triangulate_multiview_point(
    observations: Sequence[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> np.ndarray | None:
    if len(observations) < 2:
        return None
    rows: List[np.ndarray] = []
    for intrinsic, distortion, projection, image_point in observations:
        normalized = cv2.undistortPoints(
            np.asarray(image_point, dtype=np.float64).reshape(1, 1, 2),
            intrinsic.astype(np.float64),
            distortion.astype(np.float64),
        ).reshape(2)
        rows.append(normalized[0] * projection[2] - projection[0])
        rows.append(normalized[1] * projection[2] - projection[1])
    system = np.stack(rows, axis=0)
    _, _, vt = np.linalg.svd(system, full_matrices=False)
    point_h = vt[-1]
    if abs(float(point_h[3])) < 1e-12:
        return None
    return (point_h[:3] / point_h[3]).astype(np.float64)


def _project_point(
    point_world: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
    intrinsic: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(rotation.astype(np.float64))
    projected, _ = cv2.projectPoints(
        point_world.reshape(1, 3).astype(np.float64),
        rvec,
        translation.reshape(3, 1).astype(np.float64),
        intrinsic.astype(np.float64),
        distortion.astype(np.float64),
    )
    return projected.reshape(2).astype(np.float64)


def _select_relative_motion(
    points_a: np.ndarray,
    points_b: np.ndarray,
    intrinsic_a: np.ndarray,
    intrinsic_b: np.ndarray,
    distortion_a: np.ndarray,
    distortion_b: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    pts_a_undist = cv2.undistortPoints(
        points_a.reshape(-1, 1, 2).astype(np.float64),
        intrinsic_a.astype(np.float64),
        distortion_a.astype(np.float64),
        P=intrinsic_a.astype(np.float64),
    ).reshape(-1, 2)
    pts_b_undist = cv2.undistortPoints(
        points_b.reshape(-1, 1, 2).astype(np.float64),
        intrinsic_b.astype(np.float64),
        distortion_b.astype(np.float64),
        P=intrinsic_b.astype(np.float64),
    ).reshape(-1, 2)
    fundamental, mask = cv2.findFundamentalMat(pts_a_undist, pts_b_undist, cv2.FM_RANSAC, 1.0, 0.99999)
    if fundamental is None:
        raise ValueError("findFundamentalMat failed for an adjacent camera pair")
    fundamental = np.asarray(fundamental, dtype=np.float64).reshape(-1, 3, 3)[0]
    essential = intrinsic_b.T @ fundamental @ intrinsic_a
    r1, r2, t = cv2.decomposeEssentialMat(essential)
    candidates = ((r1, t.reshape(3)), (r1, -t.reshape(3)), (r2, t.reshape(3)), (r2, -t.reshape(3)))

    if mask is not None:
        inliers = mask.reshape(-1) > 0
        pts_a_undist = pts_a_undist[inliers]
        pts_b_undist = pts_b_undist[inliers]
    if len(pts_a_undist) < 2:
        raise ValueError("not enough inliers after RANSAC for adjacent camera initialization")

    best_rotation = None
    best_translation = None
    best_score = -1
    projection_a = intrinsic_a @ np.hstack([np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64)])
    for rotation_rel, translation_rel in candidates:
        projection_b = intrinsic_b @ np.hstack([rotation_rel, translation_rel.reshape(3, 1)])
        points4d = cv2.triangulatePoints(
            projection_a,
            projection_b,
            pts_a_undist.T.astype(np.float64),
            pts_b_undist.T.astype(np.float64),
        )
        points3d = (points4d[:3] / points4d[3]).T
        depth_a = points3d[:, 2]
        depth_b = (rotation_rel @ points3d.T + translation_rel.reshape(3, 1))[2]
        score = int(np.sum((depth_a > 0.0) & (depth_b > 0.0)))
        if score > best_score:
            best_rotation = rotation_rel
            best_translation = translation_rel
            best_score = score
    if best_rotation is None or best_translation is None:
        raise ValueError("failed to recover a cheirality-valid relative pose")
    return best_rotation.astype(np.float64), best_translation.astype(np.float64)


def initialize_camera_poses_chain(
    camera_order: Sequence[str],
    rows: Sequence[CameraRow],
    camera_params: Dict[str, CameraParams],
    min_pairs: int,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """Initialize camera poses by anchoring each new camera to its best parent.

    Rather than chaining strictly in sequence (which compounds errors), each
    camera beyond the first is initialised relative to whichever already-placed
    camera shares the most simultaneous observations with it.  The pair with
    the highest overlap is inherently the most reliable basis for the Essential
    Matrix decomposition, so this strategy keeps initialisation errors local
    and reduces drift for later cameras in the ordering.
    """
    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        camera_order[0]: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
    }
    initialized: List[str] = [camera_order[0]]

    for current_camera in camera_order[1:]:
        idx_curr = camera_order.index(current_camera)

        # Find the already-initialised camera with the most shared rows.
        best_parent = initialized[0]
        best_count = 0
        for parent in initialized:
            idx_parent = camera_order.index(parent)
            count = sum(
                1 for row in rows
                if row.image_points[idx_parent] is not None
                and row.image_points[idx_curr] is not None
            )
            if count > best_count:
                best_count = count
                best_parent = parent

        idx_parent = camera_order.index(best_parent)
        pair_points_parent: List[np.ndarray] = []
        pair_points_curr: List[np.ndarray] = []
        for row in rows:
            point_parent = row.image_points[idx_parent]
            point_curr = row.image_points[idx_curr]
            if point_parent is None or point_curr is None:
                continue
            pair_points_parent.append(point_parent.astype(np.float64))
            pair_points_curr.append(point_curr.astype(np.float64))

        if len(pair_points_parent) < min_pairs:
            raise ValueError(
                f"best parent pair {best_parent} <-> {current_camera} has only "
                f"{len(pair_points_parent)} usable rows (need {min_pairs})"
            )

        rotation_rel, translation_rel = _select_relative_motion(
            np.asarray(pair_points_parent, dtype=np.float64),
            np.asarray(pair_points_curr, dtype=np.float64),
            camera_params[best_parent].intrinsic_matrix,
            camera_params[current_camera].intrinsic_matrix,
            camera_params[best_parent].distortion_coeffs,
            camera_params[current_camera].distortion_coeffs,
        )
        rotation_parent, translation_parent = poses[best_parent]
        rotation_curr = rotation_rel @ rotation_parent
        translation_curr = (rotation_rel @ translation_parent.reshape(3, 1)).reshape(3) + translation_rel.reshape(3)
        poses[current_camera] = (rotation_curr.astype(np.float64), translation_curr.astype(np.float64))
        initialized.append(current_camera)

    return poses


def _pack_parameters(
    camera_order: Sequence[str],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    focal_scales: Dict[str, float],
) -> np.ndarray:
    values: List[float] = []
    for camera_id in camera_order:
        values.append(float(focal_scales[camera_id]))
    for camera_id in camera_order[1:]:
        rotation, translation = camera_poses[camera_id]
        values.extend(Rotation.from_matrix(rotation).as_rotvec().tolist())
        values.extend(np.asarray(translation, dtype=np.float64).reshape(3).tolist())
    return np.asarray(values, dtype=np.float64)


def _unpack_parameters(
    params: np.ndarray,
    camera_order: Sequence[str],
) -> tuple[Dict[str, float], Dict[str, Tuple[np.ndarray, np.ndarray]]]:
    n_cameras = len(camera_order)
    focal_scales = {
        camera_order[idx]: float(params[idx])
        for idx in range(n_cameras)
    }
    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        camera_order[0]: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
    }
    base = n_cameras
    for idx, camera_id in enumerate(camera_order[1:]):
        offset = base + idx * 6
        rotation = Rotation.from_rotvec(params[offset : offset + 3]).as_matrix().astype(np.float64)
        translation = np.asarray(params[offset + 3 : offset + 6], dtype=np.float64)
        poses[camera_id] = (rotation, translation)
    return focal_scales, poses


def _prepare_rows_for_optimization(
    camera_order: Sequence[str],
    rows: Sequence[CameraRow],
) -> List[List[tuple[str, np.ndarray]]]:
    prepared: List[List[tuple[str, np.ndarray]]] = []
    for row in rows:
        observations = [
            (camera_order[idx], point.astype(np.float64))
            for idx, point in enumerate(row.image_points)
            if point is not None
        ]
        if len(observations) >= 2:
            prepared.append(observations)
    return prepared


def bundle_adjust_camera_poses(
    camera_order: Sequence[str],
    rows: Sequence[CameraRow],
    camera_params: Dict[str, CameraParams],
    camera_poses_init: Dict[str, Tuple[np.ndarray, np.ndarray]],
) -> tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[str, float], Dict[str, Any]]:
    prepared_rows = _prepare_rows_for_optimization(camera_order, rows)
    if not prepared_rows:
        raise ValueError("no multiview rows with at least two observations are available for optimization")
    focal_scales_init = {camera_id: 1.0 for camera_id in camera_order}
    x0 = _pack_parameters(camera_order, camera_poses_init, focal_scales_init)
    lower = np.full_like(x0, -np.inf, dtype=np.float64)
    upper = np.full_like(x0, np.inf, dtype=np.float64)
    lower[: len(camera_order)] = 0.8
    upper[: len(camera_order)] = 1.2

    def residuals(params: np.ndarray) -> np.ndarray:
        focal_scales, camera_poses = _unpack_parameters(params, camera_order)
        residual_values: List[float] = []
        scaled_intrinsics = {
            camera_id: _scaled_intrinsic_matrix(camera_params[camera_id], focal_scales[camera_id])
            for camera_id in camera_order
        }
        for observations in prepared_rows:
            triangulation_rows = []
            for camera_id, image_point in observations:
                rotation, translation = camera_poses[camera_id]
                projection = np.hstack([rotation, translation.reshape(3, 1)])
                triangulation_rows.append((scaled_intrinsics[camera_id], camera_params[camera_id].distortion_coeffs, projection, image_point))
            point_world = _triangulate_multiview_point(triangulation_rows)
            if point_world is None or not np.all(np.isfinite(point_world)):
                residual_values.extend([_LARGE_RESIDUAL_PX, _LARGE_RESIDUAL_PX] * len(observations))
                continue
            for camera_id, image_point in observations:
                rotation, translation = camera_poses[camera_id]
                projected = _project_point(
                    point_world,
                    rotation,
                    translation,
                    scaled_intrinsics[camera_id],
                    camera_params[camera_id].distortion_coeffs,
                )
                residual = image_point - projected
                residual_values.extend(residual.tolist())
        return np.asarray(residual_values, dtype=np.float64)

    result = least_squares(
        residuals,
        x0,
        bounds=(lower, upper),
        loss="cauchy",
        f_scale=2.0,
        ftol=1e-2,
        xtol=1e-4,
        gtol=1e-4,
    )
    focal_scales, camera_poses = _unpack_parameters(result.x, camera_order)
    return camera_poses, focal_scales, {
        "iterations": int(result.nfev),
        "ba_cost": float(result.cost),
        "status": int(result.status),
        "success": bool(result.success),
        "message": str(result.message),
    }


def _compute_reprojection_statistics(
    camera_order: Sequence[str],
    rows: Sequence[CameraRow],
    camera_params: Dict[str, CameraParams],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    focal_scales: Dict[str, float],
) -> tuple[Dict[str, float], Dict[str, Any]]:
    per_camera_errors: Dict[str, List[float]] = {camera_id: [] for camera_id in camera_order}
    global_errors: List[float] = []
    usable_rows = 0
    scaled_intrinsics = {
        camera_id: _scaled_intrinsic_matrix(camera_params[camera_id], focal_scales[camera_id])
        for camera_id in camera_order
    }
    for row in rows:
        observations = [
            (camera_order[idx], point.astype(np.float64))
            for idx, point in enumerate(row.image_points)
            if point is not None
        ]
        if len(observations) < 2:
            continue
        usable_rows += 1
        point_world = _triangulate_multiview_point(
            [
                (
                    scaled_intrinsics[camera_id],
                    camera_params[camera_id].distortion_coeffs,
                    np.hstack([camera_poses[camera_id][0], camera_poses[camera_id][1].reshape(3, 1)]),
                    image_point,
                )
                for camera_id, image_point in observations
            ]
        )
        if point_world is None:
            continue
        for camera_id, image_point in observations:
            rotation, translation = camera_poses[camera_id]
            projected = _project_point(
                point_world,
                rotation,
                translation,
                scaled_intrinsics[camera_id],
                camera_params[camera_id].distortion_coeffs,
            )
            error = float(np.linalg.norm(projected - image_point))
            global_errors.append(error)
            per_camera_errors[camera_id].append(error)
    per_camera_median = {
        camera_id: float(np.median(values))
        for camera_id, values in per_camera_errors.items()
        if values
    }
    return per_camera_median, {
        "usable_rows": usable_rows,
        "median_reproj_error_px": float(np.median(global_errors)) if global_errors else None,
        "p90_reproj_error_px": float(np.percentile(global_errors, 90)) if global_errors else None,
    }


def _compute_matching_statistics(rows: Sequence[CameraRow]) -> Dict[str, int | None]:
    matched_deltas: List[int] = []
    for row in rows:
        observations = [point for point in row.image_points if point is not None]
        if len(observations) < 2:
            continue
        for delta in row.matched_delta_us[1:]:
            if delta is not None:
                matched_deltas.append(int(delta))
    if not matched_deltas:
        return {
            "matched_delta_us_p50": None,
            "matched_delta_us_p90": None,
            "matched_delta_us_max": None,
        }
    return {
        "matched_delta_us_p50": int(round(float(np.percentile(matched_deltas, 50)))),
        "matched_delta_us_p90": int(round(float(np.percentile(matched_deltas, 90)))),
        "matched_delta_us_max": int(max(matched_deltas)),
    }


def serialize_extrinsics_pose_v2(
    *,
    output_path: str | Path,
    pose_log_path: str | Path,
    camera_order: Sequence[str],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    focal_scales: Dict[str, float],
    per_camera_median_reproj_px: Dict[str, float],
    solve_summary: Dict[str, Any],
    metric_payload: Dict[str, Any] | None = None,
    world_payload: Dict[str, Any] | None = None,
    wand_metric_log_path: str | Path | None = None,
) -> Dict[str, Any]:
    if metric_payload is None:
        metric_payload = {
            "status": "unresolved",
            "scale_m_per_unit": None,
            "source": None,
        }
    if world_payload is None:
        world_payload = {
            "status": "unresolved",
            "frame": None,
            "to_world_matrix": None,
            "floor_plane": None,
            "source": None,
        }
    payload = {
        "schema_version": "2.0",
        "method": "reference_pose_capture",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "pose_capture_log_path": _sanitize_path_for_payload(pose_log_path),
        "camera_order": list(camera_order),
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {
                    "camera_id": camera_id,
                    "R": camera_poses[camera_id][0].tolist(),
                    "t": camera_poses[camera_id][1].tolist(),
                    "focal_scale": float(focal_scales[camera_id]),
                    "median_reproj_error_px": (
                        None
                        if camera_id not in per_camera_median_reproj_px
                        else float(per_camera_median_reproj_px[camera_id])
                    ),
                }
                for camera_id in camera_order
            ],
            "solve_summary": solve_summary,
        },
        "metric": metric_payload,
        "world": world_payload,
    }
    if wand_metric_log_path is not None:
        payload["wand_metric_log_path"] = _sanitize_path_for_payload(wand_metric_log_path)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return payload


def _serialize_camera_pose_rows(
    camera_order: Sequence[str],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    focal_scales: Dict[str, float],
    per_camera_median_reproj_px: Dict[str, float],
) -> list[dict[str, Any]]:
    return [
        {
            "camera_id": camera_id,
            "R": camera_poses[camera_id][0].tolist(),
            "t": camera_poses[camera_id][1].tolist(),
            "focal_scale": float(focal_scales[camera_id]),
            "median_reproj_error_px": (
                None
                if camera_id not in per_camera_median_reproj_px
                else float(per_camera_median_reproj_px[camera_id])
            ),
        }
        for camera_id in camera_order
    ]


def _resolved_metric_payload(
    *,
    camera_order: Sequence[str],
    camera_poses_metric: Dict[str, Tuple[np.ndarray, np.ndarray]],
    focal_scales: Dict[str, float],
    per_camera_median_reproj_px: Dict[str, float],
    scale_meta: Dict[str, Any],
    validation: Dict[str, Any],
    wand_metric_log_path: Path,
) -> Dict[str, Any]:
    return {
        "status": "resolved",
        "frame": "metric_camera",
        "scale_m_per_unit": float(scale_meta.get("scale_m_per_unit", 1.0)),
        "source": scale_meta.get("scale_source"),
        "wand_metric_log_path": _sanitize_path_for_payload(wand_metric_log_path),
        "wand_metric_frames": int(scale_meta.get("wand_metric_frames", 0) or 0),
        "camera_poses": _serialize_camera_pose_rows(
            camera_order,
            camera_poses_metric,
            focal_scales,
            validation.get("per_camera_median_reproj_px", {}) if isinstance(validation, dict) else {},
        ),
        "validation": {
            key: validation.get(key)
            for key in (
                "median_reproj_error_px",
                "p90_reproj_error_px",
                "positive_depth_ratio",
                "baseline_range_m",
                "floor_residual_mm",
                "world_up_consistency",
            )
            if isinstance(validation, dict) and key in validation
        },
        "shape_rms_error_mm": scale_meta.get("shape_rms_error_mm"),
        "wand_face": scale_meta.get("wand_face"),
    }


def _resolved_world_payload(
    *,
    to_world_coords_matrix: np.ndarray,
    scale_meta: Dict[str, Any],
    validation: Dict[str, Any],
) -> Dict[str, Any]:
    up_axis = str(scale_meta.get("up_axis", "Z")).upper()
    floor_normal = [0.0, 0.0, 1.0] if up_axis == "Z" else [0.0, 1.0, 0.0]
    return {
        "status": "resolved",
        "frame": "world",
        "to_world_matrix": to_world_coords_matrix.tolist(),
        "floor_plane": {
            "normal": floor_normal,
            "offset": 0.0,
            "axis": up_axis,
        },
        "source": scale_meta.get("floor_source"),
        "origin_world": scale_meta.get("origin_world"),
        "origin_marker": scale_meta.get("origin_marker"),
        "aligned_axis_world": scale_meta.get("aligned_axis_world"),
        "wand_face": scale_meta.get("wand_face"),
        "floor_normal_sign_source": scale_meta.get("floor_normal_sign_source"),
        "wand_face_alignment": scale_meta.get("wand_face_alignment"),
        "validation": {
            key: validation.get(key)
            for key in ("floor_residual_mm", "world_up_consistency")
            if isinstance(validation, dict) and key in validation
        },
    }


def solve_extrinsics(
    *,
    intrinsics_path: str | Path,
    pose_log_path: str | Path,
    output_path: str | Path = DEFAULT_OUTPUT,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    min_pairs: int = DEFAULT_MIN_PAIRS,
    wand_metric_log_path: str | Path | None = None,
    wand_pair_window_us: int = DEFAULT_WAND_PAIR_WINDOW_US,
    wand_face: str = WAND_FACE_FRONT_UP,
) -> Dict[str, Any]:
    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    camera_params = {
        camera_id: CalibrationLoader.to_camera_params(payload)
        for camera_id, payload in calibrations.items()
    }
    observations_by_camera = load_pose_capture_rows(pose_log_path)
    active_observations = {
        camera_id: rows
        for camera_id, rows in observations_by_camera.items()
        if camera_id in camera_params and rows
    }
    if len(active_observations) < 2:
        raise ValueError("At least two cameras with intrinsics and single-blob pose_capture observations are required")

    camera_order, rows, row_summary = build_camera_order_and_rows(active_observations, pair_window_us)
    camera_params = {camera_id: camera_params[camera_id] for camera_id in camera_order}
    camera_poses_init = initialize_camera_poses_chain(camera_order, rows, camera_params, min_pairs)
    camera_poses, focal_scales, ba_summary = bundle_adjust_camera_poses(
        camera_order,
        rows,
        camera_params,
        camera_poses_init,
    )
    per_camera_median_reproj_px, validation = _compute_reprojection_statistics(
        camera_order,
        rows,
        camera_params,
        camera_poses,
        focal_scales,
    )
    solve_summary = {
        **row_summary,
        **ba_summary,
        "usable_rows": int(validation["usable_rows"]),
        "median_reproj_error_px": validation["median_reproj_error_px"],
        "p90_reproj_error_px": validation["p90_reproj_error_px"],
        **_compute_matching_statistics(rows),
    }

    metric_payload: Dict[str, Any] | None = None
    world_payload: Dict[str, Any] | None = None
    resolved_wand_metric_log: Path | None = None
    if wand_metric_log_path is not None:
        resolved_wand_metric_log = Path(wand_metric_log_path)
        wand_observations = load_wand_metric_observations(resolved_wand_metric_log)
        camera_poses_metric, to_world_coords_matrix, scale_meta = apply_wand_metric_alignment(
            camera_params=camera_params,
            camera_poses_similarity=camera_poses,
            wand_observations_by_camera=wand_observations,
            wand_points_mm=WAND_POINTS_MM,
            pair_window_us=wand_pair_window_us,
            wand_face=wand_face,
            focal_scales=focal_scales,
        )
        if scale_meta.get("scale_source") != "none":
            metric_validation = validate_wand_metric_extrinsics(
                camera_params=camera_params,
                camera_poses=camera_poses_metric,
                wand_observations_by_camera=wand_observations,
                wand_points_mm=WAND_POINTS_MM,
                pair_window_us=wand_pair_window_us,
                focal_scales=focal_scales,
                to_world_matrix=to_world_coords_matrix,
            )
            metric_payload = _resolved_metric_payload(
                camera_order=camera_order,
                camera_poses_metric=camera_poses_metric,
                focal_scales=focal_scales,
                per_camera_median_reproj_px=per_camera_median_reproj_px,
                scale_meta=scale_meta,
                validation=metric_validation,
                wand_metric_log_path=resolved_wand_metric_log,
            )
            world_payload = _resolved_world_payload(
                to_world_coords_matrix=to_world_coords_matrix,
                scale_meta=scale_meta,
                validation=metric_validation,
            )
    return serialize_extrinsics_pose_v2(
        output_path=output_path,
        pose_log_path=pose_log_path,
        camera_order=camera_order,
        camera_poses=camera_poses,
        focal_scales=focal_scales,
        per_camera_median_reproj_px=per_camera_median_reproj_px,
        solve_summary=solve_summary,
        metric_payload=metric_payload,
        world_payload=world_payload,
        wand_metric_log_path=resolved_wand_metric_log,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Reference-style pose capture extrinsics solver")
    parser.add_argument("--intrinsics", default="calibration", help="Intrinsics directory or file")
    parser.add_argument("--pose-log", default="logs/extrinsics_pose_capture.jsonl", help="Pose capture JSONL path")
    parser.add_argument("--wand-metric-log", default=None, help="Optional wand metric/floor JSONL path")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output JSON path")
    parser.add_argument("--pair-window-us", type=int, default=DEFAULT_PAIR_WINDOW_US, help="Timestamp pairing window")
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS, help="Minimum adjacent pair rows")
    parser.add_argument("--wand-pair-window-us", type=int, default=DEFAULT_WAND_PAIR_WINDOW_US, help="Timestamp pairing window for wand metric rows")
    parser.add_argument(
        "--wand-face",
        choices=VALID_WAND_FACE_CONSTRAINTS,
        default=WAND_FACE_FRONT_UP,
        help="Which physical wand side is facing upward during floor / metric capture",
    )
    args = parser.parse_args(argv)

    result = solve_extrinsics(
        intrinsics_path=args.intrinsics,
        pose_log_path=args.pose_log,
        output_path=args.output,
        pair_window_us=args.pair_window_us,
        min_pairs=args.min_pairs,
        wand_metric_log_path=args.wand_metric_log,
        wand_pair_window_us=args.wand_pair_window_us,
        wand_face=args.wand_face,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
