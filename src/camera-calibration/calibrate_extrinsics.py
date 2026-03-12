#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

SRC_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parent
for path in (SRC_ROOT, MODULE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from extrinsics_ba import run_pose_bundle_adjustment
from extrinsics_capture import (
    load_pose_capture_observations,
    load_wand_metric_observations,
    summarize_pose_capture,
)
from extrinsics_initializer import build_initial_camera_poses, estimate_pairwise_initialization
from extrinsics_samples import build_pose_multiview_samples
from extrinsics_scale import apply_wand_metric_alignment
from extrinsics_validate import validate_extrinsics
from host.geo import CalibrationLoader
from wand_bundle_adjustment import run_joint_bundle_adjustment, run_scale_bundle_adjustment
from wand_model import WAND_POINTS_MM, wand_payload
from wand_samples import MultiViewSample


DEFAULT_PAIR_WINDOW_US = 8000
DEFAULT_MIN_PAIRS = 8
DEFAULT_MAX_BA_SAMPLES = 80
DEFAULT_MAX_WAND_METRIC_SAMPLES = 16
DEFAULT_OUTPUT = "calibration/calibration_extrinsics_v1.json"


def _current_wand_payload() -> Dict[str, Any]:
    return wand_payload()


def _to_cameras_output(
    camera_ids: List[str],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    reference_camera_id: str,
    sample_count: int,
    per_camera_median_reproj_px: Dict[str, float],
) -> List[Dict[str, Any]]:
    centers = {
        camera_id: (-(rotation.T @ translation.reshape(3, 1))).reshape(3)
        for camera_id, (rotation, translation) in camera_poses.items()
    }
    ref_center = centers.get(reference_camera_id, np.zeros(3, dtype=np.float64))
    rows: List[Dict[str, Any]] = []
    for camera_id in camera_ids:
        rotation, translation = camera_poses[camera_id]
        center = centers[camera_id]
        baseline = float(np.linalg.norm(center - ref_center)) if camera_id != reference_camera_id else 0.0
        rows.append(
            {
                "camera_id": camera_id,
                "rotation_matrix": rotation.tolist(),
                "translation_m": translation.tolist(),
                "quality": {
                    "pair_count": 0 if camera_id == reference_camera_id else int(sample_count),
                    "point_count": 0 if camera_id == reference_camera_id else int(sample_count),
                    "inlier_ratio": 1.0,
                    "median_reproj_error_px": float(per_camera_median_reproj_px.get(camera_id, 0.0)),
                    "baseline_m": baseline,
                    "scale_m_per_unit": 1.0,
                },
            }
        )
    return rows


def _serialize_result(
    *,
    output_path: str | Path,
    reference_camera_id: str,
    camera_ids: List[str],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    sample_count: int,
    per_camera_median_reproj_px: Dict[str, float],
    session_meta: Dict[str, Any],
    wand_payload: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    cameras_output = _to_cameras_output(
        camera_ids=camera_ids,
        camera_poses=camera_poses,
        reference_camera_id=reference_camera_id,
        sample_count=int(sample_count),
        per_camera_median_reproj_px=per_camera_median_reproj_px,
    )
    result = {
        "schema_version": "1.0",
        "reference_camera_id": reference_camera_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "wand": wand_payload
        or _current_wand_payload(),
        "session_meta": session_meta,
        "cameras": cameras_output,
    }
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def _camera_poses_from_payload(payload: Dict[str, Any]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    rows = payload.get("cameras")
    if not isinstance(rows, list):
        raise ValueError("extrinsics payload is missing cameras")
    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for row in rows:
        if not isinstance(row, dict):
            continue
        camera_id = str(row.get("camera_id", "")).strip()
        rotation = np.asarray(row.get("rotation_matrix"), dtype=np.float64)
        translation = np.asarray(row.get("translation_m"), dtype=np.float64)
        if not camera_id or rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError(f"invalid camera pose payload for '{camera_id or 'unknown'}'")
        poses[camera_id] = (rotation, translation)
    if len(poses) < 2:
        raise ValueError("extrinsics payload must contain at least two camera poses")
    return poses


def _downsample_samples_uniform(samples: List[Any], max_samples: int) -> List[Any]:
    if max_samples <= 0 or len(samples) <= max_samples:
        return list(samples)
    if max_samples == 1:
        return [samples[len(samples) // 2]]
    indices = np.linspace(0, len(samples) - 1, num=max_samples)
    picked = []
    used: set[int] = set()
    for idx_float in indices:
        idx = int(round(float(idx_float)))
        idx = max(0, min(len(samples) - 1, idx))
        if idx in used:
            continue
        used.add(idx)
        picked.append(samples[idx])
    return picked


def _pick_middle_samples(samples: List[Any], max_samples: int) -> List[Any]:
    if max_samples <= 0 or len(samples) <= max_samples:
        return list(samples)
    mid = len(samples) // 2
    if max_samples == 1:
        return [samples[mid]]
    start = max(0, mid - (max_samples // 2))
    end = min(len(samples), start + max_samples)
    return list(samples[start:end])


def _invert_camera_extrinsics(rotation: np.ndarray, translation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rotation_inv = rotation.T
    translation_inv = -(rotation_inv @ translation.reshape(3, 1)).reshape(3)
    return rotation_inv, translation_inv


def _build_wand_metric_samples(
    wand_observations_by_camera: Dict[str, List[Any]],
    reference_camera_id: str,
    pair_window_us: int,
) -> List[MultiViewSample]:
    refs = list(wand_observations_by_camera.get(reference_camera_id, []))
    refs.sort(key=lambda item: item.timestamp)
    camera_ids = sorted(camera_id for camera_id in wand_observations_by_camera.keys() if camera_id != reference_camera_id)
    cursors = {camera_id: 0 for camera_id in camera_ids}
    samples: List[MultiViewSample] = []
    for ref in refs:
        ref_confidence = float(getattr(ref, "confidence", 0.0))
        if ref_confidence <= 0.0:
            continue
        timestamps = {reference_camera_id: int(ref.timestamp)}
        points_by_camera = {reference_camera_id: ref.image_points.astype(np.float64)}
        raw_points_by_camera = {
            reference_camera_id: np.asarray(
                getattr(ref, "raw_points", ref.image_points),
                dtype=np.float64,
            )
        }
        confidence_by_camera = {reference_camera_id: ref_confidence}
        for camera_id in camera_ids:
            rows = wand_observations_by_camera.get(camera_id, [])
            idx = cursors[camera_id]
            while idx < len(rows) and rows[idx].timestamp < ref.timestamp - pair_window_us:
                idx += 1
            best_idx = -1
            best_delta = pair_window_us + 1
            scan = idx
            while scan < len(rows):
                row = rows[scan]
                delta = abs(int(row.timestamp) - int(ref.timestamp))
                if row.timestamp > ref.timestamp + pair_window_us:
                    break
                if delta < best_delta:
                    best_delta = delta
                    best_idx = scan
                scan += 1
            cursors[camera_id] = idx
            if best_idx < 0:
                continue
            cursors[camera_id] = best_idx + 1
            row = rows[best_idx]
            row_confidence = float(getattr(row, "confidence", 0.0))
            if row_confidence <= 0.0:
                continue
            timestamps[camera_id] = int(row.timestamp)
            points_by_camera[camera_id] = row.image_points.astype(np.float64)
            raw_points_by_camera[camera_id] = np.asarray(
                getattr(row, "raw_points", row.image_points),
                dtype=np.float64,
            )
            confidence_by_camera[camera_id] = row_confidence
        if len(points_by_camera) < 2:
            continue
        if max(timestamps.values()) - min(timestamps.values()) > pair_window_us:
            continue
        samples.append(
            MultiViewSample(
                sample_id=len(samples),
                timestamps=timestamps,
                image_points_by_camera=points_by_camera,
                raw_points_by_camera=raw_points_by_camera,
                confidence_by_camera=confidence_by_camera,
            )
        )
    return samples


def _estimate_similarity_transform_local(
    reconstructed: np.ndarray,
    model: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray, float] | None:
    if reconstructed.shape != model.shape or reconstructed.shape[0] < 2:
        return None
    rec_center = np.mean(reconstructed, axis=0, keepdims=True)
    model_center = np.mean(model, axis=0, keepdims=True)
    rec_zero = reconstructed - rec_center
    model_zero = model - model_center
    rec_var = float(np.mean(np.sum(rec_zero * rec_zero, axis=1)))
    if rec_var <= 1e-12:
        return None
    covariance = (model_zero.T @ rec_zero) / float(reconstructed.shape[0])
    u, singular_values, vt = np.linalg.svd(covariance)
    sign = np.eye(3, dtype=np.float64)
    if np.linalg.det(u @ vt) < 0.0:
        sign[-1, -1] = -1.0
    rotation = u @ sign @ vt
    scale = float(np.sum(np.diag(sign) * singular_values) / rec_var)
    if not np.isfinite(scale) or scale <= 0.0:
        return None
    translation = (model_center.reshape(3) - scale * (rotation @ rec_center.reshape(3, 1)).reshape(3)).astype(np.float64)
    aligned = (scale * (rotation @ reconstructed.T)).T + translation.reshape(1, 3)
    rms_error_m = float(np.sqrt(np.mean(np.sum((aligned - model) ** 2, axis=1))))
    return scale, rotation, translation, rms_error_m


def _triangulate_point_from_pose_seed(
    observations: Sequence[tuple[str, np.ndarray]],
    camera_params: Dict[str, Any],
    camera_poses_seed: Dict[str, Tuple[np.ndarray, np.ndarray]],
) -> np.ndarray | None:
    if len(observations) < 2:
        return None
    rows: List[np.ndarray] = []
    for camera_id, uv in observations:
        camera = camera_params[camera_id]
        rotation, translation = camera_poses_seed[camera_id]
        pmat = np.hstack([rotation, translation.reshape(3, 1)])
        norm = cv2.undistortPoints(
            np.asarray(uv, dtype=np.float64).reshape(1, 1, 2),
            camera.intrinsic_matrix.astype(np.float64),
            camera.distortion_coeffs.astype(np.float64),
        ).reshape(2)
        rows.append(norm[0] * pmat[2] - pmat[0])
        rows.append(norm[1] * pmat[2] - pmat[1])
    a = np.stack(rows, axis=0)
    _, _, vt = np.linalg.svd(a, full_matrices=False)
    xh = vt[-1]
    if abs(float(xh[3])) < 1e-12:
        return None
    return (xh[:3] / xh[3]).astype(np.float64)


def _joint_wand_shape_score(
    points_by_camera: Dict[str, np.ndarray],
    camera_params: Dict[str, Any],
    camera_poses_seed: Dict[str, Tuple[np.ndarray, np.ndarray]],
    object_points_m: np.ndarray,
) -> float:
    tri_points: List[np.ndarray] = []
    per_camera_reproj: List[float] = []
    positive_depth = 0
    depth_total = 0
    for point_idx in range(object_points_m.shape[0]):
        tri = _triangulate_point_from_pose_seed(
            [(camera_id, points[point_idx]) for camera_id, points in points_by_camera.items()],
            camera_params=camera_params,
            camera_poses_seed=camera_poses_seed,
        )
        if tri is None:
            return float("inf")
        tri_points.append(tri)
    tri = np.asarray(tri_points, dtype=np.float64)
    fit = _estimate_similarity_transform_local(tri, object_points_m)
    if fit is None:
        return float("inf")
    scale_fit, _rotation_fit, _translation_fit, shape_error_m = fit
    rec_edges = np.array(
        [np.linalg.norm(tri[a] - tri[b]) for a, b in ((0, 1), (0, 2), (0, 3), (2, 3))],
        dtype=np.float64,
    )
    known_edges = np.array(
        [np.linalg.norm(object_points_m[a] - object_points_m[b]) for a, b in ((0, 1), (0, 2), (0, 3), (2, 3))],
        dtype=np.float64,
    )
    valid = rec_edges > 1e-9
    if np.count_nonzero(valid) < 3:
        return float("inf")
    edge_rel_err = np.abs((scale_fit * rec_edges[valid]) - known_edges[valid]) / np.maximum(known_edges[valid], 1e-9)
    for camera_id, image_points in points_by_camera.items():
        rotation, translation = camera_poses_seed[camera_id]
        camera = camera_params[camera_id]
        rvec, _ = cv2.Rodrigues(rotation.astype(np.float64))
        proj, _ = cv2.projectPoints(
            tri.astype(np.float64),
            rvec.reshape(3, 1),
            translation.reshape(3, 1).astype(np.float64),
            camera.intrinsic_matrix.astype(np.float64),
            camera.distortion_coeffs.astype(np.float64),
        )
        pred = proj.reshape(-1, 2)
        per_camera_reproj.append(float(np.median(np.linalg.norm(pred - image_points.astype(np.float64), axis=1))))
        depth = (rotation @ tri.T + translation.reshape(3, 1))[2]
        positive_depth += int(np.count_nonzero(depth > 0.0))
        depth_total += int(depth.shape[0])
    depth_ratio = float(positive_depth / depth_total) if depth_total else 0.0
    depth_penalty = max(0.0, 0.95 - depth_ratio) * 2000.0
    return (shape_error_m * 1000.0) + (150.0 * float(np.median(edge_rel_err))) + (2.0 * float(np.median(np.asarray(per_camera_reproj, dtype=np.float64)))) + depth_penalty


def _build_camera_point_candidates(
    sample: MultiViewSample,
    camera_id: str,
) -> List[np.ndarray]:
    candidates: List[np.ndarray] = []
    seen: set[tuple[float, ...]] = set()

    def _append(points: np.ndarray) -> None:
        arr = np.asarray(points, dtype=np.float64)
        if arr.shape != (4, 2):
            return
        key = tuple(np.round(arr.reshape(-1), 6).tolist())
        if key in seen:
            return
        seen.add(key)
        candidates.append(arr)

    current = sample.image_points_by_camera.get(camera_id)
    if current is not None:
        _append(current)
    raw_points = None if sample.raw_points_by_camera is None else sample.raw_points_by_camera.get(camera_id)
    if raw_points is not None and np.asarray(raw_points).shape == (4, 2):
        raw_points = np.asarray(raw_points, dtype=np.float64)
        for perm in itertools.permutations(range(4)):
            _append(raw_points[list(perm)])
    return candidates


def _solve_pnp_candidates(
    camera: Any,
    image_points: np.ndarray,
    object_points_m: np.ndarray,
) -> List[tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
    out: List[tuple[np.ndarray, np.ndarray, np.ndarray, float]] = []
    generic = getattr(cv2, "solvePnPGeneric", None)
    if callable(generic):
        try:
            result = generic(
                object_points_m.astype(np.float64),
                image_points.astype(np.float64),
                camera.intrinsic_matrix.astype(np.float64),
                camera.distortion_coeffs.astype(np.float64),
                flags=getattr(cv2, "SOLVEPNP_IPPE", cv2.SOLVEPNP_ITERATIVE),
            )
            if isinstance(result, tuple) and len(result) >= 3:
                ok = bool(result[0])
                rvecs = result[1] if len(result) > 1 else []
                tvecs = result[2] if len(result) > 2 else []
                if ok:
                    for rvec, tvec in zip(rvecs, tvecs):
                        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
                        translation = np.asarray(tvec, dtype=np.float64).reshape(3)
                        center = (-(rotation.T @ translation.reshape(3, 1))).reshape(3)
                        proj, _ = cv2.projectPoints(
                            object_points_m.astype(np.float64),
                            np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                            translation.reshape(3, 1),
                            camera.intrinsic_matrix.astype(np.float64),
                            camera.distortion_coeffs.astype(np.float64),
                        )
                        pred = proj.reshape(-1, 2)
                        reproj = float(np.median(np.linalg.norm(pred - image_points, axis=1)))
                        out.append((rotation, translation, center.astype(np.float64), reproj))
        except Exception:
            pass
    if not out:
        ok, rvec, tvec = cv2.solvePnP(
            object_points_m.astype(np.float64),
            image_points.astype(np.float64),
            camera.intrinsic_matrix.astype(np.float64),
            camera.distortion_coeffs.astype(np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if ok:
            rotation, _ = cv2.Rodrigues(rvec)
            translation = np.asarray(tvec, dtype=np.float64).reshape(3)
            center = (-(rotation.T @ translation.reshape(3, 1))).reshape(3)
            proj, _ = cv2.projectPoints(
                object_points_m.astype(np.float64),
                np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                translation.reshape(3, 1),
                camera.intrinsic_matrix.astype(np.float64),
                camera.distortion_coeffs.astype(np.float64),
            )
            pred = proj.reshape(-1, 2)
            reproj = float(np.median(np.linalg.norm(pred - image_points, axis=1)))
            out.append((rotation, translation, center.astype(np.float64), reproj))
    out.sort(key=lambda item: item[3])
    return out


def _best_joint_permutation_for_sample(
    sample: MultiViewSample,
    camera_params: Dict[str, Any],
    camera_poses_seed: Dict[str, Tuple[np.ndarray, np.ndarray]],
    object_points_m: np.ndarray,
) -> Dict[str, np.ndarray]:
    camera_ids = [camera_id for camera_id in sorted(sample.image_points_by_camera.keys()) if camera_id in camera_params]
    if len(camera_ids) < 2:
        return dict(sample.image_points_by_camera)
    candidate_maps: Dict[str, List[np.ndarray]] = {}
    for camera_id in camera_ids:
        candidates = _build_camera_point_candidates(sample, camera_id)
        if not candidates:
            return dict(sample.image_points_by_camera)
        candidate_maps[camera_id] = candidates[:12]

    best_score = float("inf")
    best_points = dict(sample.image_points_by_camera)
    candidate_lists = [candidate_maps[camera_id] for camera_id in camera_ids]
    max_combinations = 4096
    total_combinations = 1
    for candidates in candidate_lists:
        total_combinations *= max(1, len(candidates))
    if total_combinations > max_combinations:
        candidate_lists = [candidates[:6] for candidates in candidate_lists]

    for combo in itertools.product(*candidate_lists):
        trial = {camera_id: points for camera_id, points in zip(camera_ids, combo)}
        score = _joint_wand_shape_score(
            trial,
            camera_params=camera_params,
            camera_poses_seed=camera_poses_seed,
            object_points_m=object_points_m,
        )
        if score < best_score:
            best_score = score
            best_points = trial
    out = dict(sample.image_points_by_camera)
    out.update(best_points)
    return out


def _relabel_wand_samples_with_pnp(
    samples: List[MultiViewSample],
    camera_params: Dict[str, Any],
    camera_poses_seed: Dict[str, Tuple[np.ndarray, np.ndarray]],
    object_points_m: np.ndarray,
) -> List[MultiViewSample]:
    relabeled: List[MultiViewSample] = []
    for sample in samples:
        raw_points_by_camera = dict(sample.raw_points_by_camera) if sample.raw_points_by_camera else None
        confidence_by_camera = dict(sample.confidence_by_camera) if sample.confidence_by_camera else None
        image_points_by_camera = _best_joint_permutation_for_sample(
            sample,
            camera_params=camera_params,
            camera_poses_seed=camera_poses_seed,
            object_points_m=object_points_m,
        )
        relabeled.append(
            MultiViewSample(
                sample_id=sample.sample_id,
                timestamps=dict(sample.timestamps),
                image_points_by_camera=image_points_by_camera,
                raw_points_by_camera=dict(raw_points_by_camera) if raw_points_by_camera else None,
                confidence_by_camera=confidence_by_camera,
            )
        )
    return relabeled


def _init_wand_pose_for_sample(
    sample: MultiViewSample,
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    object_points_m: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    best: Tuple[np.ndarray, np.ndarray] | None = None
    for camera_id, image_points in sample.image_points_by_camera.items():
        camera = camera_params[camera_id]
        ok, rvec_oc, tvec_oc = cv2.solvePnP(
            object_points_m.astype(np.float64),
            image_points.astype(np.float64),
            camera.intrinsic_matrix.astype(np.float64),
            camera.distortion_coeffs.astype(np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue
        rotation_oc, _ = cv2.Rodrigues(rvec_oc)
        rotation_cw, translation_cw = _invert_camera_extrinsics(*camera_poses[camera_id])
        rotation_wo = rotation_cw @ rotation_oc
        translation_wo = (rotation_cw @ tvec_oc.reshape(3, 1) + translation_cw.reshape(3, 1)).reshape(3)
        best = (rotation_wo, translation_wo)
        break
    if best is None:
        return np.eye(3, dtype=np.float64), np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return best


def _project_rotation_to_so3(rotation: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(rotation)
    proj = u @ vt
    if np.linalg.det(proj) < 0.0:
        u[:, -1] *= -1.0
        proj = u @ vt
    return proj


def _scale_similarity_camera_poses(
    camera_poses_similarity: Dict[str, Tuple[np.ndarray, np.ndarray]],
    scale: float,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    out: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for camera_id, (rotation, translation) in camera_poses_similarity.items():
        center = (-(rotation.T @ translation.reshape(3, 1))).reshape(3)
        center_scaled = center * float(scale)
        out[camera_id] = (rotation.astype(np.float64), (-(rotation @ center_scaled.reshape(3, 1))).reshape(3))
    return out


def _estimate_metric_camera_poses_from_wand(
    camera_params: Dict[str, Any],
    reference_camera_id: str,
    wand_observations_by_camera: Dict[str, List[Any]],
    pair_window_us: int,
    camera_pose_hint: Dict[str, Tuple[np.ndarray, np.ndarray]] | None = None,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]] | None:
    samples = _build_wand_metric_samples(wand_observations_by_camera, reference_camera_id, pair_window_us)
    if len(samples) < 4:
        return None
    object_points_m = np.asarray(WAND_POINTS_MM, dtype=np.float64) / 1000.0
    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        reference_camera_id: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
    }
    for camera_id in sorted(camera_params.keys()):
        if camera_id == reference_camera_id:
            continue
        rot_rows: List[np.ndarray] = []
        trans_rows: List[np.ndarray] = []
        for sample in samples:
            if reference_camera_id not in sample.image_points_by_camera or camera_id not in sample.image_points_by_camera:
                continue
            ref_camera = camera_params[reference_camera_id]
            other_camera = camera_params[camera_id]
            ref_solutions = _solve_pnp_candidates(
                ref_camera,
                sample.image_points_by_camera[reference_camera_id].astype(np.float64),
                object_points_m,
            )
            other_solutions = _solve_pnp_candidates(
                other_camera,
                sample.image_points_by_camera[camera_id].astype(np.float64),
                object_points_m,
            )
            if not ref_solutions or not other_solutions:
                continue
            best_pair: tuple[np.ndarray, np.ndarray] | None = None
            best_pair_score = float("inf")
            for r_ref, t_ref, _center_ref, reproj_ref in ref_solutions[:4]:
                for r_other, t_other, _center_other, reproj_other in other_solutions[:4]:
                    r_other_ref = r_other @ r_ref.T
                    t_other_ref = t_other.reshape(3) - (r_other_ref @ t_ref.reshape(3, 1)).reshape(3)
                    score = reproj_ref + reproj_other
                    if camera_pose_hint is not None and camera_id in camera_pose_hint:
                        r_hint, t_hint = camera_pose_hint[camera_id]
                        rot_delta = r_other_ref @ r_hint.T
                        trace = float(np.clip((np.trace(rot_delta) - 1.0) * 0.5, -1.0, 1.0))
                        rot_err = float(np.arccos(trace))
                        t_other_norm = float(np.linalg.norm(t_other_ref))
                        t_hint_norm = float(np.linalg.norm(t_hint))
                        dir_err = 0.0
                        if t_other_norm > 1e-9 and t_hint_norm > 1e-9:
                            cos_dir = float(np.clip(np.dot(t_other_ref, t_hint.reshape(3)) / (t_other_norm * t_hint_norm), -1.0, 1.0))
                            dir_err = float(np.arccos(cos_dir))
                        score += (50.0 * rot_err) + (25.0 * dir_err)
                    if score < best_pair_score:
                        best_pair_score = score
                        best_pair = (r_other_ref, t_other_ref)
            if best_pair is None:
                continue
            r_other_ref, t_other_ref = best_pair
            rot_rows.append(r_other_ref)
            trans_rows.append(t_other_ref)
        if len(rot_rows) < 4:
            return None
        rotation_avg = _project_rotation_to_so3(np.mean(np.stack(rot_rows, axis=0), axis=0))
        translation_avg = np.median(np.stack(trans_rows, axis=0), axis=0)
        poses[camera_id] = (rotation_avg, translation_avg.astype(np.float64))
    return poses if len(poses) >= 2 else None


def _validation_score(validation: Dict[str, Any]) -> float:
    median_reproj = float(validation.get("median_reproj_error_px", 1e9) or 1e9)
    floor_residual_mm = float(validation.get("floor_residual_mm", 500.0) or 500.0)
    positive_depth_ratio = float(validation.get("positive_depth_ratio", 0.0) or 0.0)
    depth_penalty = max(0.0, 0.98 - positive_depth_ratio) * 1000.0
    floor_penalty = min(floor_residual_mm, 3000.0) * 0.2
    return median_reproj + floor_penalty + depth_penalty


def solve_pose_capture_extrinsics(
    intrinsics_path: str | Path,
    pose_log_path: str | Path,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    min_pairs: int = DEFAULT_MIN_PAIRS,
    max_ba_samples: int = DEFAULT_MAX_BA_SAMPLES,
    reference_camera_id: Optional[str] = None,
) -> Dict[str, Any]:
    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    if not calibrations:
        raise ValueError("No intrinsic calibration files found")

    observations = load_pose_capture_observations(pose_log_path)
    shared = sorted(camera_id for camera_id in calibrations.keys() if camera_id in observations)
    if len(shared) < 2:
        raise ValueError("At least two cameras with intrinsics and pose_capture observations are required")

    ref_camera_id = reference_camera_id or shared[0]
    if ref_camera_id not in shared:
        raise ValueError(f"Reference camera '{ref_camera_id}' not found in shared inputs")

    camera_params = {
        camera_id: CalibrationLoader.to_camera_params(calibrations[camera_id])
        for camera_id in shared
    }
    samples = build_pose_multiview_samples(
        observations_by_camera=observations,
        reference_camera_id=ref_camera_id,
        pair_window_us=pair_window_us,
    )
    if len(samples) < min_pairs:
        raise ValueError(f"Not enough pose samples: {len(samples)}")

    pair_rows = estimate_pairwise_initialization(
        camera_params=camera_params,
        samples=samples,
        min_pairs=min_pairs,
    )
    poses_init, graph_edges = build_initial_camera_poses(
        reference_camera_id=ref_camera_id,
        camera_ids=shared,
        pair_rows=pair_rows,
    )
    solved_camera_ids = sorted(poses_init.keys())
    if len(solved_camera_ids) < 2:
        raise ValueError("No non-reference cameras were solvable")

    camera_params = {camera_id: camera_params[camera_id] for camera_id in solved_camera_ids}
    samples = [
        sample
        for sample in samples
        if len([camera_id for camera_id in sample.image_points_by_camera.keys() if camera_id in camera_params]) >= 2
    ]
    accepted_sample_count = len(samples)
    samples = _downsample_samples_uniform(samples, max_ba_samples)
    ba = run_pose_bundle_adjustment(
        camera_params=camera_params,
        reference_camera_id=ref_camera_id,
        samples=samples,
        camera_poses_init=poses_init,
        loss="huber",
    )
    pose_summary = summarize_pose_capture(observations)
    return {
        "reference_camera_id": ref_camera_id,
        "camera_ids": solved_camera_ids,
        "camera_poses": ba.camera_poses,
        "camera_graph_edges": graph_edges,
        "sample_count": len(samples),
        "accepted_sample_count": accepted_sample_count,
        "optimizer": {
            "iterations": ba.iterations,
            "cost": ba.cost,
            "initial_cost": ba.initial_cost,
        },
        "capture_summary": pose_summary,
    }


def solve_extrinsics(
    intrinsics_path: str | Path,
    pose_log_path: str | Path,
    output_path: str | Path = DEFAULT_OUTPUT,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    min_pairs: int = DEFAULT_MIN_PAIRS,
    max_ba_samples: int = DEFAULT_MAX_BA_SAMPLES,
    reference_camera_id: Optional[str] = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    pose_result = solve_pose_capture_extrinsics(
        intrinsics_path=intrinsics_path,
        pose_log_path=pose_log_path,
        pair_window_us=pair_window_us,
        min_pairs=min_pairs,
        max_ba_samples=max_ba_samples,
        reference_camera_id=reference_camera_id,
    )

    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    camera_params = {
        camera_id: CalibrationLoader.to_camera_params(calibrations[camera_id])
        for camera_id in pose_result["camera_ids"]
    }
    validation = validate_extrinsics(
        camera_params=camera_params,
        camera_poses=pose_result["camera_poses"],
        wand_observations_by_camera={},
        wand_points_mm=WAND_POINTS_MM,
        pair_window_us=pair_window_us,
        up_axis="Z",
    )
    session_meta = {
        "session_id": session_id,
        "method": "pose_capture",
        "pose_capture_log_path": str(Path(pose_log_path)),
        "wand_metric_log_path": None,
        "pair_window_us": pair_window_us,
        "min_pairs": min_pairs,
        "max_ba_samples": max_ba_samples,
        "sample_count": int(pose_result["sample_count"]),
        "accepted_sample_count": int(pose_result["accepted_sample_count"]),
        "camera_graph_edges": pose_result["camera_graph_edges"],
        "optimizer": pose_result["optimizer"],
        "scale_source": "none",
        "floor_source": "none",
        "scale_m_per_unit": 1.0,
        "wand_metric_frames": 0,
        "validation": validation,
        "capture_summary": pose_result["capture_summary"],
    }
    return _serialize_result(
        output_path=output_path,
        reference_camera_id=pose_result["reference_camera_id"],
        camera_ids=pose_result["camera_ids"],
        camera_poses=pose_result["camera_poses"],
        sample_count=int(pose_result["sample_count"]),
        per_camera_median_reproj_px=dict(validation.get("per_camera_median_reproj_px", {})),
        session_meta=session_meta,
    )


def apply_wand_scale_floor(
    intrinsics_path: str | Path,
    extrinsics_path: str | Path,
    wand_metric_log_path: str | Path,
    output_path: str | Path | None = None,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    max_wand_metric_samples: int = DEFAULT_MAX_WAND_METRIC_SAMPLES,
    expected_baseline_m: float | None = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    payload = json.loads(Path(extrinsics_path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("extrinsics payload is invalid")
    camera_poses_similarity = _camera_poses_from_payload(payload)
    camera_ids = [str(row.get("camera_id")) for row in payload.get("cameras", []) if isinstance(row, dict)]
    reference_camera_id = str(payload.get("reference_camera_id", "")).strip()
    if reference_camera_id not in camera_poses_similarity:
        raise ValueError("reference camera is missing from extrinsics payload")
    camera_params = {
        camera_id: CalibrationLoader.to_camera_params(calibrations[camera_id])
        for camera_id in camera_ids
        if camera_id in camera_poses_similarity and camera_id in calibrations
    }
    if len(camera_params) < 2:
        raise ValueError("intrinsics are missing for one or more extrinsics cameras")

    wand_obs = load_wand_metric_observations(wand_metric_log_path)
    wand_metric_samples_all = _build_wand_metric_samples(wand_obs, reference_camera_id, pair_window_us=pair_window_us)
    wand_metric_samples = _pick_middle_samples(wand_metric_samples_all, max_wand_metric_samples)
    if not wand_metric_samples:
        raise ValueError("No valid wand metric samples were built from the wand metric log")
    object_points_m = np.asarray(WAND_POINTS_MM, dtype=np.float64) / 1000.0
    wand_metric_samples = _relabel_wand_samples_with_pnp(
        wand_metric_samples,
        camera_params=camera_params,
        camera_poses_seed=camera_poses_similarity,
        object_points_m=object_points_m,
    )
    wand_obs_trimmed: Dict[str, List[Any]] = {}
    for sample in wand_metric_samples:
        for camera_id, points in sample.image_points_by_camera.items():
            wand_obs_trimmed.setdefault(camera_id, []).append(
                type("WandObsRow", (), {
                    "camera_id": camera_id,
                    "timestamp": sample.timestamps[camera_id],
                    "frame_index": 0,
                    "image_points": points,
                    "confidence": float((sample.confidence_by_camera or {}).get(camera_id, 0.0)),
                })()
            )
    metric_camera_poses_from_wand = _estimate_metric_camera_poses_from_wand(
        camera_params=camera_params,
        reference_camera_id=reference_camera_id,
        wand_observations_by_camera=wand_obs_trimmed,
        pair_window_us=pair_window_us,
        camera_pose_hint=camera_poses_similarity,
    )
    candidate_seeds: List[Tuple[str, Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[str, Any]]] = []
    if metric_camera_poses_from_wand is not None:
        candidate_seeds.append((
            "wand_pnp",
            metric_camera_poses_from_wand,
            {},
        ))
    candidate_seeds.append(("pose_capture_similarity", camera_poses_similarity, {}))

    best_candidate: Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[str, Any], Dict[str, Any], float] | None = None
    for metric_pose_source, pose_seed, summary_override in candidate_seeds:
        aligned_poses, aligned_summary = apply_wand_metric_alignment(
            camera_params=camera_params,
            camera_poses_similarity=pose_seed,
            wand_observations_by_camera=wand_obs_trimmed,
            wand_points_mm=WAND_POINTS_MM,
            up_axis="Z",
            pair_window_us=pair_window_us,
            assume_metric_scale=(metric_pose_source == "wand_pnp"),
        )
        if aligned_summary.get("scale_source") == "none":
            aligned_summary.update({key: value for key, value in summary_override.items() if value is not None})
        aligned_summary["metric_pose_source"] = metric_pose_source
        aligned_validation = validate_extrinsics(
            camera_params=camera_params,
            camera_poses=aligned_poses,
            wand_observations_by_camera=wand_obs_trimmed,
            wand_points_mm=WAND_POINTS_MM,
            pair_window_us=pair_window_us,
            up_axis="Z",
        )
        if expected_baseline_m is not None:
            aligned_validation["baseline_target_m"] = float(expected_baseline_m)
        aligned_score = _validation_score(aligned_validation)
        aligned_shape_mm = float(aligned_summary.get("shape_rms_error_mm", 0.0) or 0.0)
        if (
            int(aligned_summary.get("wand_metric_frames", 0) or 0) > 0
            and aligned_shape_mm <= 100.0
            and (best_candidate is None or aligned_score < best_candidate[3])
        ):
            best_candidate = (aligned_poses, dict(aligned_summary), aligned_validation, aligned_score)

        if int(aligned_summary.get("wand_metric_frames", 0) or 0) <= 0:
            continue
        if metric_pose_source == "pose_capture_similarity":
            scale_seed = float(aligned_summary.get("scale_m_per_unit", 1.0) or 1.0)
            if scale_seed > 0.0:
                scaled_seed_poses = _scale_similarity_camera_poses(pose_seed, scale_seed)
                wand_poses_scale_init = {
                    sample.sample_id: _init_wand_pose_for_sample(sample, camera_params, scaled_seed_poses, object_points_m)
                    for sample in wand_metric_samples
                }
                scale_ba = run_scale_bundle_adjustment(
                    camera_params=camera_params,
                    reference_camera_id=reference_camera_id,
                    samples=wand_metric_samples,
                    object_points_m=object_points_m,
                    camera_poses_similarity=pose_seed,
                    scale_init=scale_seed,
                    wand_poses_init=wand_poses_scale_init,
                )
                scale_ba_poses, scale_ba_summary = apply_wand_metric_alignment(
                    camera_params=camera_params,
                    camera_poses_similarity=scale_ba.camera_poses,
                    wand_observations_by_camera=wand_obs_trimmed,
                    wand_points_mm=WAND_POINTS_MM,
                    up_axis="Z",
                    pair_window_us=pair_window_us,
                    assume_metric_scale=True,
                )
                if int(scale_ba_summary.get("wand_metric_frames", 0) or 0) > 0:
                    scale_ba_summary["metric_pose_source"] = "pose_capture_scale_ba"
                    scale_ba_summary["scale_ba_iterations"] = int(scale_ba.optimizer_iterations)
                    scale_ba_summary["scale_ba_cost"] = float(scale_ba.optimizer_cost)
                    scale_ba_summary["scale_ba_median_reproj_error_px"] = float(scale_ba.final_median_reproj_error_px)
                    scale_ba_validation = validate_extrinsics(
                        camera_params=camera_params,
                        camera_poses=scale_ba_poses,
                        wand_observations_by_camera=wand_obs_trimmed,
                        wand_points_mm=WAND_POINTS_MM,
                        pair_window_us=pair_window_us,
                        up_axis="Z",
                    )
                    if expected_baseline_m is not None:
                        scale_ba_validation["baseline_target_m"] = float(expected_baseline_m)
                    scale_ba_score = _validation_score(scale_ba_validation)
                    if (
                        float(scale_ba_summary.get("shape_rms_error_mm", 0.0) or 0.0) <= 100.0
                        and float(scale_ba.final_median_reproj_error_px) <= 100.0
                        and (best_candidate is None or scale_ba_score < best_candidate[3])
                    ):
                        best_candidate = (scale_ba_poses, scale_ba_summary, scale_ba_validation, scale_ba_score)
        if len(wand_metric_samples) < 4:
            continue
        wand_poses_init = {
            sample.sample_id: _init_wand_pose_for_sample(sample, camera_params, aligned_poses, object_points_m)
            for sample in wand_metric_samples
        }
        translation_delta = max(
            0.25,
            max(
                (
                    float(np.linalg.norm(translation.reshape(3)))
                    for camera_id, (_rotation, translation) in aligned_poses.items()
                    if camera_id != reference_camera_id
                ),
                default=0.25,
            ),
        )
        wand_ba = run_joint_bundle_adjustment(
            camera_params=camera_params,
            reference_camera_id=reference_camera_id,
            samples=wand_metric_samples,
            object_points_m=object_points_m,
            camera_poses_init=aligned_poses,
            wand_poses_init=wand_poses_init,
            translation_max_delta_m=translation_delta,
        )
        refined_summary = dict(aligned_summary)
        refined_summary["wand_refine_iterations"] = int(wand_ba.optimizer_iterations)
        refined_summary["wand_refine_cost"] = float(wand_ba.optimizer_cost)
        refined_summary["wand_refine_median_reproj_error_px"] = float(wand_ba.final_median_reproj_error_px)
        refined_summary["wand_refine_p90_reproj_error_px"] = float(wand_ba.final_p90_reproj_error_px)
        refined_validation = validate_extrinsics(
            camera_params=camera_params,
            camera_poses=wand_ba.camera_poses,
            wand_observations_by_camera=wand_obs_trimmed,
            wand_points_mm=WAND_POINTS_MM,
            pair_window_us=pair_window_us,
            up_axis="Z",
        )
        if expected_baseline_m is not None:
            refined_validation["baseline_target_m"] = float(expected_baseline_m)
        refined_score = _validation_score(refined_validation)
        refined_shape_mm = float(refined_summary.get("shape_rms_error_mm", 0.0) or 0.0)
        if refined_shape_mm <= 100.0 and (best_candidate is None or refined_score < best_candidate[3]):
            best_candidate = (wand_ba.camera_poses, refined_summary, refined_validation, refined_score)

    if best_candidate is None:
        raise ValueError("Failed to produce a valid wand metric candidate")
    camera_poses_metric, metric_summary, validation, _ = best_candidate
    previous_meta = payload.get("session_meta", {})
    if not isinstance(previous_meta, dict):
        previous_meta = {}
    output_target = Path(output_path) if output_path is not None else Path(extrinsics_path)
    session_meta = {
        **previous_meta,
        "session_id": session_id if session_id is not None else previous_meta.get("session_id"),
        "wand_metric_log_path": str(Path(wand_metric_log_path)),
        "pair_window_us": pair_window_us,
        "max_wand_metric_samples": max_wand_metric_samples,
        "expected_baseline_m": float(expected_baseline_m) if expected_baseline_m is not None else None,
        **metric_summary,
        "scale_m_per_unit": float(metric_summary.get("scale_m_per_unit", 1.0)),
        "wand_metric_frames": int(metric_summary.get("wand_metric_frames", 0)),
        "validation": validation,
    }
    return _serialize_result(
        output_path=output_target,
        reference_camera_id=reference_camera_id,
        camera_ids=camera_ids,
        camera_poses=camera_poses_metric,
        sample_count=int(previous_meta.get("sample_count", 0) or 0),
        per_camera_median_reproj_px=dict(validation.get("per_camera_median_reproj_px", {})),
        session_meta=session_meta,
        wand_payload=_current_wand_payload(),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate camera extrinsics from pose capture logs")
    parser.add_argument("--intrinsics", required=True, help="Path to intrinsic calibration directory or file")
    parser.add_argument("--pose-log", required=True, help="Path to pose capture JSONL log")
    parser.add_argument("--wand-metric-log", default=None, help="Optional wand metric capture JSONL log")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output JSON path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--pair-window-us", type=int, default=DEFAULT_PAIR_WINDOW_US, help="Maximum timestamp delta for pairing")
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS, help="Minimum paired observations per camera pair")
    parser.add_argument("--max-ba-samples", type=int, default=DEFAULT_MAX_BA_SAMPLES, help="Maximum BA samples after uniform downsampling")
    parser.add_argument("--max-wand-metric-samples", type=int, default=DEFAULT_MAX_WAND_METRIC_SAMPLES, help="Maximum wand metric samples; default uses only the middle frame")
    parser.add_argument("--expected-baseline-m", type=float, default=None, help="Optional measured camera baseline used as a validation prior")
    parser.add_argument("--reference-camera", default=None, help="Reference camera ID")
    parser.add_argument("--session-id", default=None, help="Optional session ID for metadata")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = solve_extrinsics(
        intrinsics_path=args.intrinsics,
        pose_log_path=args.pose_log,
        output_path=args.output,
        pair_window_us=args.pair_window_us,
        min_pairs=args.min_pairs,
        max_ba_samples=args.max_ba_samples,
        reference_camera_id=args.reference_camera,
        session_id=args.session_id,
    )
    if args.wand_metric_log:
        result = apply_wand_scale_floor(
            intrinsics_path=args.intrinsics,
            extrinsics_path=args.output,
            wand_metric_log_path=args.wand_metric_log,
            output_path=args.output,
            pair_window_us=args.pair_window_us,
            max_wand_metric_samples=args.max_wand_metric_samples,
            expected_baseline_m=args.expected_baseline_m,
            session_id=args.session_id,
        )
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
