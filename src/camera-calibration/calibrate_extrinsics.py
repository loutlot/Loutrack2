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

SRC_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parent
for path in (SRC_ROOT, MODULE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from host.geo import CalibrationLoader
from host.wand_session import WAND_MARKER_DIAMETER_MM, WAND_NAME, WAND_POINTS_MM
from wand_bundle_adjustment import run_joint_bundle_adjustment
from wand_label import canonicalize_wand_points
from wand_samples import LabeledFrameObservation, MultiViewSample, build_multiview_samples


DEFAULT_PAIR_WINDOW_US = 8000
DEFAULT_MIN_PAIRS = 8
DEFAULT_MAX_BA_SAMPLES = 320
DEFAULT_OUTPUT = "calibration/calibration_extrinsics_v1.json"


@dataclass(frozen=True)
class FrameObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    blobs: List[Dict[str, float]]


def load_wand_log(path: str | Path) -> Dict[str, List[FrameObservation]]:
    observations: Dict[str, List[FrameObservation]] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            entry = json.loads(line)
            if entry.get("_type") == "frame":
                frame = entry.get("data", {})
            elif "camera_id" in entry and "blobs" in entry:
                frame = entry
            else:
                continue
            camera_id = frame.get("camera_id")
            if not camera_id:
                continue
            observations.setdefault(camera_id, []).append(
                FrameObservation(
                    camera_id=str(camera_id),
                    timestamp=int(frame["timestamp"]),
                    frame_index=int(frame.get("frame_index", 0)),
                    blobs=list(frame.get("blobs", [])),
                )
            )
    for camera_id in observations:
        observations[camera_id].sort(key=lambda item: item.timestamp)
    return observations


def label_observations(
    observations: Dict[str, Sequence[FrameObservation]],
) -> Tuple[Dict[str, List[LabeledFrameObservation]], Dict[str, int]]:
    labeled: Dict[str, List[LabeledFrameObservation]] = {}
    dropped_counts = {"not_enough_blobs": 0, "failed_labeling": 0}
    for camera_id, frames in observations.items():
        rows: List[LabeledFrameObservation] = []
        for frame in frames:
            if len(frame.blobs) < len(WAND_POINTS_MM):
                dropped_counts["not_enough_blobs"] += 1
                continue
            label = canonicalize_wand_points(frame.blobs)
            if label is None:
                dropped_counts["failed_labeling"] += 1
                continue
            rows.append(
                LabeledFrameObservation(
                    camera_id=camera_id,
                    timestamp=frame.timestamp,
                    frame_index=frame.frame_index,
                    image_points=label.points,
                    collinearity_error=label.collinearity_error,
                    midpoint_ratio_error=label.midpoint_ratio_error,
                )
            )
        labeled[camera_id] = rows
    return labeled, dropped_counts


def _normalized_points(camera: Any, points: np.ndarray) -> np.ndarray:
    return cv2.undistortPoints(
        points.reshape(-1, 1, 2),
        camera.intrinsic_matrix,
        camera.distortion_coeffs,
    ).reshape(-1, 2)


def _collect_pair_correspondences(
    reference_camera: Any,
    other_camera: Any,
    samples: Sequence[MultiViewSample],
    reference_camera_id: str,
    other_camera_id: str,
) -> Tuple[np.ndarray, np.ndarray]:
    ref_points: List[np.ndarray] = []
    other_points: List[np.ndarray] = []
    for sample in samples:
        if reference_camera_id not in sample.image_points_by_camera:
            continue
        if other_camera_id not in sample.image_points_by_camera:
            continue
        ref_points.append(_normalized_points(reference_camera, sample.image_points_by_camera[reference_camera_id]))
        other_points.append(_normalized_points(other_camera, sample.image_points_by_camera[other_camera_id]))
    if not ref_points:
        return np.empty((0, 2), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    return np.concatenate(ref_points, axis=0), np.concatenate(other_points, axis=0)


def _estimate_pair_pose(
    reference_camera: Any,
    other_camera: Any,
    samples: Sequence[MultiViewSample],
    reference_camera_id: str,
    other_camera_id: str,
    min_pairs: int,
) -> Dict[str, Any]:
    ref_norm, other_norm = _collect_pair_correspondences(
        reference_camera,
        other_camera,
        samples,
        reference_camera_id,
        other_camera_id,
    )
    required_points = max(8, min_pairs * len(WAND_POINTS_MM))
    if ref_norm.shape[0] < required_points:
        raise ValueError(
            f"Not enough paired points for {other_camera_id}: {ref_norm.shape[0]} < {required_points}"
        )

    essential, _ = cv2.findEssentialMat(
        ref_norm,
        other_norm,
        focal=1.0,
        pp=(0.0, 0.0),
        method=cv2.RANSAC,
        prob=0.999,
        threshold=1e-3,
    )
    if essential is None:
        raise ValueError(f"findEssentialMat failed for {other_camera_id}")

    _, rotation, translation_unit, pose_mask = cv2.recoverPose(essential, ref_norm, other_norm)
    inlier_mask = pose_mask.ravel() > 0 if pose_mask is not None else np.ones(ref_norm.shape[0], dtype=bool)

    proj_ref = np.hstack([np.eye(3), np.zeros((3, 1))])
    proj_other = np.hstack([rotation, translation_unit.reshape(3, 1)])
    wand_points_m = np.array(WAND_POINTS_MM, dtype=np.float64) / 1000.0
    edge_pairs = [(i, j) for i, j in combinations(range(wand_points_m.shape[0]), 2)]
    known_edges = np.array(
        [np.linalg.norm(wand_points_m[a] - wand_points_m[b]) for a, b in edge_pairs],
        dtype=np.float64,
    )

    scale_candidates: List[float] = []
    for sample in samples:
        if reference_camera_id not in sample.image_points_by_camera:
            continue
        if other_camera_id not in sample.image_points_by_camera:
            continue
        ref_pts = _normalized_points(reference_camera, sample.image_points_by_camera[reference_camera_id])
        other_pts = _normalized_points(other_camera, sample.image_points_by_camera[other_camera_id])
        points_4d = cv2.triangulatePoints(proj_ref, proj_other, ref_pts.T, other_pts.T)
        points_3d = (points_4d[:3] / points_4d[3]).T
        reconstructed_edges = np.array(
            [np.linalg.norm(points_3d[a] - points_3d[b]) for a, b in edge_pairs],
            dtype=np.float64,
        )
        valid = reconstructed_edges > 1e-9
        if np.count_nonzero(valid) < 2:
            continue
        scale_candidates.append(float(np.median(known_edges[valid] / reconstructed_edges[valid])))
    if not scale_candidates:
        raise ValueError(f"Failed to estimate metric scale for {other_camera_id}")
    scale_m_per_unit = float(np.median(scale_candidates))
    translation_m = translation_unit.reshape(3) * scale_m_per_unit

    return {
        "rotation": rotation,
        "translation": translation_m,
        "essential_matrix": essential,
        "inlier_ratio": float(np.mean(inlier_mask.astype(np.float64))),
        "scale_m_per_unit": scale_m_per_unit,
    }


def _invert_camera_extrinsics(rotation: np.ndarray, translation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rotation_inv = rotation.T
    translation_inv = -(rotation_inv @ translation.reshape(3, 1)).reshape(3)
    return rotation_inv, translation_inv


def _init_wand_pose_for_sample(
    sample: MultiViewSample,
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    object_points_m: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    best = None
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
        if best is None:
            best = (rotation_wo, translation_wo)
    if best is None:
        return np.eye(3, dtype=np.float64), np.array([0.0, 0.0, 1.5], dtype=np.float64)
    return best


def _compute_camera_reprojection_stats(
    camera_params: Dict[str, Any],
    samples: Sequence[MultiViewSample],
    object_points_m: np.ndarray,
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]],
) -> Dict[str, List[float]]:
    errors_by_camera: Dict[str, List[float]] = {camera_id: [] for camera_id in camera_params.keys()}
    for sample in samples:
        wand_pose = wand_poses[sample.sample_id]
        for camera_id, image_points in sample.image_points_by_camera.items():
            camera = camera_params[camera_id]
            R_c, t_c = camera_poses[camera_id]
            R_w, t_w = wand_pose
            R_oc = R_c @ R_w
            t_oc = (R_c @ t_w.reshape(3, 1) + t_c.reshape(3, 1)).reshape(3)
            rvec, _ = cv2.Rodrigues(R_oc)
            proj, _ = cv2.projectPoints(
                object_points_m.astype(np.float64),
                rvec.reshape(3, 1),
                t_oc.reshape(3, 1),
                camera.intrinsic_matrix.astype(np.float64),
                camera.distortion_coeffs.astype(np.float64),
            )
            error = np.linalg.norm(proj.reshape(-1, 2) - image_points, axis=1)
            errors_by_camera[camera_id].extend(error.astype(float).tolist())
    return errors_by_camera


def _filter_samples_to_camera_set(
    samples: Sequence[MultiViewSample],
    camera_ids: Sequence[str],
    reference_camera_id: str,
) -> List[MultiViewSample]:
    keep = set(camera_ids)
    filtered: List[MultiViewSample] = []
    for sample in samples:
        points_by_camera = {
            camera_id: points
            for camera_id, points in sample.image_points_by_camera.items()
            if camera_id in keep
        }
        if reference_camera_id not in points_by_camera:
            continue
        if len(points_by_camera) < 2:
            continue
        timestamps = {
            camera_id: ts
            for camera_id, ts in sample.timestamps.items()
            if camera_id in points_by_camera
        }
        filtered.append(
            MultiViewSample(
                sample_id=len(filtered),
                timestamps=timestamps,
                image_points_by_camera=points_by_camera,
            )
        )
    return filtered


def _downsample_samples_uniform(samples: Sequence[MultiViewSample], max_samples: int) -> List[MultiViewSample]:
    rows = list(samples)
    if max_samples <= 0 or len(rows) <= max_samples:
        return rows
    if max_samples == 1:
        pick = [rows[len(rows) // 2]]
    else:
        step = (len(rows) - 1) / float(max_samples - 1)
        pick = [rows[int(round(i * step))] for i in range(max_samples)]
    out: List[MultiViewSample] = []
    for i, sample in enumerate(pick):
        out.append(
            MultiViewSample(
                sample_id=i,
                timestamps=dict(sample.timestamps),
                image_points_by_camera=dict(sample.image_points_by_camera),
            )
        )
    return out


def solve_wand_extrinsics(
    intrinsics_path: str | Path,
    log_path: str | Path,
    output_path: str | Path = DEFAULT_OUTPUT,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    min_pairs: int = DEFAULT_MIN_PAIRS,
    max_ba_samples: int = DEFAULT_MAX_BA_SAMPLES,
    reference_camera_id: Optional[str] = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    if not calibrations:
        raise ValueError("No intrinsic calibration files found")

    observations = load_wand_log(log_path)
    available_cameras = sorted(camera_id for camera_id in calibrations.keys() if camera_id in observations)
    if len(available_cameras) < 2:
        raise ValueError("At least two cameras with intrinsics and wand observations are required")

    ref_camera_id = reference_camera_id or available_cameras[0]
    if ref_camera_id not in available_cameras:
        raise ValueError(f"Reference camera '{ref_camera_id}' not found in shared inputs")

    all_camera_params = {
        camera_id: CalibrationLoader.to_camera_params(calib)
        for camera_id, calib in calibrations.items()
        if camera_id in available_cameras
    }

    labeled_by_camera, dropped_counts = label_observations(observations)
    samples_all = build_multiview_samples(
        labeled_by_camera=labeled_by_camera,
        reference_camera_id=ref_camera_id,
        pair_window_us=pair_window_us,
    )
    if len(samples_all) < min_pairs:
        raise ValueError(f"Not enough paired observations: {len(samples_all)}")

    support_counts: Dict[str, int] = {camera_id: 0 for camera_id in available_cameras}
    for sample in samples_all:
        for camera_id in sample.image_points_by_camera.keys():
            if camera_id != ref_camera_id and camera_id in support_counts:
                support_counts[camera_id] += 1

    excluded_camera_reasons: Dict[str, str] = {}
    eligible_cameras: List[str] = [ref_camera_id]
    for camera_id in available_cameras:
        if camera_id == ref_camera_id:
            continue
        if support_counts.get(camera_id, 0) < min_pairs:
            excluded_camera_reasons[camera_id] = (
                f"insufficient_samples:{support_counts.get(camera_id, 0)}<{min_pairs}"
            )
            continue
        eligible_cameras.append(camera_id)
    if len(eligible_cameras) < 2:
        raise ValueError("Not enough cameras with sufficient paired observations")

    camera_params = {camera_id: all_camera_params[camera_id] for camera_id in eligible_cameras}
    samples = _filter_samples_to_camera_set(samples_all, eligible_cameras, ref_camera_id)
    if len(samples) < min_pairs:
        raise ValueError(f"Not enough paired observations after camera filtering: {len(samples)}")

    ref_camera = camera_params[ref_camera_id]
    pair_init: Dict[str, Dict[str, Any]] = {}
    for camera_id in eligible_cameras:
        if camera_id == ref_camera_id:
            continue
        try:
            pair_init[camera_id] = _estimate_pair_pose(
                ref_camera,
                camera_params[camera_id],
                samples,
                ref_camera_id,
                camera_id,
                min_pairs=min_pairs,
            )
        except ValueError as exc:
            excluded_camera_reasons[camera_id] = f"init_failed:{exc}"

    solved_cameras = [ref_camera_id] + [camera_id for camera_id in eligible_cameras if camera_id in pair_init]
    if len(solved_cameras) < 2:
        raise ValueError("No non-reference cameras were solvable")

    camera_params = {camera_id: camera_params[camera_id] for camera_id in solved_cameras}
    samples = _filter_samples_to_camera_set(samples, solved_cameras, ref_camera_id)
    if len(samples) < min_pairs:
        raise ValueError(f"Not enough paired observations after initialization filtering: {len(samples)}")
    samples_for_ba = _downsample_samples_uniform(samples, max_samples=max_ba_samples)
    if len(samples_for_ba) < min_pairs:
        raise ValueError(f"Not enough paired observations after downsampling: {len(samples_for_ba)}")

    object_points_m = np.array(WAND_POINTS_MM, dtype=np.float64) / 1000.0
    camera_poses_init: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        ref_camera_id: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
    }
    for camera_id, info in pair_init.items():
        camera_poses_init[camera_id] = (info["rotation"], info["translation"])

    wand_poses_init: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    for sample in samples_for_ba:
        wand_poses_init[sample.sample_id] = _init_wand_pose_for_sample(
            sample,
            camera_params,
            camera_poses_init,
            object_points_m,
        )

    ba = run_joint_bundle_adjustment(
        camera_params=camera_params,
        reference_camera_id=ref_camera_id,
        samples=samples_for_ba,
        object_points_m=object_points_m,
        camera_poses_init=camera_poses_init,
        wand_poses_init=wand_poses_init,
    )
    errors_by_camera = _compute_camera_reprojection_stats(
        camera_params=camera_params,
        samples=samples_for_ba,
        object_points_m=object_points_m,
        camera_poses=ba.camera_poses,
        wand_poses=ba.wand_poses,
    )

    support_counts_final: Dict[str, int] = {camera_id: 0 for camera_id in solved_cameras}
    for sample in samples_for_ba:
        for camera_id in sample.image_points_by_camera.keys():
            if camera_id in support_counts_final and camera_id != ref_camera_id:
                support_counts_final[camera_id] += 1

    cameras_output: List[Dict[str, Any]] = []
    for camera_id in solved_cameras:
        rotation, translation = ba.camera_poses[camera_id]
        if camera_id == ref_camera_id:
            quality = {
                "pair_count": 0,
                "point_count": 0,
                "inlier_ratio": 1.0,
                "median_reproj_error_px": 0.0,
                "baseline_m": 0.0,
                "scale_m_per_unit": 1.0,
                "init_median_reproj_error_px": ba.initial_median_reproj_error_px,
                "final_p90_reproj_error_px": ba.final_p90_reproj_error_px,
            }
            cameras_output.append(
                {
                    "camera_id": camera_id,
                    "rotation_matrix": rotation.tolist(),
                    "translation_m": translation.tolist(),
                    "quality": quality,
                }
            )
            continue

        pair_count = int(support_counts_final.get(camera_id, 0))
        point_count = pair_count * len(WAND_POINTS_MM)
        camera_errors = np.array(errors_by_camera[camera_id], dtype=np.float64)
        median_reproj_error = float(np.median(camera_errors)) if camera_errors.size else 0.0
        cameras_output.append(
            {
                "camera_id": camera_id,
                "rotation_matrix": rotation.tolist(),
                "translation_m": translation.tolist(),
                "essential_matrix": pair_init[camera_id]["essential_matrix"].tolist(),
                "quality": {
                    "pair_count": pair_count,
                    "point_count": point_count,
                    "inlier_ratio": float(pair_init[camera_id]["inlier_ratio"]),
                    "median_reproj_error_px": median_reproj_error,
                    "baseline_m": float(np.linalg.norm(translation)),
                    "scale_m_per_unit": float(pair_init[camera_id]["scale_m_per_unit"]),
                    "init_median_reproj_error_px": ba.initial_median_reproj_error_px,
                    "final_p90_reproj_error_px": ba.final_p90_reproj_error_px,
                },
            }
        )

    sample_spans = np.array(
        [max(sample.timestamps.values()) - min(sample.timestamps.values()) for sample in samples_for_ba],
        dtype=np.float64,
    )
    result = {
        "schema_version": "1.0",
        "reference_camera_id": ref_camera_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "wand": {
            "name": WAND_NAME,
            "marker_diameter_mm": WAND_MARKER_DIAMETER_MM,
            "points_mm": [list(point) for point in WAND_POINTS_MM],
        },
        "session_meta": {
            "session_id": session_id,
            "log_path": str(Path(log_path)),
            "pair_window_us": pair_window_us,
            "min_pairs": min_pairs,
            "max_ba_samples": max_ba_samples,
            "target_camera_ids": solved_cameras,
            "sample_count": len(samples),
            "accepted_sample_count": len(samples_for_ba),
            "excluded_camera_ids": sorted(excluded_camera_reasons.keys()),
            "excluded_camera_reasons": excluded_camera_reasons,
            "pair_delta_us_stats": {
                "p50": float(np.percentile(sample_spans, 50)) if sample_spans.size else 0.0,
                "p90": float(np.percentile(sample_spans, 90)) if sample_spans.size else 0.0,
                "p99": float(np.percentile(sample_spans, 99)) if sample_spans.size else 0.0,
                "max": float(np.max(sample_spans)) if sample_spans.size else 0.0,
            },
            "dropped_frame_counts_by_reason": dropped_counts,
            "optimizer": {
                "iterations": ba.optimizer_iterations,
                "cost": ba.optimizer_cost,
            },
        },
        "cameras": cameras_output,
    }

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate camera extrinsics from wand capture logs")
    parser.add_argument("--intrinsics", required=True, help="Path to intrinsic calibration directory or file")
    parser.add_argument("--log", required=True, help="Path to wand capture JSONL log")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output JSON path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--pair-window-us", type=int, default=DEFAULT_PAIR_WINDOW_US, help="Maximum timestamp delta for pairing")
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS, help="Minimum paired observations per camera pair")
    parser.add_argument("--max-ba-samples", type=int, default=DEFAULT_MAX_BA_SAMPLES, help="Maximum multiview samples used for BA")
    parser.add_argument("--reference-camera", default=None, help="Reference camera ID")
    parser.add_argument("--session-id", default=None, help="Optional session ID for metadata")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = solve_wand_extrinsics(
        intrinsics_path=args.intrinsics,
        log_path=args.log,
        output_path=args.output,
        pair_window_us=args.pair_window_us,
        min_pairs=args.min_pairs,
        max_ba_samples=args.max_ba_samples,
        reference_camera_id=args.reference_camera,
        session_id=args.session_id,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
