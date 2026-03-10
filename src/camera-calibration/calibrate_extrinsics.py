#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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
from host.wand_session import WAND_MARKER_DIAMETER_MM, WAND_NAME, WAND_POINTS_MM


DEFAULT_PAIR_WINDOW_US = 8000
DEFAULT_MIN_PAIRS = 8
DEFAULT_OUTPUT = "calibration/calibration_extrinsics_v1.json"


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
        or {
            "name": WAND_NAME,
            "marker_diameter_mm": WAND_MARKER_DIAMETER_MM,
            "points_mm": [list(point) for point in WAND_POINTS_MM],
        },
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


def solve_pose_capture_extrinsics(
    intrinsics_path: str | Path,
    pose_log_path: str | Path,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    min_pairs: int = DEFAULT_MIN_PAIRS,
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
    reference_camera_id: Optional[str] = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    pose_result = solve_pose_capture_extrinsics(
        intrinsics_path=intrinsics_path,
        pose_log_path=pose_log_path,
        pair_window_us=pair_window_us,
        min_pairs=min_pairs,
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
        "sample_count": int(pose_result["sample_count"]),
        "accepted_sample_count": int(pose_result["sample_count"]),
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
    camera_poses_metric, metric_summary = apply_wand_metric_alignment(
        camera_params=camera_params,
        camera_poses_similarity=camera_poses_similarity,
        wand_observations_by_camera=wand_obs,
        wand_points_mm=WAND_POINTS_MM,
        up_axis="Z",
        pair_window_us=pair_window_us,
    )
    validation = validate_extrinsics(
        camera_params=camera_params,
        camera_poses=camera_poses_metric,
        wand_observations_by_camera=wand_obs,
        wand_points_mm=WAND_POINTS_MM,
        pair_window_us=pair_window_us,
        up_axis="Z",
    )
    previous_meta = payload.get("session_meta", {})
    if not isinstance(previous_meta, dict):
        previous_meta = {}
    output_target = Path(output_path) if output_path is not None else Path(extrinsics_path)
    session_meta = {
        **previous_meta,
        "session_id": session_id if session_id is not None else previous_meta.get("session_id"),
        "wand_metric_log_path": str(Path(wand_metric_log_path)),
        "pair_window_us": pair_window_us,
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
        wand_payload=payload.get("wand") if isinstance(payload.get("wand"), dict) else None,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate camera extrinsics from pose capture logs")
    parser.add_argument("--intrinsics", required=True, help="Path to intrinsic calibration directory or file")
    parser.add_argument("--pose-log", required=True, help="Path to pose capture JSONL log")
    parser.add_argument("--wand-metric-log", default=None, help="Optional wand metric capture JSONL log")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output JSON path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--pair-window-us", type=int, default=DEFAULT_PAIR_WINDOW_US, help="Maximum timestamp delta for pairing")
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS, help="Minimum paired observations per camera pair")
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
            session_id=args.session_id,
        )
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
