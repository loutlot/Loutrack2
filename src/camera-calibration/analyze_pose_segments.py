#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

SRC_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parent
for path in (SRC_ROOT, MODULE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from calibrate_extrinsics import (  # type: ignore
    _project_point,
    _scaled_intrinsic_matrix,
    _triangulate_multiview_point,
    build_camera_order_and_rows,
    load_pose_capture_rows,
)
from host.geo import CalibrationLoader


def _load_extrinsics_payload(path: str | Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _camera_pose_map(payload: Dict[str, Any]) -> tuple[list[str], Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[str, float]]:
    pose = payload.get("pose", {})
    rows = pose.get("camera_poses", []) if isinstance(pose, dict) else []
    camera_order = payload.get("camera_order", [])
    if not isinstance(camera_order, list):
        camera_order = []
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    focal_scales: Dict[str, float] = {}
    for row in rows:
        if not isinstance(row, dict):
            continue
        camera_id = str(row.get("camera_id", "")).strip()
        if not camera_id:
            continue
        camera_poses[camera_id] = (
            np.asarray(row.get("R", np.eye(3)), dtype=np.float64).reshape(3, 3),
            np.asarray(row.get("t", [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3),
        )
        focal_scales[camera_id] = float(row.get("focal_scale", 1.0) or 1.0)
    ordered = [camera_id for camera_id in camera_order if camera_id in camera_poses]
    if not ordered:
        ordered = sorted(camera_poses.keys())
    return ordered, camera_poses, focal_scales


def _world_transform(payload: Dict[str, Any]) -> tuple[float, np.ndarray | None]:
    metric = payload.get("metric", {})
    world = payload.get("world", {})
    scale = float(metric.get("scale_m_per_unit", 1.0) or 1.0)
    matrix = None
    if isinstance(world, dict) and world.get("status") == "resolved":
        raw = world.get("to_world_matrix")
        if raw is not None:
            matrix = np.asarray(raw, dtype=np.float64).reshape(4, 4)
    return scale, matrix


def _summarize_scalar(values: Sequence[float]) -> Dict[str, float | None]:
    arr = np.asarray(list(values), dtype=np.float64)
    if arr.size == 0:
        return {"count": 0, "median": None, "p90": None, "max": None}
    return {
        "count": int(arr.size),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(np.max(arr)),
    }


def _segment_by_time(rows: list[Dict[str, Any]], bin_count: int) -> list[Dict[str, Any]]:
    if not rows:
        return []
    ordered = sorted(rows, key=lambda row: int(row["anchor_timestamp"]))
    segments: list[Dict[str, Any]] = []
    n = len(ordered)
    for idx in range(bin_count):
        start = (idx * n) // bin_count
        end = ((idx + 1) * n) // bin_count
        chunk = ordered[start:end]
        if not chunk:
            continue
        errors = [float(row["row_median_reproj_error_px"]) for row in chunk]
        segments.append(
            {
                "segment_id": idx,
                "time_range_us": [int(chunk[0]["anchor_timestamp"]), int(chunk[-1]["anchor_timestamp"])],
                **_summarize_scalar(errors),
            }
        )
    return segments


def _segment_by_image_grid(
    rows: list[Dict[str, Any]],
    width: int,
    height: int,
    cols: int,
    rows_count: int,
) -> list[Dict[str, Any]]:
    buckets: Dict[Tuple[int, int], List[float]] = {}
    for row in rows:
        x, y = row["anchor_image_xy"]
        col = min(max(int(float(x) / max(width, 1) * cols), 0), cols - 1)
        row_idx = min(max(int(float(y) / max(height, 1) * rows_count), 0), rows_count - 1)
        buckets.setdefault((row_idx, col), []).append(float(row["row_median_reproj_error_px"]))
    segments: list[Dict[str, Any]] = []
    for (row_idx, col_idx), errors in sorted(buckets.items()):
        segments.append(
            {
                "grid_cell": {"row": row_idx, "col": col_idx},
                **_summarize_scalar(errors),
            }
        )
    return segments


def _segment_axis_quantiles(rows: list[Dict[str, Any]], axis_name: str, value_index: int, bin_count: int) -> list[Dict[str, Any]]:
    if not rows:
        return []
    ordered = sorted(rows, key=lambda row: float(row["point_position"][value_index]))
    segments: list[Dict[str, Any]] = []
    n = len(ordered)
    for idx in range(bin_count):
        start = (idx * n) // bin_count
        end = ((idx + 1) * n) // bin_count
        chunk = ordered[start:end]
        if not chunk:
            continue
        errors = [float(row["row_median_reproj_error_px"]) for row in chunk]
        coords = [float(row["point_position"][value_index]) for row in chunk]
        segments.append(
            {
                "axis": axis_name,
                "segment_id": idx,
                "coordinate_range": [float(min(coords)), float(max(coords))],
                **_summarize_scalar(errors),
            }
        )
    return segments


def _dominance_report(global_median: float | None, time_segments: list[Dict[str, Any]], image_segments: list[Dict[str, Any]], axis_segments: Dict[str, list[Dict[str, Any]]]) -> Dict[str, Any]:
    def _worst_ratio(segments: list[Dict[str, Any]]) -> float:
        if not segments or global_median is None or global_median <= 1e-9:
            return 1.0
        medians = [float(item["median"]) for item in segments if item.get("median") is not None]
        return max(medians) / global_median if medians else 1.0

    time_ratio = _worst_ratio(time_segments)
    image_ratio = _worst_ratio(image_segments)
    axis_ratio = max((_worst_ratio(items) for items in axis_segments.values()), default=1.0)
    ratios = {"time": time_ratio, "image": image_ratio, "position_axis": axis_ratio}
    likely_driver = max(ratios.items(), key=lambda item: item[1])[0]
    # Spatial segments can inherit timestamp-driven drift because inconsistent
    # observations perturb the triangulated 3D point. Prefer "time" when its
    # signal is strong and close to the strongest spatial-axis ratio.
    if time_ratio >= 1.25 and time_ratio >= image_ratio and time_ratio >= axis_ratio * 0.9:
        likely_driver = "time"
    if ratios[likely_driver] < 1.25:
        likely_driver = "none"
    return {
        "global_median_reproj_error_px": global_median,
        "worst_ratio_vs_global": ratios,
        "likely_driver": likely_driver,
    }


def analyze_pose_segments(
    *,
    intrinsics_path: str | Path,
    extrinsics_path: str | Path,
    pose_log_path: str | Path,
    pair_window_us: int,
    time_bins: int = 6,
    position_bins: int = 4,
    image_grid_cols: int = 3,
    image_grid_rows: int = 3,
) -> Dict[str, Any]:
    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    camera_params = {
        camera_id: CalibrationLoader.to_camera_params(payload)
        for camera_id, payload in calibrations.items()
    }
    extrinsics_payload = _load_extrinsics_payload(extrinsics_path)
    _, camera_poses, focal_scales = _camera_pose_map(extrinsics_payload)
    scale_m_per_unit, to_world_matrix = _world_transform(extrinsics_payload)
    observations_by_camera = load_pose_capture_rows(pose_log_path)
    active_observations = {
        camera_id: rows
        for camera_id, rows in observations_by_camera.items()
        if camera_id in camera_params and camera_id in camera_poses and rows
    }
    camera_order, rows, row_summary = build_camera_order_and_rows(active_observations, pair_window_us)
    if not camera_order:
        raise ValueError("No active cameras found in pose log")

    row_metrics: list[Dict[str, Any]] = []
    for row in rows:
        observations = []
        visible_points = []
        for camera_index, camera_id in enumerate(camera_order):
            point = row.image_points[camera_index]
            if point is None:
                continue
            camera = camera_params[camera_id]
            rotation, translation = camera_poses[camera_id]
            intrinsic = _scaled_intrinsic_matrix(camera, focal_scales.get(camera_id, 1.0))
            projection = np.hstack([rotation, translation.reshape(3, 1)])
            observations.append((intrinsic, projection, point))
            visible_points.append((camera_id, point, intrinsic, rotation, translation, camera.distortion_coeffs))
        if len(observations) < 2:
            continue
        point_similarity = _triangulate_multiview_point(observations)
        if point_similarity is None:
            continue
        per_camera_errors: Dict[str, float] = {}
        for camera_id, point, intrinsic, rotation, translation, distortion in visible_points:
            pred = _project_point(point_similarity, rotation, translation, intrinsic, distortion)
            per_camera_errors[camera_id] = float(np.linalg.norm(pred - point))
        error_values = list(per_camera_errors.values())
        anchor_point = row.image_points[0]
        if anchor_point is None:
            continue
        point_position = point_similarity.copy()
        position_frame = "similarity_camera"
        if to_world_matrix is not None:
            point_metric = point_similarity * scale_m_per_unit
            point_h = np.ones(4, dtype=np.float64)
            point_h[:3] = point_metric
            point_position = (to_world_matrix @ point_h)[:3]
            position_frame = "world"
        row_metrics.append(
            {
                "anchor_timestamp": int(row.anchor_timestamp),
                "anchor_frame_index": int(row.anchor_frame_index),
                "anchor_image_xy": [float(anchor_point[0]), float(anchor_point[1])],
                "row_median_reproj_error_px": float(np.median(np.asarray(error_values, dtype=np.float64))),
                "row_p90_reproj_error_px": float(np.percentile(np.asarray(error_values, dtype=np.float64), 90)),
                "row_max_reproj_error_px": float(np.max(np.asarray(error_values, dtype=np.float64))),
                "matched_span_us": max((int(delta) for delta in row.matched_delta_us if delta is not None), default=0),
                "per_camera_reproj_error_px": per_camera_errors,
                "point_position": point_position.tolist(),
                "position_frame": position_frame,
            }
        )

    anchor_camera = camera_order[0]
    width, height = camera_params[anchor_camera].resolution
    global_errors = [float(row["row_median_reproj_error_px"]) for row in row_metrics]
    time_segments = _segment_by_time(row_metrics, time_bins)
    image_segments = _segment_by_image_grid(row_metrics, width, height, image_grid_cols, image_grid_rows)
    axis_segments = {
        "x": _segment_axis_quantiles(row_metrics, "x", 0, position_bins),
        "y": _segment_axis_quantiles(row_metrics, "y", 1, position_bins),
        "z": _segment_axis_quantiles(row_metrics, "z", 2, position_bins),
    }
    global_summary = _summarize_scalar(global_errors)
    return {
        "pose_log_path": str(Path(pose_log_path)),
        "extrinsics_path": str(Path(extrinsics_path)),
        "pair_window_us": int(pair_window_us),
        "camera_order": camera_order,
        "anchor_camera_id": anchor_camera,
        "row_summary": row_summary,
        "global_row_error_px": global_summary,
        "time_segments": time_segments,
        "image_grid_segments": image_segments,
        "position_axis_segments": axis_segments,
        "diagnosis": _dominance_report(global_summary["median"], time_segments, image_segments, axis_segments),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze pose capture reprojection error by time and position segments")
    parser.add_argument("--intrinsics", default="calibration", help="Intrinsics directory or file")
    parser.add_argument("--extrinsics", default="calibration/extrinsics_pose_v2.json", help="Extrinsics v2 JSON path")
    parser.add_argument("--pose-log", default="logs/extrinsics_pose_capture.jsonl", help="Pose capture JSONL path")
    parser.add_argument("--pair-window-us", type=int, default=2000, help="Pairing window for building rows")
    parser.add_argument("--time-bins", type=int, default=6, help="Number of time quantile segments")
    parser.add_argument("--position-bins", type=int, default=4, help="Number of quantile bins per world/similarity axis")
    parser.add_argument("--image-grid-cols", type=int, default=3, help="Anchor image grid columns")
    parser.add_argument("--image-grid-rows", type=int, default=3, help="Anchor image grid rows")
    parser.add_argument("--output", default=None, help="Optional output JSON path")
    args = parser.parse_args(argv)

    result = analyze_pose_segments(
        intrinsics_path=args.intrinsics,
        extrinsics_path=args.extrinsics,
        pose_log_path=args.pose_log,
        pair_window_us=args.pair_window_us,
        time_bins=args.time_bins,
        position_bins=args.position_bins,
        image_grid_cols=args.image_grid_cols,
        image_grid_rows=args.image_grid_rows,
    )
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
