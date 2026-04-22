#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import matplotlib.pyplot as plt
import numpy as np

SRC_ROOT = Path(__file__).resolve().parents[2]
CALIB_ROOT = SRC_ROOT / "camera-calibration"
for path in (SRC_ROOT, CALIB_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from calibration.targets.wand import WAND_POINTS_MM
from extrinsics_capture import load_wand_metric_observations


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _camera_fov_from_intrinsics(payload: Dict[str, Any]) -> tuple[float, float]:
    resolution = payload.get("resolution", {})
    width = float(resolution.get("width", 1280))
    height = float(resolution.get("height", 960))
    camera_matrix = payload.get("camera_matrix", {})
    if "matrix" in camera_matrix:
        matrix = np.asarray(camera_matrix["matrix"], dtype=np.float64)
        fx = float(matrix[0, 0])
        fy = float(matrix[1, 1])
    else:
        fx = float(camera_matrix.get("fx", 500.0))
        fy = float(camera_matrix.get("fy", 500.0))
    hfov = 2.0 * np.arctan(width / (2.0 * fx))
    vfov = 2.0 * np.arctan(height / (2.0 * fy))
    return float(hfov), float(vfov)


def _camera_center(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (-(rotation.T @ translation.reshape(3, 1))).reshape(3)


def _transform_points(points: np.ndarray, transform: np.ndarray | None) -> np.ndarray:
    if transform is None:
        return points
    return ((transform[:3, :3] @ points.T).T + transform[:3, 3])


def _camera_frustum_points(
    rotation: np.ndarray,
    translation: np.ndarray,
    hfov_rad: float,
    vfov_rad: float,
    depth: float,
    transform: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    center = _camera_center(rotation, translation)
    half_width = np.tan(hfov_rad / 2.0) * depth
    half_height = np.tan(vfov_rad / 2.0) * depth
    corners_camera = np.array(
        [
            [-half_width, -half_height, depth],
            [half_width, -half_height, depth],
            [half_width, half_height, depth],
            [-half_width, half_height, depth],
        ],
        dtype=np.float64,
    )
    corners_metric = np.array(
        [center + rotation.T @ corner for corner in corners_camera],
        dtype=np.float64,
    )
    center_out = _transform_points(center.reshape(1, 3), transform).reshape(3)
    corners_out = _transform_points(corners_metric, transform)
    return center_out, corners_out


def _scaled_intrinsic_matrix(intrinsics_payload: Dict[str, Any], focal_scale: float) -> np.ndarray:
    camera_matrix = intrinsics_payload.get("camera_matrix", {})
    if "matrix" in camera_matrix:
        intrinsic = np.asarray(camera_matrix["matrix"], dtype=np.float64).copy()
    else:
        intrinsic = np.array(
            [
                [float(camera_matrix.get("fx", 500.0)), 0.0, float(camera_matrix.get("cx", 640.0))],
                [0.0, float(camera_matrix.get("fy", 500.0)), float(camera_matrix.get("cy", 480.0))],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    intrinsic[0, 0] *= focal_scale
    intrinsic[1, 1] *= focal_scale
    return intrinsic


def _distortion_array(intrinsics_payload: Dict[str, Any]) -> np.ndarray:
    distortion = intrinsics_payload.get("distortion_coefficients", {})
    if "array" in distortion:
        return np.asarray(distortion["array"], dtype=np.float64)
    return np.array(
        [
            float(distortion.get("k1", 0.0)),
            float(distortion.get("k2", 0.0)),
            float(distortion.get("p1", 0.0)),
            float(distortion.get("p2", 0.0)),
            float(distortion.get("k3", 0.0)),
        ],
        dtype=np.float64,
    )


def _pair_wand_samples(
    observations_by_camera: Dict[str, List[Any]],
    camera_ids: List[str],
    pair_window_us: int,
) -> List[Dict[str, np.ndarray]]:
    if len(camera_ids) < 2:
        return []
    reference_camera_id = max(
        camera_ids,
        key=lambda camera_id: len(observations_by_camera.get(camera_id, [])),
    )
    refs = list(observations_by_camera.get(reference_camera_id, []))
    refs.sort(key=lambda item: item.timestamp)
    other_ids = [camera_id for camera_id in camera_ids if camera_id != reference_camera_id]
    cursors = {camera_id: 0 for camera_id in other_ids}
    paired: List[Dict[str, np.ndarray]] = []
    for ref in refs:
        timestamps = {reference_camera_id: int(ref.timestamp)}
        points = {reference_camera_id: np.asarray(ref.image_points, dtype=np.float64)}
        for camera_id in other_ids:
            rows = observations_by_camera.get(camera_id, [])
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
            match = rows[best_idx]
            timestamps[camera_id] = int(match.timestamp)
            points[camera_id] = np.asarray(match.image_points, dtype=np.float64)
        if len(points) < 2:
            continue
        if max(timestamps.values()) - min(timestamps.values()) > pair_window_us:
            continue
        paired.append(points)
    return paired


def _triangulate_point(
    observations: List[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> np.ndarray | None:
    if len(observations) < 2:
        return None
    rows: List[np.ndarray] = []
    for projection, intrinsic, distortion, uv in observations:
        import cv2

        normalized = cv2.undistortPoints(
            np.asarray(uv, dtype=np.float64).reshape(1, 1, 2),
            intrinsic.astype(np.float64),
            distortion.astype(np.float64),
        ).reshape(2)
        ray = np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)
        rows.append(ray[0] * projection[2] - projection[0])
        rows.append(ray[1] * projection[2] - projection[1])
    system = np.stack(rows, axis=0)
    _, _, vt = np.linalg.svd(system, full_matrices=False)
    point_h = vt[-1]
    if abs(float(point_h[3])) < 1e-12:
        return None
    return (point_h[:3] / point_h[3]).astype(np.float64)


def _collect_wand_shapes(
    payload: Dict[str, Any],
    intrinsics_dir: Path,
    pair_window_us: int,
    transform: np.ndarray | None,
) -> List[np.ndarray]:
    wand_metric_log_raw = payload.get("wand_metric_log_path")
    if not isinstance(wand_metric_log_raw, str) or not wand_metric_log_raw.strip():
        return []
    wand_metric_log_path = Path(wand_metric_log_raw)
    if not wand_metric_log_path.is_absolute():
        wand_metric_log_path = Path.cwd() / wand_metric_log_path
    if not wand_metric_log_path.exists():
        return []

    metric = payload.get("metric", {})
    metric_rows = metric.get("camera_poses", []) if isinstance(metric, dict) else []
    if not isinstance(metric_rows, list) or len(metric_rows) < 2:
        return []

    observations_by_camera = load_wand_metric_observations(wand_metric_log_path)
    camera_ids = [str(row.get("camera_id", "")) for row in metric_rows if row.get("camera_id")]
    paired_samples = _pair_wand_samples(observations_by_camera, camera_ids, pair_window_us)
    if not paired_samples:
        return []

    camera_models: Dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for row in metric_rows:
        camera_id = str(row.get("camera_id", ""))
        if not camera_id:
            continue
        intrinsics = _load_json(
            intrinsics_dir / f"calibration_intrinsics_v1_{camera_id}.json"
        )
        focal_scale = float(row.get("focal_scale", 1.0))
        intrinsic = _scaled_intrinsic_matrix(intrinsics, focal_scale)
        distortion = _distortion_array(intrinsics)
        rotation = np.asarray(row["R"], dtype=np.float64)
        translation = np.asarray(row["t"], dtype=np.float64)
        projection = np.hstack([rotation, translation.reshape(3, 1)])
        camera_models[camera_id] = (projection, intrinsic, distortion)

    shapes: List[np.ndarray] = []
    for sample in paired_samples:
        tri_points: List[np.ndarray] = []
        for point_idx in range(4):
            observations: List[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
            for camera_id, image_points in sample.items():
                if camera_id not in camera_models:
                    continue
                projection, intrinsic, distortion = camera_models[camera_id]
                observations.append(
                    (
                        projection,
                        intrinsic,
                        distortion,
                        image_points[point_idx],
                    )
                )
            point = _triangulate_point(observations)
            if point is None:
                tri_points = []
                break
            tri_points.append(point)
        if len(tri_points) != 4:
            continue
        shape = np.asarray(tri_points, dtype=np.float64)
        if transform is not None:
            shape = _transform_points(shape, transform)
        shapes.append(shape)
    return shapes


def _resolve_camera_rows(
    payload: Dict[str, Any],
    frame_name: str,
) -> tuple[List[Dict[str, Any]], np.ndarray | None, str]:
    pose = payload.get("pose", {})
    metric = payload.get("metric", {})
    world = payload.get("world", {})
    if frame_name == "similarity":
        rows = pose.get("camera_poses", []) if isinstance(pose, dict) else []
        return rows, None, "similarity_camera"
    if frame_name == "metric":
        rows = metric.get("camera_poses", []) if isinstance(metric, dict) else []
        return rows, None, "metric_camera"
    if frame_name == "world":
        rows = metric.get("camera_poses", []) if isinstance(metric, dict) else []
        transform = (
            np.asarray(world.get("to_world_matrix", np.eye(4)), dtype=np.float64)
            if isinstance(world, dict)
            else np.eye(4, dtype=np.float64)
        )
        return rows, transform, "world"
    raise ValueError(f"unknown frame: {frame_name}")


def _set_equal_axes(ax: Any, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = float(np.max(maxs - mins) / 2.0)
    if radius <= 0.0:
        radius = 1.0
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Interactive 3D viewer for v2 extrinsics"
    )
    parser.add_argument("--extrinsics", default="calibration/extrinsics_pose_v2.json")
    parser.add_argument("--intrinsics-dir", default="calibration")
    parser.add_argument("--frustum-depth", type=float, default=0.7)
    parser.add_argument(
        "--frame",
        choices=("auto", "similarity", "metric", "world"),
        default="auto",
    )
    parser.add_argument(
        "--show-wand",
        action="store_true",
        help="Overlay triangulated wand metric samples when available",
    )
    parser.add_argument("--wand-pair-window-us", type=int, default=4166)
    parser.add_argument(
        "--wand-skeleton-step",
        type=int,
        default=20,
        help="Draw every Nth wand sample as a skeleton",
    )
    parser.add_argument("--save", default=None, help="Optional output image path")
    args = parser.parse_args()

    payload = _load_json(Path(args.extrinsics))
    intrinsics_dir = Path(args.intrinsics_dir)
    pose = payload.get("pose", {})
    metric = payload.get("metric", {})
    world = payload.get("world", {})

    frame_name = args.frame
    if frame_name == "auto":
        if args.show_wand and isinstance(world, dict) and world.get("status") == "resolved":
            frame_name = "world"
        elif args.show_wand and isinstance(metric, dict) and metric.get("status") == "resolved":
            frame_name = "metric"
        else:
            frame_name = "similarity"
    camera_rows, transform, resolved_frame = _resolve_camera_rows(payload, frame_name)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"Extrinsics Pose v2 View ({resolved_frame})")

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
    all_points: List[np.ndarray] = []

    for idx, row in enumerate(camera_rows):
        camera_id = str(row["camera_id"])
        intrinsics = _load_json(
            intrinsics_dir / f"calibration_intrinsics_v1_{camera_id}.json"
        )
        hfov, vfov = _camera_fov_from_intrinsics(intrinsics)
        rotation = np.asarray(row["R"], dtype=np.float64)
        translation = np.asarray(row["t"], dtype=np.float64)
        center, frustum = _camera_frustum_points(
            rotation,
            translation,
            hfov,
            vfov,
            float(args.frustum_depth),
            transform=transform,
        )
        color = colors[idx % len(colors)]
        all_points.append(center)
        all_points.extend(list(frustum))
        ax.scatter([center[0]], [center[1]], [center[2]], color=color, s=40)
        ax.text(center[0], center[1], center[2], f" {camera_id}", color=color)
        for corner in frustum:
            ax.plot(
                [center[0], corner[0]],
                [center[1], corner[1]],
                [center[2], corner[2]],
                color=color,
                linewidth=1.5,
            )
        for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
            ax.plot(
                [frustum[a, 0], frustum[b, 0]],
                [frustum[a, 1], frustum[b, 1]],
                [frustum[a, 2], frustum[b, 2]],
                color=color,
                linewidth=1.5,
            )

    if args.show_wand:
        wand_shapes = _collect_wand_shapes(
            payload,
            intrinsics_dir,
            int(args.wand_pair_window_us),
            transform,
        )
        if wand_shapes:
            elbow_points = np.asarray(
                [shape[0] for shape in wand_shapes],
                dtype=np.float64,
            )
            all_points.extend(list(elbow_points))
            ax.scatter(
                elbow_points[:, 0],
                elbow_points[:, 1],
                elbow_points[:, 2],
                color="#111111",
                s=10,
                alpha=0.28,
            )
            step = max(1, int(args.wand_skeleton_step))
            sampled_shapes = wand_shapes[::step]
            edge_pairs = ((0, 1), (0, 2), (0, 3), (2, 3))
            for idx, shape in enumerate(sampled_shapes):
                is_latest = idx == len(sampled_shapes) - 1
                color = "#ff7f0e" if is_latest else "#2ca02c"
                alpha = 0.75 if is_latest else 0.18
                for a, b in edge_pairs:
                    ax.plot(
                        [shape[a, 0], shape[b, 0]],
                        [shape[a, 1], shape[b, 1]],
                        [shape[a, 2], shape[b, 2]],
                        color=color,
                        linewidth=1.1,
                        alpha=alpha,
                    )
                ax.scatter(
                    shape[:, 0],
                    shape[:, 1],
                    shape[:, 2],
                    color=color,
                    s=8,
                    alpha=alpha,
                )
            latest_shape = wand_shapes[-1]
            all_points.extend(list(latest_shape))
            ax.text(
                latest_shape[0, 0],
                latest_shape[0, 1],
                latest_shape[0, 2],
                " wand",
                color="#ff7f0e",
            )

    if not all_points:
        all_points.append(np.zeros(3, dtype=np.float64))
    _set_equal_axes(ax, np.asarray(all_points, dtype=np.float64))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=22, azim=36)

    note = (
        f"view.frame={resolved_frame}  "
        f"pose.frame={pose.get('frame', 'unknown')}  "
        f"metric.status={metric.get('status', 'unknown')}  "
        f"world.status={world.get('status', 'unknown')}"
    )
    fig.text(0.02, 0.02, note, fontsize=9)

    summary = pose.get("solve_summary", {}) if isinstance(pose, dict) else {}
    if isinstance(summary, dict):
        summary_note = (
            f"usable_rows={summary.get('usable_rows', 'n/a')}  "
            f"complete_rows={summary.get('complete_rows', 'n/a')}  "
            f"median_reproj_error_px={summary.get('median_reproj_error_px', 'n/a')}"
        )
        fig.text(0.02, 0.05, summary_note, fontsize=9)
    if args.show_wand:
        wand_note = (
            f"wand.points_mm={list(WAND_POINTS_MM)}  "
            f"wand_pair_window_us={int(args.wand_pair_window_us)}"
        )
        fig.text(0.02, 0.08, wand_note, fontsize=8)

    if args.save:
        output = Path(args.save)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=160, bbox_inches="tight")

    plt.show()


if __name__ == "__main__":
    main()
