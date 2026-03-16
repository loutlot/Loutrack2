#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List

import matplotlib.pyplot as plt
import numpy as np


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


def _camera_frustum_points(
    rotation: np.ndarray,
    translation: np.ndarray,
    hfov_rad: float,
    vfov_rad: float,
    depth: float,
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
    corners_world = np.array([center + rotation.T @ corner for corner in corners_camera], dtype=np.float64)
    return center, corners_world


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
    parser = argparse.ArgumentParser(description="Interactive 3D viewer for v2 extrinsics")
    parser.add_argument("--extrinsics", default="calibration/extrinsics_pose_v2.json")
    parser.add_argument("--intrinsics-dir", default="calibration")
    parser.add_argument("--frustum-depth", type=float, default=0.7)
    parser.add_argument("--save", default=None, help="Optional output image path")
    args = parser.parse_args()

    payload = _load_json(Path(args.extrinsics))
    intrinsics_dir = Path(args.intrinsics_dir)
    pose = payload.get("pose", {})
    camera_rows = pose.get("camera_poses", []) if isinstance(pose, dict) else []
    metric = payload.get("metric", {})
    world = payload.get("world", {})

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Extrinsics Pose v2 View")

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
    all_points: List[np.ndarray] = []

    for idx, row in enumerate(camera_rows):
        camera_id = str(row["camera_id"])
        intrinsics = _load_json(intrinsics_dir / f"calibration_intrinsics_v1_{camera_id}.json")
        hfov, vfov = _camera_fov_from_intrinsics(intrinsics)
        rotation = np.asarray(row["R"], dtype=np.float64)
        translation = np.asarray(row["t"], dtype=np.float64)
        center, frustum = _camera_frustum_points(rotation, translation, hfov, vfov, float(args.frustum_depth))
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

    if not all_points:
        all_points.append(np.zeros(3, dtype=np.float64))
    _set_equal_axes(ax, np.asarray(all_points, dtype=np.float64))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=22, azim=36)

    note = (
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

    if args.save:
        output = Path(args.save)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=160, bbox_inches="tight")

    plt.show()


if __name__ == "__main__":
    main()
