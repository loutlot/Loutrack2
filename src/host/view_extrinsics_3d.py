#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _camera_center(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (-(rotation.T @ translation.reshape(3, 1))).reshape(3)


def _camera_frustum_points(
    rotation: np.ndarray,
    translation: np.ndarray,
    hfov_rad: float,
    vfov_rad: float,
    depth: float,
) -> Tuple[np.ndarray, np.ndarray]:
    center = _camera_center(rotation, translation)
    half_w = depth * math.tan(hfov_rad * 0.5)
    half_h = depth * math.tan(vfov_rad * 0.5)
    local = np.array(
        [
            [-half_w, -half_h, depth],
            [half_w, -half_h, depth],
            [half_w, half_h, depth],
            [-half_w, half_h, depth],
        ],
        dtype=np.float64,
    )
    world = (rotation.T @ (local.T - translation.reshape(3, 1))).T
    return center, world


def _camera_fov_from_intrinsics(intrinsics: Dict[str, Any]) -> Tuple[float, float]:
    fx = float(intrinsics["camera_matrix"]["fx"])
    fy = float(intrinsics["camera_matrix"]["fy"])
    width = float(intrinsics["resolution"]["width"])
    height = float(intrinsics["resolution"]["height"])
    hfov = 2.0 * math.atan(width / (2.0 * fx))
    vfov = 2.0 * math.atan(height / (2.0 * fy))
    return hfov, vfov


def _set_equal_axes(ax: Any, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = max(float(np.max(maxs - mins)) * 0.5, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def _plot_grid(ax: Any, x_min: float, x_max: float, y_min: float, y_max: float, z: float = 0.0, step: float = 0.5) -> None:
    xs = np.arange(x_min, x_max + step * 0.5, step)
    ys = np.arange(y_min, y_max + step * 0.5, step)
    for x in xs:
        ax.plot([x, x], [y_min, y_max], [z, z], color="#d0d0d0", linewidth=0.8, zorder=0)
    for y in ys:
        ax.plot([x_min, x_max], [y, y], [z, z], color="#d0d0d0", linewidth=0.8, zorder=0)


def _plot_axes(ax: Any, axis_len: float = 1.0) -> None:
    ax.plot([0.0, axis_len], [0.0, 0.0], [0.0, 0.0], color="#2a8f4a", linewidth=2)
    ax.plot([0.0, 0.0], [0.0, axis_len], [0.0, 0.0], color="#1f77b4", linewidth=2)
    ax.plot([0.0, 0.0], [0.0, 0.0], [0.0, axis_len], color="#777777", linewidth=2)
    ax.text(axis_len, 0.0, 0.0, "X", color="#2a8f4a")
    ax.text(0.0, axis_len, 0.0, "Y", color="#1f77b4")
    ax.text(0.0, 0.0, axis_len, "Z", color="#777777")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive 3D viewer for extrinsics + floor grid")
    parser.add_argument("--extrinsics", default="calibration/calibration_extrinsics_v1.json")
    parser.add_argument("--intrinsics-dir", default="calibration")
    parser.add_argument("--frustum-depth", type=float, default=0.7)
    parser.add_argument("--grid-size", type=float, default=3.0)
    parser.add_argument("--grid-step", type=float, default=0.5)
    parser.add_argument("--save", default=None, help="Optional output image path")
    args = parser.parse_args()

    extrinsics_path = Path(args.extrinsics)
    intrinsics_dir = Path(args.intrinsics_dir)
    payload = _load_json(extrinsics_path)
    cameras = payload.get("cameras", [])
    session_meta = payload.get("session_meta", {})

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Extrinsics 3D View")

    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
    all_points: List[np.ndarray] = []

    centers: List[np.ndarray] = []
    for idx, row in enumerate(cameras):
        camera_id = str(row["camera_id"])
        intrinsics = _load_json(intrinsics_dir / f"calibration_intrinsics_v1_{camera_id}.json")
        hfov, vfov = _camera_fov_from_intrinsics(intrinsics)
        rotation = np.asarray(row["rotation_matrix"], dtype=np.float64)
        translation = np.asarray(row["translation_m"], dtype=np.float64)
        center, frustum = _camera_frustum_points(
            rotation,
            translation,
            hfov_rad=hfov,
            vfov_rad=vfov,
            depth=float(args.frustum_depth),
        )
        color = colors[idx % len(colors)]
        centers.append(center)
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

    centers_np = np.asarray(centers, dtype=np.float64) if centers else np.zeros((1, 3), dtype=np.float64)
    x_min = float(np.min(centers_np[:, 0]) - args.grid_size)
    x_max = float(np.max(centers_np[:, 0]) + args.grid_size)
    y_min = float(np.min(centers_np[:, 1]) - args.grid_size)
    y_max = float(np.max(centers_np[:, 1]) + args.grid_size)
    _plot_grid(ax, x_min, x_max, y_min, y_max, z=0.0, step=float(args.grid_step))
    _plot_axes(ax, axis_len=1.0)

    all_points.extend(
        [
            np.array([x_min, y_min, 0.0], dtype=np.float64),
            np.array([x_max, y_max, 0.0], dtype=np.float64),
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
        ]
    )
    _set_equal_axes(ax, np.asarray(all_points, dtype=np.float64))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=22, azim=36)

    note = (
        f"scale_source={session_meta.get('scale_source', 'none')}  "
        f"floor_source={session_meta.get('floor_source', 'none')}  "
        f"metric_pose_source={session_meta.get('metric_pose_source', 'n/a')}"
    )
    fig.text(0.02, 0.02, note, fontsize=9)

    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160, bbox_inches="tight")

    plt.show()


if __name__ == "__main__":
    main()
