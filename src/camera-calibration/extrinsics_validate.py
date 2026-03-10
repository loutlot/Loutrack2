from __future__ import annotations

from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np

from extrinsics_capture import WandMetricObservation


def _camera_center(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (-(rotation.T @ translation.reshape(3, 1))).reshape(3)


def _build_wand_metric_multiview(
    observations_by_camera: Dict[str, Sequence[WandMetricObservation]],
    reference_camera_id: str,
    pair_window_us: int,
) -> List[Dict[str, np.ndarray]]:
    refs = list(observations_by_camera.get(reference_camera_id, []))
    refs.sort(key=lambda item: item.timestamp)
    camera_ids = sorted(camera_id for camera_id in observations_by_camera.keys() if camera_id != reference_camera_id)
    cursors = {camera_id: 0 for camera_id in camera_ids}
    out: List[Dict[str, np.ndarray]] = []

    for ref in refs:
        timestamps = {reference_camera_id: int(ref.timestamp)}
        points = {reference_camera_id: ref.image_points}
        for camera_id in camera_ids:
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
            points[camera_id] = match.image_points
        if len(points) < 2:
            continue
        if max(timestamps.values()) - min(timestamps.values()) > pair_window_us:
            continue
        out.append(points)
    return out


def _triangulate_point_ls(
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    observations: Sequence[Tuple[str, np.ndarray]],
) -> np.ndarray | None:
    if len(observations) < 2:
        return None
    rows: List[np.ndarray] = []
    for camera_id, uv in observations:
        camera = camera_params[camera_id]
        rotation, translation = camera_poses[camera_id]
        pmat = np.hstack([rotation, translation.reshape(3, 1)])
        norm = cv2.undistortPoints(
            uv.reshape(1, 1, 2).astype(np.float64),
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


def validate_extrinsics(
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    wand_observations_by_camera: Dict[str, Sequence[WandMetricObservation]],
    wand_points_mm: Sequence[Sequence[float]],
    pair_window_us: int = 8000,
    up_axis: str = "Z",
) -> Dict[str, Any]:
    point_count = len(wand_points_mm)
    reproj_errors: Dict[str, List[float]] = {camera_id: [] for camera_id in camera_poses.keys()}
    all_depth_positive = 0
    all_depth_total = 0
    floor_residuals_mm: List[float] = []
    up_consistency_scores: List[float] = []

    active_rows: Dict[str, Sequence[WandMetricObservation]] = {}
    for camera_id, rows in wand_observations_by_camera.items():
        if camera_id in camera_poses and camera_id in camera_params and rows:
            active_rows[camera_id] = rows
    if len(active_rows) >= 2:
        reference_camera_id = max(active_rows.keys(), key=lambda camera_id: len(active_rows[camera_id]))
        samples = _build_wand_metric_multiview(active_rows, reference_camera_id, pair_window_us=pair_window_us)
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64) if up_axis.upper() == "Z" else np.array([0.0, 1.0, 0.0], dtype=np.float64)

        for sample in samples:
            tri_points: List[np.ndarray] = []
            for point_idx in range(point_count):
                observations = [(camera_id, points[point_idx]) for camera_id, points in sample.items()]
                tri = _triangulate_point_ls(camera_params, camera_poses, observations)
                if tri is None:
                    break
                tri_points.append(tri)
            if len(tri_points) != point_count:
                continue

            tri = np.asarray(tri_points, dtype=np.float64)
            centered = tri - np.mean(tri, axis=0)
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
            normal = vt[-1]
            normal = normal / max(np.linalg.norm(normal), 1e-12)
            up_consistency_scores.append(float(abs(np.dot(normal, up))))
            if up_axis.upper() == "Z":
                floor_residuals_mm.append(float(np.sqrt(np.mean(tri[:, 2] ** 2)) * 1000.0))
            else:
                floor_residuals_mm.append(float(np.sqrt(np.mean(tri[:, 1] ** 2)) * 1000.0))

            for camera_id, image_points in sample.items():
                camera = camera_params[camera_id]
                rotation, translation = camera_poses[camera_id]
                rvec, _ = cv2.Rodrigues(rotation)
                proj, _ = cv2.projectPoints(
                    tri.astype(np.float64),
                    rvec.reshape(3, 1),
                    translation.reshape(3, 1),
                    camera.intrinsic_matrix.astype(np.float64),
                    camera.distortion_coeffs.astype(np.float64),
                )
                pred = proj.reshape(-1, 2)
                err = np.linalg.norm(pred - image_points, axis=1)
                reproj_errors[camera_id].extend(err.astype(float).tolist())
                depth = (rotation @ tri.T + translation.reshape(3, 1))[2]
                all_depth_positive += int(np.count_nonzero(depth > 0.0))
                all_depth_total += int(depth.shape[0])

    all_err = np.array([e for rows in reproj_errors.values() for e in rows], dtype=np.float64)
    centers = np.array([_camera_center(r, t) for r, t in camera_poses.values()], dtype=np.float64)
    if centers.shape[0] >= 2:
        baseline_vals = [
            float(np.linalg.norm(centers[i] - centers[j]))
            for i in range(len(centers))
            for j in range(i + 1, len(centers))
        ]
        baseline_min = min(baseline_vals)
        baseline_max = max(baseline_vals)
    else:
        baseline_min = 0.0
        baseline_max = 0.0

    return {
        "median_reproj_error_px": float(np.median(all_err)) if all_err.size else 0.0,
        "p90_reproj_error_px": float(np.percentile(all_err, 90)) if all_err.size else 0.0,
        "positive_depth_ratio": float(all_depth_positive / all_depth_total) if all_depth_total else 1.0,
        "baseline_range_m": [float(baseline_min), float(baseline_max)],
        "per_camera_median_reproj_px": {
            camera_id: float(np.median(np.asarray(errors, dtype=np.float64))) if errors else 0.0
            for camera_id, errors in reproj_errors.items()
        },
        "floor_residual_mm": float(np.median(np.asarray(floor_residuals_mm, dtype=np.float64))) if floor_residuals_mm else 0.0,
        "world_up_consistency": float(np.mean(np.asarray(up_consistency_scores, dtype=np.float64))) if up_consistency_scores else 0.0,
    }
