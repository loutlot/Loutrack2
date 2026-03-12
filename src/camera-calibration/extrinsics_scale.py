from __future__ import annotations

from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np

from extrinsics_capture import WandMetricObservation


WAND_EDGE_PAIRS = ((0, 1), (0, 2), (0, 3), (2, 3))


def _camera_center(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (-(rotation.T @ translation.reshape(3, 1))).reshape(3)


def _rotation_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    x, y, z = axis
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=np.float64,
    )


def _rotation_from_normal_to_up(normal: np.ndarray, up_axis: str) -> np.ndarray:
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64) if up_axis.upper() == "Z" else np.array([0.0, 1.0, 0.0], dtype=np.float64)
    n = normal / max(np.linalg.norm(normal), 1e-12)
    axis = np.cross(n, up)
    s = float(np.linalg.norm(axis))
    c = float(np.clip(np.dot(n, up), -1.0, 1.0))
    if s < 1e-12:
        return np.eye(3, dtype=np.float64) if c > 0.0 else _rotation_from_axis_angle(np.array([1.0, 0.0, 0.0]), np.pi)
    return _rotation_from_axis_angle(axis, np.arctan2(s, c))


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
    observations: Sequence[Tuple[np.ndarray, Any, np.ndarray]],
) -> np.ndarray | None:
    if len(observations) < 2:
        return None
    rows: List[np.ndarray] = []
    for pmat, camera, uv in observations:
        norm = cv2.undistortPoints(
            uv.reshape(1, 1, 2).astype(np.float64),
            camera.intrinsic_matrix.astype(np.float64),
            camera.distortion_coeffs.astype(np.float64),
        ).reshape(2)
        x = np.array([norm[0], norm[1], 1.0], dtype=np.float64)
        rows.append(x[0] * pmat[2] - pmat[0])
        rows.append(x[1] * pmat[2] - pmat[1])
    a = np.stack(rows, axis=0)
    _, _, vt = np.linalg.svd(a, full_matrices=False)
    xh = vt[-1]
    if abs(float(xh[3])) < 1e-12:
        return None
    return (xh[:3] / xh[3]).astype(np.float64)


def _estimate_similarity_transform(
    reconstructed: np.ndarray,
    model: np.ndarray,
) -> Tuple[float, np.ndarray, np.ndarray, float] | None:
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


def apply_wand_metric_alignment(
    camera_params: Dict[str, Any],
    camera_poses_similarity: Dict[str, Tuple[np.ndarray, np.ndarray]],
    wand_observations_by_camera: Dict[str, Sequence[WandMetricObservation]],
    wand_points_mm: Sequence[Sequence[float]],
    up_axis: str = "Z",
    pair_window_us: int = 8000,
    assume_metric_scale: bool = False,
) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[str, Any]]:
    wand_points_m = np.asarray(wand_points_mm, dtype=np.float64) / 1000.0
    known_edges = np.array(
        [np.linalg.norm(wand_points_m[a] - wand_points_m[b]) for a, b in WAND_EDGE_PAIRS],
        dtype=np.float64,
    )
    active_rows: Dict[str, Sequence[WandMetricObservation]] = {}
    for camera_id, rows in wand_observations_by_camera.items():
        if camera_id in camera_poses_similarity and camera_id in camera_params and rows:
            active_rows[camera_id] = rows
    if len(active_rows) < 2:
        return camera_poses_similarity, {
            "scale_source": "none",
            "floor_source": "none",
            "scale_m_per_unit": 1.0,
            "wand_metric_frames": 0,
        }

    reference_camera_id = max(active_rows.keys(), key=lambda camera_id: len(active_rows[camera_id]))
    samples = _build_wand_metric_multiview(active_rows, reference_camera_id, pair_window_us=pair_window_us)
    scale_candidates: List[float] = []
    floor_points: List[np.ndarray] = []
    elbow_points: List[np.ndarray] = []
    axis_vectors: List[np.ndarray] = []
    shape_errors_mm: List[float] = []
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64) if up_axis.upper() == "Z" else np.array([0.0, 1.0, 0.0], dtype=np.float64)
    target_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64) if up_axis.upper() == "Z" else np.array([1.0, 0.0, 0.0], dtype=np.float64)

    for obs_by_camera in samples:
        tri_points: List[np.ndarray] = []
        for point_idx in range(4):
            obs_rows: List[Tuple[np.ndarray, Any, np.ndarray]] = []
            for camera_id, points in obs_by_camera.items():
                rotation, translation = camera_poses_similarity[camera_id]
                pmat = np.hstack([rotation, translation.reshape(3, 1)])
                obs_rows.append((pmat, camera_params[camera_id], points[point_idx]))
            point = _triangulate_point_ls(obs_rows)
            if point is None:
                tri_points = []
                break
            tri_points.append(point)
        if len(tri_points) != 4:
            continue

        tri = np.asarray(tri_points, dtype=np.float64)
        rec_edges = np.array([np.linalg.norm(tri[a] - tri[b]) for a, b in WAND_EDGE_PAIRS], dtype=np.float64)
        valid = rec_edges > 1e-9
        if np.count_nonzero(valid) < 3:
            continue
        similarity_fit = _estimate_similarity_transform(tri, wand_points_m)
        if similarity_fit is None:
            continue
        scale_fit, rotation_fit, translation_fit, shape_error_m = similarity_fit
        if assume_metric_scale:
            edge_rel_err = np.abs(rec_edges[valid] - known_edges[valid]) / np.maximum(known_edges[valid], 1e-9)
            if np.median(edge_rel_err) > 0.35:
                continue
            scale = 1.0
            tri_metric = tri
            aligned = (rotation_fit @ tri_metric.T).T + translation_fit.reshape(1, 3)
            shape_errors_mm.append(float(np.sqrt(np.mean(np.sum((aligned - wand_points_m) ** 2, axis=1)))) * 1000.0)
        else:
            edge_scale = float(np.median(known_edges[valid] / rec_edges[valid]))
            rel_scale_delta = abs(scale_fit - edge_scale) / max(scale_fit, edge_scale, 1e-9)
            if rel_scale_delta > 0.35:
                continue
            scale = float(np.median(np.array([scale_fit, edge_scale], dtype=np.float64)))
            if not np.isfinite(scale) or scale <= 0.0:
                continue
            scale_candidates.append(scale)
            shape_errors_mm.append(shape_error_m * 1000.0)
            tri_metric = tri * scale
        floor_points.append(tri_metric)
        elbow_points.append(tri_metric[0])
        axis_vectors.append(tri_metric[3] - tri_metric[0])

    if (not assume_metric_scale and not scale_candidates) or not floor_points:
        return camera_poses_similarity, {
            "scale_source": "none",
            "floor_source": "none",
            "scale_m_per_unit": 1.0,
            "wand_metric_frames": 0,
        }

    scale = 1.0
    if not assume_metric_scale:
        scale_array = np.asarray(scale_candidates, dtype=np.float64)
        if scale_array.size >= 3:
            scale_med = float(np.median(scale_array))
            scale_mad = float(np.median(np.abs(scale_array - scale_med)))
            if scale_mad > 1e-9:
                keep = np.abs(scale_array - scale_med) <= (2.5 * scale_mad)
                if np.count_nonzero(keep) >= 1:
                    scale_array = scale_array[keep]
        scale = float(np.median(scale_array))
    all_floor = np.concatenate(floor_points, axis=0)
    centroid = np.mean(all_floor, axis=0)
    centered = all_floor - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    floor_normal = vt[-1]
    if float(np.dot(floor_normal, up)) < 0.0:
        floor_normal = -floor_normal
    rotation_floor = _rotation_from_normal_to_up(floor_normal, up_axis)

    axis_acc = np.zeros(3, dtype=np.float64)
    for axis_vec in axis_vectors:
        axis_rot = rotation_floor @ axis_vec
        axis_proj = axis_rot - (np.dot(axis_rot, up) * up)
        norm = np.linalg.norm(axis_proj)
        if norm <= 1e-9:
            continue
        axis_acc += axis_proj / norm
    axis_norm = np.linalg.norm(axis_acc)
    if axis_norm <= 1e-9:
        rotation_yaw = np.eye(3, dtype=np.float64)
        world_axis = target_axis
    else:
        world_axis = axis_acc / axis_norm
        yaw_from = np.array([world_axis[0], world_axis[1], 0.0], dtype=np.float64) if up_axis.upper() == "Z" else np.array([world_axis[0], 0.0, world_axis[2]], dtype=np.float64)
        yaw_to = np.array([target_axis[0], target_axis[1], 0.0], dtype=np.float64) if up_axis.upper() == "Z" else np.array([target_axis[0], 0.0, target_axis[2]], dtype=np.float64)
        yaw_from = yaw_from / max(np.linalg.norm(yaw_from), 1e-12)
        yaw_to = yaw_to / max(np.linalg.norm(yaw_to), 1e-12)
        sin_yaw = float(np.dot(np.cross(yaw_from, yaw_to), up))
        cos_yaw = float(np.clip(np.dot(yaw_from, yaw_to), -1.0, 1.0))
        rotation_yaw = _rotation_from_axis_angle(up, np.arctan2(sin_yaw, cos_yaw))

    rotation_world = rotation_yaw @ rotation_floor
    elbows_world = np.asarray([(rotation_world @ point.reshape(3, 1)).reshape(3) for point in elbow_points], dtype=np.float64)
    origin = np.median(elbows_world, axis=0)

    poses_metric: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for camera_id, (rotation, translation) in camera_poses_similarity.items():
        center = _camera_center(rotation, translation)
        center_metric = (rotation_world @ (center * scale).reshape(3, 1)).reshape(3) - origin
        rotation_metric = rotation @ rotation_world.T
        translation_metric = -(rotation_metric @ center_metric.reshape(3, 1)).reshape(3)
        poses_metric[camera_id] = (rotation_metric, translation_metric)

    return poses_metric, {
        "scale_source": "wand_floor_metric",
        "floor_source": "wand_floor_metric",
        "scale_m_per_unit": scale,
        "wand_metric_frames": len(floor_points),
        "shape_rms_error_mm": float(np.median(np.asarray(shape_errors_mm, dtype=np.float64))) if shape_errors_mm else 0.0,
        "floor_normal_similarity": floor_normal.tolist(),
        "origin_world": origin.tolist(),
        "origin_marker": "elbow",
        "aligned_axis_world": target_axis.tolist(),
        "up_axis": up_axis.upper(),
    }
