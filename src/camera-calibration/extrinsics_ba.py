from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from extrinsics_samples import PoseCaptureSample


@dataclass(frozen=True)
class BAResult:
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]
    points_3d: Dict[int, np.ndarray]
    iterations: int
    cost: float
    initial_cost: float


def _triangulate_seed_points(
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    samples: Sequence[PoseCaptureSample],
) -> Dict[int, np.ndarray]:
    out: Dict[int, np.ndarray] = {}
    for sample in samples:
        cameras = [
            camera_id
            for camera_id in sorted(sample.image_points_by_camera.keys())
            if camera_id in camera_poses and camera_id in camera_params
        ]
        if len(cameras) < 2:
            continue
        a, b = cameras[0], cameras[1]
        ra, ta = camera_poses[a]
        rb, tb = camera_poses[b]
        pa = np.hstack([ra, ta.reshape(3, 1)])
        pb = np.hstack([rb, tb.reshape(3, 1)])
        xa = cv2.undistortPoints(
            sample.image_points_by_camera[a].reshape(1, 1, 2).astype(np.float64),
            camera_params[a].intrinsic_matrix.astype(np.float64),
            camera_params[a].distortion_coeffs.astype(np.float64),
        ).reshape(2, 1)
        xb = cv2.undistortPoints(
            sample.image_points_by_camera[b].reshape(1, 1, 2).astype(np.float64),
            camera_params[b].intrinsic_matrix.astype(np.float64),
            camera_params[b].distortion_coeffs.astype(np.float64),
        ).reshape(2, 1)
        x4 = cv2.triangulatePoints(pa, pb, xa, xb)
        out[sample.sample_id] = (x4[:3] / x4[3]).reshape(3)
    return out


def _project(camera: Any, rotation: np.ndarray, translation: np.ndarray, point: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(rotation)
    proj, _ = cv2.projectPoints(
        point.reshape(1, 3).astype(np.float64),
        rvec.reshape(3, 1),
        translation.reshape(3, 1).astype(np.float64),
        camera.intrinsic_matrix.astype(np.float64),
        camera.distortion_coeffs.astype(np.float64),
    )
    return proj.reshape(2)


def run_pose_bundle_adjustment(
    camera_params: Dict[str, Any],
    reference_camera_id: str,
    samples: Sequence[PoseCaptureSample],
    camera_poses_init: Dict[str, Tuple[np.ndarray, np.ndarray]],
    loss: str = "huber",
) -> BAResult:
    camera_ids = [camera_id for camera_id in sorted(camera_params.keys()) if camera_id != reference_camera_id]
    points_init = _triangulate_seed_points(camera_params, camera_poses_init, samples)
    if not points_init:
        raise ValueError("No triangulatable samples for BA")

    point_ids = sorted(points_init.keys())
    point_index = {sample_id: idx for idx, sample_id in enumerate(point_ids)}

    cam_params = []
    for camera_id in camera_ids:
        rmat, tvec = camera_poses_init[camera_id]
        rvec, _ = cv2.Rodrigues(rmat)
        cam_params.append(np.concatenate([rvec.reshape(3), tvec.reshape(3)]))
    x0_cam = np.concatenate(cam_params) if cam_params else np.empty((0,), dtype=np.float64)
    x0_pts = np.concatenate([points_init[sample_id].reshape(3) for sample_id in point_ids])
    x0 = np.concatenate([x0_cam, x0_pts])

    obs_rows: List[Tuple[str, int, np.ndarray]] = []
    for sample in samples:
        if sample.sample_id not in point_index:
            continue
        for camera_id, image_point in sample.image_points_by_camera.items():
            obs_rows.append((camera_id, sample.sample_id, image_point.astype(np.float64)))

    def unpack(x: np.ndarray) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[int, np.ndarray]]:
        poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
            reference_camera_id: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
        }
        cursor = 0
        for camera_id in camera_ids:
            rvec = x[cursor : cursor + 3]
            tvec = x[cursor + 3 : cursor + 6]
            cursor += 6
            rmat, _ = cv2.Rodrigues(rvec.reshape(3, 1))
            poses[camera_id] = (rmat, tvec)
        points: Dict[int, np.ndarray] = {}
        for sample_id in point_ids:
            points[sample_id] = x[cursor : cursor + 3]
            cursor += 3
        return poses, points

    def residuals(x: np.ndarray) -> np.ndarray:
        poses, points = unpack(x)
        rows: List[float] = []
        for camera_id, sample_id, image_point in obs_rows:
            if sample_id not in points:
                continue
            camera = camera_params[camera_id]
            rotation, translation = poses[camera_id]
            projected = _project(camera, rotation, translation, points[sample_id])
            diff = projected - image_point
            rows.extend([float(diff[0]), float(diff[1])])
        return np.asarray(rows, dtype=np.float64)

    initial_residual = residuals(x0)
    result = least_squares(
        residuals,
        x0,
        method="trf",
        loss=loss,
        f_scale=2.0,
        max_nfev=200,
    )
    poses_out, points_out = unpack(result.x)
    return BAResult(
        camera_poses=poses_out,
        points_3d=points_out,
        iterations=int(result.nfev),
        cost=float(result.cost),
        initial_cost=float(np.sum(initial_residual**2) * 0.5),
    )
