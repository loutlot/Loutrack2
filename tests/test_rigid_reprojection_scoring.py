from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import CameraParams, create_dummy_calibration
from host.rigid import RigidBodyEstimator, WAIST_PATTERN


def _project(camera: CameraParams, point: np.ndarray) -> tuple[float, float]:
    point_cam = camera.rotation @ point + camera.translation
    return (
        float(camera.fx * point_cam[0] / point_cam[2] + camera.cx),
        float(camera.fy * point_cam[1] / point_cam[2] + camera.cy),
    )


def _observations_for_points(
    cameras: dict[str, CameraParams],
    points_world: np.ndarray,
) -> dict[str, list[dict[str, object]]]:
    observations: dict[str, list[dict[str, object]]] = {}
    for camera_id, camera in cameras.items():
        camera_observations = []
        for blob_index, point in enumerate(points_world):
            uv = _project(camera, point)
            camera_observations.append(
                {
                    "camera_id": camera_id,
                    "blob_index": blob_index,
                    "raw_uv": [uv[0], uv[1]],
                    "undistorted_uv": [uv[0], uv[1]],
                    "area": 4.0,
                }
            )
        observations[camera_id] = camera_observations
    return observations


def test_process_context_reports_high_reprojection_score_for_matching_blobs() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])

    poses = estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )

    assert poses["waist"].valid is True
    diagnostics = estimator.get_tracking_status()["waist"]
    score = diagnostics["reprojection_score"]
    assert score["scored"] is True
    assert score["coordinate_space"] == "raw_pixel"
    assert score["matched_marker_views"] == 8
    assert score["missing_marker_views"] == 0
    assert score["duplicate_assignment_count"] == 0
    assert score["mean_error_px"] < 1e-4
    assert score["score"] > 0.99


def test_process_points_keeps_3d_only_path_without_2d_score() -> None:
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])

    poses = estimator.process_points(points_world, 1_000_000)

    assert poses["waist"].valid is True
    score = estimator.get_tracking_status()["waist"]["reprojection_score"]
    assert score["scored"] is False
    assert score["reason"] == "no_2d_context"
