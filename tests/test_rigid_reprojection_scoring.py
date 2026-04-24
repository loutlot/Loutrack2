from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import CameraParams, create_dummy_calibration
from host.rigid import ReacquireGuardConfig, RigidBodyEstimator, RigidBodyPose, WAIST_PATTERN


def _project(camera: CameraParams, point: np.ndarray) -> tuple[float, float]:
    point_cam = camera.rotation @ point + camera.translation
    return (
        float(camera.fx * point_cam[0] / point_cam[2] + camera.cx),
        float(camera.fy * point_cam[1] / point_cam[2] + camera.cy),
    )


def _observations_for_points(
    cameras: dict[str, CameraParams],
    points_world: np.ndarray,
    *,
    offset_px: tuple[float, float] = (0.0, 0.0),
) -> dict[str, list[dict[str, object]]]:
    observations: dict[str, list[dict[str, object]]] = {}
    for camera_id, camera in cameras.items():
        camera_observations = []
        for blob_index, point in enumerate(points_world):
            uv = _project(camera, point)
            shifted_uv = (uv[0] + offset_px[0], uv[1] + offset_px[1])
            camera_observations.append(
                {
                    "camera_id": camera_id,
                    "blob_index": blob_index,
                    "raw_uv": [shifted_uv[0], shifted_uv[1]],
                    "undistorted_uv": [shifted_uv[0], shifted_uv[1]],
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


def _invalid_pose(timestamp: int) -> RigidBodyPose:
    return RigidBodyPose(
        timestamp=timestamp,
        position=np.zeros(3, dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        valid=False,
    )


def _prime_tracker_into_reacquire(
    estimator: RigidBodyEstimator,
    cameras: dict[str, CameraParams],
    points_world: np.ndarray,
) -> None:
    observations = _observations_for_points(cameras, points_world)
    for index in range(3):
        estimator.process_context(
            points_world,
            1_000_000 + index * 16_000,
            camera_params=cameras,
            observations_by_camera=observations,
        )
    assert estimator.get_tracking_status()["waist"]["mode"] == "continue"
    estimator.trackers["waist"].update(_invalid_pose(1_048_000))
    assert estimator.get_tracking_status()["waist"]["mode"] == "reacquire"


def test_reacquire_guard_shadow_reports_would_reject_without_changing_acceptance() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])
    _prime_tracker_into_reacquire(estimator, cameras, points_world)

    bad_observations = _observations_for_points(cameras, points_world, offset_px=(80.0, 0.0))
    poses = estimator.process_context(
        points_world,
        1_064_000,
        camera_params=cameras,
        observations_by_camera=bad_observations,
    )

    assert poses["waist"].valid is True
    diagnostics = estimator.get_tracking_status()["waist"]
    guard = diagnostics["reacquire_guard"]
    assert guard["evaluated"] is True
    assert guard["enforced"] is False
    assert guard["would_reject"] is True
    assert "insufficient_matched_marker_views" in guard["reason"]
    assert diagnostics["invalid_reason"] == ""


def test_reacquire_guard_enforcement_commits_invalid_pose_when_candidate_fails() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        reacquire_guard_config=ReacquireGuardConfig(enforced=True),
    )
    _prime_tracker_into_reacquire(estimator, cameras, points_world)

    bad_observations = _observations_for_points(cameras, points_world, offset_px=(80.0, 0.0))
    poses = estimator.process_context(
        points_world,
        1_064_000,
        camera_params=cameras,
        observations_by_camera=bad_observations,
    )

    assert poses["waist"].valid is False
    diagnostics = estimator.get_tracking_status()["waist"]
    guard = diagnostics["reacquire_guard"]
    assert guard["evaluated"] is True
    assert guard["enforced"] is True
    assert guard["would_reject"] is True
    assert guard["rejected_count"] == 1
    assert diagnostics["invalid_reason"] == "reprojection_guard_rejected"
