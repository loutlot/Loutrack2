from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import CameraParams, create_dummy_calibration
from host.rigid import (
    KabschEstimator,
    ObjectGatingConfig,
    ReacquireGuardConfig,
    RigidBodyEstimator,
    RigidBodyPose,
    SubsetSolveConfig,
    WAIST_PATTERN,
)


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


def test_partial_marker_correspondence_is_pose_invariant() -> None:
    translation = np.array([0.25, -0.12, 2.5], dtype=np.float64)
    marker_indices = [3, 0, 1]
    observed = WAIST_PATTERN.marker_positions[marker_indices] + translation

    matched_obs, matched_ref, rms_error = KabschEstimator.find_correspondence(
        observed,
        WAIST_PATTERN.marker_positions,
    )
    rotation, position, error = KabschEstimator.estimate(matched_ref, matched_obs)

    assert rms_error < 1e-9
    assert error < 1e-9
    assert np.allclose(rotation, np.eye(3), atol=1e-9)
    assert np.allclose(position, translation, atol=1e-9)


def test_reprojection_scoring_uses_one_to_one_blob_assignment() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = {}
    for camera_id, camera in cameras.items():
        uv = _project(camera, points_world[0])
        observations[camera_id] = [
            {
                "camera_id": camera_id,
                "blob_index": 0,
                "raw_uv": [uv[0], uv[1]],
                "undistorted_uv": [uv[0], uv[1]],
                "area": 4.0,
            }
        ]
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        reprojection_match_gate_px=1000.0,
    )

    estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )

    score = estimator.get_tracking_status()["waist"]["reprojection_score"]
    assert score["scored"] is True
    assert score["matched_marker_views"] == 2
    assert score["missing_marker_views"] == 6
    assert score["duplicate_assignment_count"] == 0


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


class _Frame:
    def __init__(self, blobs: list[dict[str, float]]) -> None:
        self.blobs = blobs


def test_object_conditioned_gating_assigns_predicted_marker_windows() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])
    estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )
    frames = {
        camera_id: _Frame([
            {"x": float(obs["raw_uv"][0]), "y": float(obs["raw_uv"][1]), "area": 4.0}
            for obs in camera_observations
        ])
        for camera_id, camera_observations in observations.items()
    }

    gating = estimator.evaluate_object_conditioned_gating(
        timestamp=1_016_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )["waist"]

    assert gating["evaluated"] is True
    assert gating["prediction_valid"] is True
    assert gating["candidate_window_count"] == 8
    assert gating["assigned_marker_views"] == 8
    assert gating["markers_with_two_or_more_rays"] == WAIST_PATTERN.num_markers
    assert gating["generic_fallback_blob_count"] == 0
    assert gating["pixel_gate_px"] >= ObjectGatingConfig().pixel_min
    assert estimator.get_tracking_status()["waist"]["object_gating"]["assigned_marker_views"] == 8


def test_object_conditioned_gating_can_stay_inactive_outside_reacquire() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(activation_mode="reacquire_only"),
    )
    for index in range(3):
        estimator.process_context(
            points_world,
            1_000_000 + index * 16_000,
            camera_params=cameras,
            observations_by_camera=observations,
        )
    frames = {
        camera_id: _Frame([
            {"x": float(obs["raw_uv"][0]), "y": float(obs["raw_uv"][1]), "area": 4.0}
            for obs in camera_observations
        ])
        for camera_id, camera_observations in observations.items()
    }

    inactive = estimator.evaluate_object_conditioned_gating(
        timestamp=1_048_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )["waist"]
    estimator.trackers["waist"].update(_invalid_pose(1_064_000))
    active = estimator.evaluate_object_conditioned_gating(
        timestamp=1_080_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )["waist"]

    assert inactive["evaluated"] is False
    assert inactive["reason"] == "inactive_mode"
    assert inactive["mode"] == "continue"
    assert active["evaluated"] is True
    assert active["mode"] == "reacquire"


def test_object_gating_enforcement_selects_valid_rigid_hint_pose() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    generic_points = points_world + np.array([0.12, 0.0, 0.0])
    observations = _observations_for_points(cameras, points_world)
    rigid_hint_points = [
        {
            "point": [float(value) for value in point],
            "source": "rigid_hint",
            "rigid_name": "waist",
            "marker_idx": marker_idx,
            "is_virtual": False,
            "contributing_rays": 2,
            "reprojection_errors_px": [0.0, 0.0],
        }
        for marker_idx, point in enumerate(points_world)
    ]
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(enforce=True),
    )

    poses = estimator.process_context(
        generic_points,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
        rigid_hint_triangulated_points=rigid_hint_points,
    )

    assert poses["waist"].valid is True
    assert np.linalg.norm(poses["waist"].position - np.array([0.0, 0.0, 2.5])) < 1e-9
    diagnostics = estimator.get_tracking_status()["waist"]["rigid_hint_pose"]
    assert diagnostics["enforced"] is True
    assert diagnostics["diagnostics_only"] is False
    assert diagnostics["selected_for_pose"] is True
    assert diagnostics["selection_reason"] == "object_gating_enforced"


def test_object_gating_enforcement_keeps_generic_pose_when_hint_quality_is_insufficient() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    generic_points = points_world + np.array([0.12, 0.0, 0.0])
    observations = _observations_for_points(cameras, points_world)
    rigid_hint_points = [
        {
            "point": [float(value) for value in point],
            "source": "rigid_hint",
            "rigid_name": "waist",
            "marker_idx": marker_idx,
            "is_virtual": False,
            "contributing_rays": 2,
            "reprojection_errors_px": [0.0, 0.0],
        }
        for marker_idx, point in enumerate(points_world[:2])
    ]
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(enforce=True),
    )

    poses = estimator.process_context(
        generic_points,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
        rigid_hint_triangulated_points=rigid_hint_points,
    )

    assert poses["waist"].valid is True
    assert np.linalg.norm(poses["waist"].position - np.array([0.12, 0.0, 2.5])) < 1e-9
    diagnostics = estimator.get_tracking_status()["waist"]["rigid_hint_pose"]
    assert diagnostics["enforced"] is True
    assert diagnostics["selected_for_pose"] is False
    assert diagnostics["selection_reason"] in {
        "insufficient_rigid_hint_points",
        "not_enforceable",
    }


def test_subset_solve_config_rejects_unimplemented_adoption_mode() -> None:
    try:
        SubsetSolveConfig(diagnostics_only=False)
    except ValueError as exc:
        assert "diagnostics_only must remain True" in str(exc)
    else:
        raise AssertionError("SubsetSolveConfig should reject diagnostics_only=False")


def test_process_context_reports_rigid_hint_pose_side_by_side_without_commit() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    rigid_hint_points = [
        {
            "point": [float(value) for value in point],
            "source": "rigid_hint",
            "rigid_name": "waist",
            "marker_idx": marker_idx,
            "is_virtual": False,
            "contributing_rays": 2,
            "reprojection_errors_px": [0.0, 0.0],
        }
        for marker_idx, point in enumerate(points_world)
    ]
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])

    poses = estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
        rigid_hint_triangulated_points=rigid_hint_points,
    )

    assert poses["waist"].valid is True
    diagnostics = estimator.get_tracking_status()["waist"]["rigid_hint_pose"]
    assert diagnostics["evaluated"] is True
    assert diagnostics["diagnostics_only"] is True
    assert diagnostics["valid"] is True
    assert diagnostics["generic_valid"] is True
    assert diagnostics["candidate_points"] == WAIST_PATTERN.num_markers
    assert diagnostics["marker_indices"] == [0, 1, 2, 3]
    assert diagnostics["score"]["matched_marker_views"] == 8
    assert diagnostics["position_delta_m"] < 1e-9
    assert diagnostics["rotation_delta_deg"] < 1e-6


def test_process_context_reports_subset_hypotheses_with_noise_points() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    noise = np.array(
        [
            [0.22, -0.18, 2.48],
            [-0.19, 0.21, 2.56],
            [0.31, 0.16, 2.43],
            [-0.28, -0.20, 2.62],
            [0.02, 0.33, 2.51],
            [0.36, -0.02, 2.59],
        ],
        dtype=np.float64,
    )
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])

    estimator.process_context(
        np.vstack([points_world, noise]),
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )

    subset = estimator.get_tracking_status()["waist"]["subset_hypothesis"]
    assert subset["evaluated"] is True
    assert subset["diagnostics_only"] is True
    assert subset["candidate_count"] > 0
    assert subset["valid_candidate_count"] > 0
    assert subset["best"]["score"] > 0.95
    assert subset["best"]["matched_marker_views"] == 8
    assert subset["weighted_solve"]["valid"] is True
