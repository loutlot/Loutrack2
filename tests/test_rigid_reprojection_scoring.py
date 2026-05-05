from __future__ import annotations

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import CameraParams, create_dummy_calibration
from host.rigid import (
    KabschEstimator,
    ObjectGatingConfig,
    ReacquireGuardConfig,
    RigidBodyEstimator,
    RigidBodyPose,
    SubsetSolveConfig,
    TrackMode,
    WAIST_PATTERN,
    _quaternion_delta_rotvec,
    compact_object_gating_diagnostics,
    marker_pattern_from_points,
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


def test_quaternion_delta_rotvec_matches_rotation_delta() -> None:
    previous = Rotation.from_euler("xyz", [0.1, -0.2, 0.3])
    current = Rotation.from_euler("xyz", [0.18, -0.05, 0.41])
    previous_xyzw = previous.as_quat()
    current_xyzw = current.as_quat()

    actual = _quaternion_delta_rotvec(
        np.array([current_xyzw[3], current_xyzw[0], current_xyzw[1], current_xyzw[2]]),
        np.array([previous_xyzw[3], previous_xyzw[0], previous_xyzw[1], previous_xyzw[2]]),
    )
    expected = (current * previous.inv()).as_rotvec()

    assert np.allclose(actual, expected)


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


def test_reprojection_scoring_broadens_acceptance_for_small_blobs() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world, offset_px=(14.0, 0.0))
    for camera_observations in observations.values():
        for observation in camera_observations:
            observation["area"] = 1.0
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])

    poses = estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )

    assert poses["waist"].valid is True
    score = estimator.get_tracking_status()["waist"]["reprojection_score"]
    assert score["matched_marker_views"] == 8
    assert score["mean_error_px"] > estimator.reprojection_match_gate_px
    assert score["mean_normalized_error"] < score["mean_error_px"]
    assert all(
        observation["uncertainty_px"] > 1.0
        for observation in score["matched_observations"]
    )


def test_process_points_keeps_3d_only_path_without_2d_score() -> None:
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])

    poses = estimator.process_points(points_world, 1_000_000)

    assert poses["waist"].valid is True
    score = estimator.get_tracking_status()["waist"]["reprojection_score"]
    assert score["scored"] is False
    assert score["reason"] == "no_2d_context"


def test_subset_sampled_mode_skips_diagnostics_without_changing_committed_pose() -> None:
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    full = RigidBodyEstimator(patterns=[WAIST_PATTERN], subset_diagnostics_mode="full")
    sampled = RigidBodyEstimator(patterns=[WAIST_PATTERN], subset_diagnostics_mode="sampled")

    for timestamp in (1_000_000, 1_010_000, 1_020_000):
        full.process_points(points_world, timestamp)
        sampled.process_points(points_world, timestamp)
    full_pose = full.process_points(points_world, 1_030_000)["waist"]
    sampled_pose = sampled.process_points(points_world, 1_030_000)["waist"]

    assert full_pose.valid is True
    assert sampled_pose.valid is True
    assert np.allclose(sampled_pose.position, full_pose.position)
    assert np.allclose(sampled_pose.quaternion, full_pose.quaternion)
    subset = sampled.get_tracking_status()["waist"]["subset_hypothesis"]
    assert subset["evaluated"] is False
    assert subset["sampled"] is True
    assert sampled.get_variant_metrics()["subset_skipped_count"] == 1


def test_subset_time_budget_truncates_diagnostics_without_invalidating_pose() -> None:
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        subset_diagnostics_mode="full",
        subset_time_budget_ms=0.000001,
        subset_max_hypotheses=512,
    )
    points = WAIST_PATTERN.marker_positions.copy()

    poses = estimator.process_context(points, timestamp=123)
    subset = estimator.get_tracking_status()["waist"]["subset_hypothesis"]

    assert poses["waist"].valid is True
    assert subset["evaluated"] is True
    assert subset["time_budget_exceeded"] is True
    assert subset["truncated"] is True
    assert estimator.get_variant_metrics()["subset_budget_exceeded_count"] == 1


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


def test_five_marker_partial_correspondence_tolerates_missing_and_scrambled_points() -> None:
    pattern = marker_pattern_from_points(
        "five_marker_correspondence_test",
        np.vstack(
            [
                WAIST_PATTERN.marker_positions,
                np.array([[0.0, -0.055, 0.035]], dtype=np.float64),
            ]
        ),
        marker_diameter=WAIST_PATTERN.marker_diameter,
    )
    rotation = Rotation.from_euler("xyz", [0.22, -0.18, 0.31]).as_matrix()
    translation = np.array([0.18, -0.09, 2.45], dtype=np.float64)
    marker_indices = [4, 0, 2, 1]
    observed = (
        rotation @ pattern.marker_positions[marker_indices].T
    ).T + translation.reshape(1, 3)

    matched_obs, matched_ref, rms_error = KabschEstimator.find_correspondence(
        observed,
        pattern.marker_positions,
    )
    estimated_rotation, estimated_position, error = KabschEstimator.estimate(
        matched_ref,
        matched_obs,
    )

    assert rms_error < 1e-9
    assert error < 1e-9
    assert np.allclose(estimated_rotation, rotation, atol=1e-9)
    assert np.allclose(estimated_position, translation, atol=1e-9)


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


def test_object_conditioned_gating_marks_ambiguous_nearby_blobs() -> None:
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
    frames = {}
    for camera_id, camera_observations in observations.items():
        blobs = [
            {"x": float(obs["raw_uv"][0]), "y": float(obs["raw_uv"][1]), "area": 4.0}
            for obs in camera_observations
        ]
        blobs.append({"x": blobs[0]["x"] + 0.25, "y": blobs[0]["y"], "area": 4.0})
        frames[camera_id] = _Frame(blobs)

    gating = estimator.evaluate_object_conditioned_gating(
        timestamp=1_016_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )["waist"]

    assert gating["assigned_marker_views"] == 8
    assert gating["ambiguous_assignment_count"] == 2
    assert gating["unmatched_marker_views"] == 0
    assert gating["body_assignment"]["policy"] == "keep_body_assignment_mark_local_ambiguity"
    assert all(
        camera["ambiguous_assignment_count"] == 1
        for camera in gating["per_camera"].values()
    )
    assert sum(
        int(assignment["ambiguous"])
        for camera in gating["per_camera"].values()
        for assignment in camera["assignments"]
    ) == 2


def test_object_conditioned_gating_marks_blob_diameter_overlap() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(
            ambiguous_blob_min_separation_px=0.60,
            ambiguous_blob_diameter_overlap_ratio=0.50,
        ),
    )
    estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )
    frames = {}
    for camera_id, camera_observations in observations.items():
        blobs = [
            {"x": float(obs["raw_uv"][0]), "y": float(obs["raw_uv"][1]), "area": 25.0}
            for obs in camera_observations
        ]
        blobs.append({"x": blobs[0]["x"] + 1.25, "y": blobs[0]["y"], "area": 25.0})
        frames[camera_id] = _Frame(blobs)

    gating = estimator.evaluate_object_conditioned_gating(
        timestamp=1_016_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )["waist"]

    assert gating["assigned_marker_views"] == 8
    assert gating["ambiguous_assignment_count"] == 2
    assert gating["unmatched_marker_views"] == 0
    assert sum(
        int(assignment["ambiguous"])
        for camera in gating["per_camera"].values()
        for assignment in camera["assignments"]
    ) == 2


def test_object_conditioned_gating_marks_low_marker_assignment_margin() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    points_world = WAIST_PATTERN.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(
            pixel_min=20.0,
            pixel_max=20.0,
            ambiguous_blob_min_separation_px=0.0,
            ambiguous_blob_diameter_overlap_ratio=0.0,
            ambiguous_marker_assignment_min_margin_px=2.0,
        ),
    )
    estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )
    frames = {}
    for camera_id, camera_observations in observations.items():
        blobs = [
            {"x": float(obs["raw_uv"][0]), "y": float(obs["raw_uv"][1]), "area": 4.0}
            for obs in camera_observations
        ]
        midpoint = 0.5 * (
            np.asarray(camera_observations[0]["raw_uv"], dtype=np.float64)
            + np.asarray(camera_observations[1]["raw_uv"], dtype=np.float64)
        )
        blobs[0] = {"x": float(midpoint[0]), "y": float(midpoint[1]), "area": 4.0}
        frames[camera_id] = _Frame(blobs)

    gating = estimator.evaluate_object_conditioned_gating(
        timestamp=1_016_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )["waist"]

    assert gating["assigned_marker_views"] == 8
    assert gating["ambiguous_assignment_count"] == 2
    assert gating["marker_margin_assignment_count"] == 2
    assert gating["unmatched_marker_views"] == 0
    assert sum(
        int(assignment["marker_margin_ambiguous"])
        for camera in gating["per_camera"].values()
        for assignment in camera["assignments"]
    ) == 2


def test_object_gated_generic_skip_requires_complete_unambiguous_body_evidence() -> None:
    pattern = marker_pattern_from_points(
        "five_marker_skip_test",
        np.vstack(
            [
                WAIST_PATTERN.marker_positions,
                np.array([[0.0, -0.055, 0.035]], dtype=np.float64),
            ]
        ),
        marker_diameter=WAIST_PATTERN.marker_diameter,
    )
    estimator = RigidBodyEstimator(
        patterns=[pattern],
        object_gating_config=ObjectGatingConfig(
            skip_generic_search_when_object_gated=True,
        ),
    )
    tracker = estimator.trackers[pattern.name]
    tracker._mode = TrackMode.CONTINUE
    tracker.track_count = 3

    common = {
        "object_gating_enforced": True,
        "hint_marker_count": pattern.num_markers,
        "gating_evaluated": True,
        "gating_camera_count": 2,
        "gating_assigned_marker_views": pattern.num_markers * 2,
        "gating_marker_margin_assignment_count": 0,
        "gating_duplicate_assignment_count": 0,
    }

    assert estimator._should_skip_generic_search_for_object_gated_continue(
        pattern,
        tracker,
        gating_markers_with_two_or_more_rays=pattern.num_markers,
        gating_ambiguous_assignment_count=0,
        **common,
    )
    assert not estimator._should_skip_generic_search_for_object_gated_continue(
        pattern,
        tracker,
        gating_markers_with_two_or_more_rays=pattern.num_markers - 1,
        gating_ambiguous_assignment_count=0,
        **common,
    )
    assert not estimator._should_skip_generic_search_for_object_gated_continue(
        pattern,
        tracker,
        gating_markers_with_two_or_more_rays=pattern.num_markers,
        gating_ambiguous_assignment_count=1,
        **common,
    )


def test_compact_object_gating_diagnostics_omits_hot_path_assignment_lists() -> None:
    payload = {
        "evaluated": True,
        "mode": "continue",
        "assigned_marker_views": 10,
        "markers_with_two_or_more_rays": 5,
        "per_camera": {
            "cam0": {
                "assigned_marker_views": 5,
                "body_shifted_assignment": True,
                "assignments": [{"marker_idx": 0, "blob_index": 2}],
                "body_level_2d_nbest": [{"score": 1.0}],
            }
        },
    }

    compact = compact_object_gating_diagnostics(payload)
    detailed = compact_object_gating_diagnostics(payload, include_detail=True)

    assert compact["assigned_marker_views"] == 10
    assert compact["per_camera"]["cam0"]["assigned_marker_views"] == 5
    assert "assignments" not in compact["per_camera"]["cam0"]
    assert "body_level_2d_nbest" not in compact["per_camera"]["cam0"]
    assert detailed["per_camera"]["cam0"]["assignments"]


def test_body_level_2d_nbest_recovers_translated_constellation() -> None:
    cameras = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    pattern = marker_pattern_from_points(
        "five_marker_test",
        np.vstack(
            [
                WAIST_PATTERN.marker_positions,
                np.array([[0.0, -0.055, 0.035]], dtype=np.float64),
            ]
        ),
        marker_diameter=WAIST_PATTERN.marker_diameter,
    )
    points_world = pattern.marker_positions + np.array([0.0, 0.0, 2.5])
    observations = _observations_for_points(cameras, points_world)
    estimator = RigidBodyEstimator(
        patterns=[pattern],
        object_gating_config=ObjectGatingConfig(
            pixel_min=4.0,
            pixel_max=4.0,
            body_level_2d_recovery=True,
        ),
    )
    estimator.process_context(
        points_world,
        1_000_000,
        camera_params=cameras,
        observations_by_camera=observations,
    )
    frames = {
        camera_id: _Frame(
            [
                {
                    "x": float(obs["raw_uv"][0]) + 18.0,
                    "y": float(obs["raw_uv"][1]) - 7.0,
                    "area": 4.0,
                }
                for obs in camera_observations
            ]
        )
        for camera_id, camera_observations in observations.items()
    }

    gating = estimator.evaluate_object_conditioned_gating(
        timestamp=1_016_000,
        camera_params=cameras,
        frames_by_camera=frames,
    )[pattern.name]

    assert gating["assigned_marker_views"] == pattern.num_markers * len(cameras)
    assert gating["markers_with_two_or_more_rays"] == pattern.num_markers
    assert all(
        camera_payload["body_shifted_assignment"] is True
        for camera_payload in gating["per_camera"].values()
    )
    assert all(
        camera_payload["body_level_2d_nbest_candidate_count"] >= 1
        for camera_payload in gating["per_camera"].values()
    )
    assert all(
        camera_payload["body_level_2d_nbest"]
        for camera_payload in gating["per_camera"].values()
    )


def test_body_level_2d_cached_bin_can_early_exit_on_four_of_five() -> None:
    projected = np.asarray(
        [
            [10.0, 10.0],
            [30.0, 10.0],
            [10.0, 30.0],
            [30.0, 30.0],
            [20.0, 20.0],
        ],
        dtype=np.float64,
    )
    offset = np.asarray([14.0, -6.0], dtype=np.float64)
    blobs = projected[:4] + offset.reshape(1, 2)

    result = RigidBodyEstimator._translated_body_assignment(
        projected,
        blobs,
        np.ones(len(blobs), dtype=np.float64),
        pixel_gate_px=4.0,
        max_offsets=8,
        nearest_per_marker=2,
        assignment_nearest_per_marker=2,
        max_distance_scale=0.12,
        marker_count=5,
        seed_offsets=[offset],
        early_accept_coverage=4,
    )

    assert result is not None
    assert result["early_accepted"] is True
    assert result["candidate_count"] == 1
    assert result["coverage"] == 4
    assert result["offset_source"] == "cache"


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
