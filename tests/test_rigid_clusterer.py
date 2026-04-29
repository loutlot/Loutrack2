from __future__ import annotations

import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.rigid import (
    MarkerPattern,
    ObjectGatingConfig,
    PointClusterer,
    PositionContinuityGuardConfig,
    PoseContinuityGuardConfig,
    RigidBodyEstimator,
    RigidBodyPose,
    RigidBodyTracker,
    TrackMode,
    TrackModeConfig,
    WAIST_PATTERN,
)


def test_point_clusterer_defaults_to_80mm_radius() -> None:
    clusterer = PointClusterer()

    assert clusterer.eps == 0.08
    assert clusterer.cluster_radius_m == 0.08


def test_point_clusterer_groups_points_within_80mm_chain() -> None:
    clusterer = PointClusterer()
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.06, 0.0, 0.0],
            [0.12, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.56, 0.0, 0.0],
            [0.62, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    clusters = clusterer.cluster(points)

    assert sorted(len(cluster) for cluster in clusters) == [3, 3]


def test_rigid_body_estimator_passes_80mm_radius_to_clusterer() -> None:
    estimator = RigidBodyEstimator()

    assert estimator.cluster_radius_m == 0.08
    assert estimator.clusterer.eps == 0.08
    assert estimator.max_rms_error_m == 0.055


def test_single_pattern_estimator_falls_back_to_full_points_when_80mm_clusters_split() -> None:
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN], cluster_radius_m=0.08)
    points = WAIST_PATTERN.marker_positions + np.array([1.0, 2.0, 3.0], dtype=np.float64)

    pose = estimator.process_points(points, timestamp=123)["waist"]

    assert pose.valid is True
    assert pose.observed_markers == WAIST_PATTERN.num_markers


def test_multi_pattern_boot_finds_rigids_when_marker_clusters_split() -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=np.array(
            [
                [-0.16720382613672097, 0.021044859056731813, -0.0003093655967190481],
                [-0.03215422561435377, -0.101157870515302, 0.0001515195503982332],
                [0.057345780432333765, -0.006039561048765074, -0.0006549956316947575],
                [0.14201227131874097, 0.08615257250733527, 0.0008128416780155724],
            ],
            dtype=np.float64,
        ),
    )
    waist_offset = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    wand_offset = np.array([4.0, 5.0, 6.0], dtype=np.float64)
    mixed_points = np.vstack(
        [
            WAIST_PATTERN.marker_positions + waist_offset,
            wand_pattern.marker_positions + wand_offset,
        ]
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        cluster_radius_m=0.08,
        subset_diagnostics_mode="off",
    )

    poses = estimator.process_points(mixed_points, timestamp=123)

    assert poses["waist"].valid is True
    assert poses["wand"].valid is True
    assert poses["waist"].observed_markers == WAIST_PATTERN.num_markers
    assert poses["wand"].observed_markers == wand_pattern.num_markers
    assert np.allclose(poses["waist"].position, waist_offset, atol=1e-9)
    assert np.allclose(poses["wand"].position, wand_offset, atol=1e-9)


def test_multi_pattern_boot_finds_rigids_among_32_points() -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=np.array(
            [
                [-0.16720382613672097, 0.021044859056731813, -0.0003093655967190481],
                [-0.03215422561435377, -0.101157870515302, 0.0001515195503982332],
                [0.057345780432333765, -0.006039561048765074, -0.0006549956316947575],
                [0.14201227131874097, 0.08615257250733527, 0.0008128416780155724],
            ],
            dtype=np.float64,
        ),
    )
    rng = np.random.default_rng(123)
    noise_points = rng.uniform(-3.0, 3.0, size=(24, 3))
    noise_points[:, 2] += 8.0
    waist_offset = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    wand_offset = np.array([4.0, 5.0, 6.0], dtype=np.float64)
    mixed_points = np.vstack(
        [
            noise_points[:9],
            WAIST_PATTERN.marker_positions + waist_offset,
            noise_points[9:17],
            wand_pattern.marker_positions + wand_offset,
            noise_points[17:],
        ]
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        cluster_radius_m=0.08,
        subset_diagnostics_mode="off",
    )

    poses = estimator.process_points(mixed_points, timestamp=123)

    assert poses["waist"].valid is True
    assert poses["wand"].valid is True
    assert np.allclose(poses["waist"].position, waist_offset, atol=1e-9)
    assert np.allclose(poses["wand"].position, wand_offset, atol=1e-9)


def test_multi_pattern_boot_keeps_rigid_with_one_missing_marker() -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=np.array(
            [
                [-0.16720382613672097, 0.021044859056731813, -0.0003093655967190481],
                [-0.03215422561435377, -0.101157870515302, 0.0001515195503982332],
                [0.057345780432333765, -0.006039561048765074, -0.0006549956316947575],
                [0.14201227131874097, 0.08615257250733527, 0.0008128416780155724],
            ],
            dtype=np.float64,
        ),
    )
    waist_offset = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    wand_offset = np.array([4.0, 5.0, 6.0], dtype=np.float64)
    mixed_points = np.vstack(
        [
            WAIST_PATTERN.marker_positions + waist_offset,
            wand_pattern.marker_positions[:3] + wand_offset,
        ]
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        cluster_radius_m=0.08,
        subset_diagnostics_mode="off",
    )

    poses = estimator.process_points(mixed_points, timestamp=123)

    assert poses["waist"].valid is True
    assert poses["wand"].valid is True
    assert poses["wand"].observed_markers == 3
    assert np.allclose(poses["wand"].position, wand_offset, atol=1e-9)


def test_rigid_body_estimator_rejects_pose_over_max_rms(monkeypatch: pytest.MonkeyPatch) -> None:
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN], max_rms_error_m=0.05)
    points = WAIST_PATTERN.marker_positions + np.array([1.0, 2.0, 3.0], dtype=np.float64)

    def _fake_estimate_pose(_points, _pattern, timestamp):  # noqa: ANN001
        return RigidBodyPose(
            timestamp=timestamp,
            position=np.array([1.0, 2.0, 3.0], dtype=np.float64),
            rotation=np.eye(3, dtype=np.float64),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            rms_error=0.08,
            observed_markers=4,
            valid=True,
        )

    monkeypatch.setattr(estimator, "estimate_pose", _fake_estimate_pose)
    pose = estimator.process_points(points, timestamp=123)["waist"]

    assert pose.valid is False
    assert pose.observed_markers == 0


def test_object_gated_continue_rejects_bad_generic_reprojection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(enabled=True, enforce=True),
        subset_diagnostics_mode="off",
    )
    estimator.trackers["waist"].mode_config = TrackModeConfig(boot_consecutive_accepts=1)
    points = WAIST_PATTERN.marker_positions + np.array([1.0, 2.0, 3.0], dtype=np.float64)

    scores = iter(
        [
            {
                "scored": True,
                "reason": "ok",
                "score": 0.9,
                "mean_error_px": 1.0,
                "p95_error_px": 2.0,
                "matched_marker_views": 8,
                "expected_marker_views": 8,
                "missing_marker_views": 0,
                "duplicate_assignment_count": 0,
            },
            {
                "scored": True,
                "reason": "ok",
                "score": 0.2,
                "mean_error_px": 9.0,
                "p95_error_px": 10.0,
                "matched_marker_views": 8,
                "expected_marker_views": 8,
                "missing_marker_views": 0,
                "duplicate_assignment_count": 0,
            },
        ]
    )

    def _fake_score(*_args, **_kwargs):  # noqa: ANN002, ANN003
        return next(scores)

    monkeypatch.setattr(estimator, "_score_pose_reprojection", _fake_score)
    first = estimator.process_points(points, timestamp=1_000_000)["waist"]
    second = estimator.process_points(points + np.array([0.2, 0.0, 0.0]), timestamp=1_010_000)[
        "waist"
    ]

    status = estimator.get_tracking_status()["waist"]
    assert first.valid is True
    assert second.valid is False
    assert status["last_mode_reason"] == "continue_measurement_rejected"
    assert status["invalid_reason"].startswith(
        "generic_continue_reprojection_guard_rejected"
    )
    assert "mean_reprojection_error_too_high" in status["invalid_reason"]


def test_object_gated_continue_rejects_generic_pose_using_other_rigid_blobs() -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=WAIST_PATTERN.marker_positions + np.array([0.25, 0.0, 0.0]),
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        object_gating_config=ObjectGatingConfig(enabled=True, enforce=True),
        subset_diagnostics_mode="off",
    )
    waist_tracker = estimator.trackers["waist"]
    wand_tracker = estimator.trackers["wand"]
    waist_tracker.mode_config = TrackModeConfig(boot_consecutive_accepts=1)
    wand_tracker.mode_config = TrackModeConfig(boot_consecutive_accepts=1)
    pose = RigidBodyPose(
        timestamp=1_000_000,
        position=np.array([1.0, 2.0, 3.0], dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.001,
        observed_markers=4,
        valid=True,
    )
    waist_tracker.update(pose)
    wand_tracker.update(pose)
    wand_tracker.record_object_gating(
        {
            "evaluated": True,
            "per_camera": {
                "pi-cam-01": {
                    "assignments": [
                        {"marker_idx": 0, "blob_index": 4},
                        {"marker_idx": 1, "blob_index": 5},
                    ]
                },
                "pi-cam-02": {
                    "assignments": [
                        {"marker_idx": 0, "blob_index": 6},
                        {"marker_idx": 1, "blob_index": 7},
                    ]
                },
            },
        }
    )
    score = {
        "scored": True,
        "reason": "ok",
        "matched_marker_views": 8,
        "missing_marker_views": 0,
        "mean_error_px": 1.0,
        "p95_error_px": 2.0,
        "duplicate_assignment_count": 0,
        "matched_observations": [
            {"camera_id": "pi-cam-01", "blob_index": 4},
            {"camera_id": "pi-cam-02", "blob_index": 6},
        ],
    }

    reasons = estimator._generic_continue_reject_reasons(
        waist_tracker,
        selected_source="pose_continuity_guard",
        selected_score=score,
        hint_diagnostic={"selected_for_pose": False},
    )

    assert "matched_other_rigid_gate_blobs" in reasons


def test_object_gated_rigid_hint_rejects_pose_using_other_rigid_blobs() -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=WAIST_PATTERN.marker_positions + np.array([0.25, 0.0, 0.0]),
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        object_gating_config=ObjectGatingConfig(enabled=True, enforce=True),
        subset_diagnostics_mode="off",
    )
    waist_tracker = estimator.trackers["waist"]
    wand_tracker = estimator.trackers["wand"]
    wand_tracker.record_object_gating(
        {
            "evaluated": True,
            "per_camera": {
                "pi-cam-01": {"assignments": [{"marker_idx": 0, "blob_index": 4}]},
                "pi-cam-02": {"assignments": [{"marker_idx": 0, "blob_index": 6}]},
            },
        }
    )
    diagnostic = {
        "candidate_points": 4,
        "valid": True,
        "score": {
            "scored": True,
            "matched_marker_views": 8,
            "matched_observations": [
                {"camera_id": "pi-cam-01", "blob_index": 4},
                {"camera_id": "pi-cam-02", "blob_index": 6},
            ],
        },
    }

    selected = estimator._should_select_rigid_hint_pose(waist_tracker, diagnostic)

    assert selected is False
    assert diagnostic["selection_reason"] == "matched_other_rigid_gate_blobs"


def test_pose_continuity_guard_holds_rotation_only_for_low_marker_rotation_jump(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        pose_continuity_guard_config=PoseContinuityGuardConfig(
            enabled=True,
            enforced=True,
            max_rotation_innovation_deg=30.0,
            max_angular_velocity_deg_s=1000.0,
        ),
    )
    good_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    bad_xyzw = Rotation.from_euler("z", 120.0, degrees=True).as_quat()
    bad_quat = np.array([bad_xyzw[3], bad_xyzw[0], bad_xyzw[1], bad_xyzw[2]], dtype=np.float64)

    def _fake_estimate_pose(points, _pattern, timestamp):  # noqa: ANN001
        observed = len(points)
        quat = good_quat if observed == WAIST_PATTERN.num_markers else bad_quat
        position = (
            np.array([1.0, 2.0, 3.0], dtype=np.float64)
            if observed == WAIST_PATTERN.num_markers
            else np.array([4.0, 5.0, 6.0], dtype=np.float64)
        )
        return RigidBodyPose(
            timestamp=timestamp,
            position=position,
            rotation=Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix(),
            quaternion=quat.copy(),
            rms_error=0.001,
            observed_markers=observed,
            valid=True,
        )

    monkeypatch.setattr(estimator, "estimate_pose", _fake_estimate_pose)
    first = estimator.process_points(
        WAIST_PATTERN.marker_positions + np.array([1.0, 2.0, 3.0], dtype=np.float64),
        timestamp=1_000_000,
    )["waist"]
    second = estimator.process_points(
        WAIST_PATTERN.marker_positions[:3] + np.array([1.0, 2.0, 3.0], dtype=np.float64),
        timestamp=1_008_000,
    )["waist"]

    guard = estimator.get_tracking_status()["waist"]["pose_continuity_guard"]
    assert first.valid is True
    assert second.valid is True
    assert guard["would_reject"] is True
    assert guard["held_prediction"] is True
    assert guard["held_rotation"] is True
    assert guard["held_count"] == 1
    assert np.allclose(second.position, np.array([4.0, 5.0, 6.0], dtype=np.float64))
    assert np.allclose(second.quaternion, good_quat)


def test_position_continuity_guard_clamps_low_marker_acceleration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        position_continuity_guard_config=PositionContinuityGuardConfig(
            enabled=True,
            enforced=True,
            max_accel_m_s2=10.0,
            max_velocity_m_s=100.0,
        ),
    )
    quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def _fake_estimate_pose(points, _pattern, timestamp):  # noqa: ANN001
        observed = len(points)
        position = (
            np.zeros(3, dtype=np.float64)
            if observed == WAIST_PATTERN.num_markers
            else np.array([1.0, 0.0, 0.0], dtype=np.float64)
        )
        return RigidBodyPose(
            timestamp=timestamp,
            position=position,
            rotation=np.eye(3, dtype=np.float64),
            quaternion=quat.copy(),
            rms_error=0.001,
            observed_markers=observed,
            valid=True,
        )

    monkeypatch.setattr(estimator, "estimate_pose", _fake_estimate_pose)
    first = estimator.process_points(WAIST_PATTERN.marker_positions, timestamp=1_000_000)[
        "waist"
    ]
    second = estimator.process_points(WAIST_PATTERN.marker_positions[:3], timestamp=1_010_000)[
        "waist"
    ]

    guard = estimator.get_tracking_status()["waist"]["position_continuity_guard"]
    assert first.valid is True
    assert second.valid is True
    assert guard["would_reject"] is True
    assert guard["clamped_position"] is True
    assert guard["clamped_count"] == 1
    assert np.allclose(second.position, np.array([0.001, 0.0, 0.0], dtype=np.float64))
    assert np.allclose(second.quaternion, quat)


def test_simultaneous_reacquire_shared_blob_observations_commit_one_loser_invalid(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=WAIST_PATTERN.marker_positions + np.array([0.25, 0.0, 0.0]),
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        object_gating_config=ObjectGatingConfig(enabled=True, enforce=True),
        rigid_candidate_separation_enabled=True,
        subset_diagnostics_mode="off",
    )
    if not hasattr(estimator, "_resolve_frame_local_candidate_ownership"):
        pytest.xfail("pending frame-local rigid candidate ownership resolver")

    for tracker in estimator.trackers.values():
        tracker.mode_config = TrackModeConfig(
            boot_consecutive_accepts=1,
            reacquire_consecutive_accepts=1,
        )
        tracker.update(_test_pose(1_000_000, [0.0, 0.0, 2.5]))
        tracker.update(_test_pose(1_016_000, [0.0, 0.0, 2.5], valid=False))

    shared_score = {
        "scored": True,
        "reason": "ok",
        "score": 0.95,
        "matched_marker_views": 8,
        "expected_marker_views": 8,
        "missing_marker_views": 0,
        "mean_error_px": 1.0,
        "p95_error_px": 2.0,
        "duplicate_assignment_count": 0,
        "matched_observations": [
            {"camera_id": "pi-cam-01", "blob_index": 4},
            {"camera_id": "pi-cam-02", "blob_index": 6},
            {"camera_id": "pi-cam-01", "blob_index": 5},
            {"camera_id": "pi-cam-02", "blob_index": 7},
        ],
    }
    monkeypatch.setattr(
        estimator,
        "_score_pose_reprojection",
        lambda *_args, **_kwargs: dict(shared_score),
    )
    points = WAIST_PATTERN.marker_positions + np.array([0.02, 0.0, 2.5], dtype=np.float64)
    rigid_hints = [
        {
            "rigid_name": rigid_name,
            "marker_idx": marker_idx,
            "point": point.copy(),
            "contributing_rays": 2,
        }
        for rigid_name in ("waist", "wand")
        for marker_idx, point in enumerate(points)
    ]

    poses = estimator.process_context(
        np.empty((0, 3), dtype=np.float64),
        1_032_000,
        camera_params={"pi-cam-01": object(), "pi-cam-02": object()},
        observations_by_camera={"pi-cam-01": [object()], "pi-cam-02": [object()]},
        rigid_hint_triangulated_points=rigid_hints,
    )

    committed = [name for name, pose in poses.items() if pose.valid]
    rejected = [name for name, pose in poses.items() if not pose.valid]
    assert len(committed) == 1
    assert len(rejected) == 1
    assert estimator.get_tracking_status()[rejected[0]]["invalid_reason"].startswith(
        "frame_local_blob_ownership_conflict"
    )


def test_reacquire_large_innovation_candidate_does_not_mutate_tracker_state() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)
    if not hasattr(tracker, "_stage_reacquire_candidate"):
        pytest.xfail("pending side-effect-free reacquire candidate staging helper")

    tracker.mode_config = TrackModeConfig(
        boot_consecutive_accepts=1,
        reacquire_consecutive_accepts=2,
    )
    tracker.update(_test_pose(1_000_000, [0.0, 0.0, 2.5]))
    tracker.update(_test_pose(1_016_000, [0.0, 0.0, 2.5], valid=False))
    before_prediction = tracker.peek_prediction(1_032_000)
    before_latest = tracker.get_latest_pose()

    tracker.update(_test_pose(1_032_000, [1.0, 0.0, 2.5]))

    diagnostics = tracker.get_diagnostics()
    after_prediction = tracker.peek_prediction(1_048_000)
    after_latest = tracker.get_latest_pose()
    assert diagnostics["mode"] == "reacquire"
    assert diagnostics["last_mode_reason"] == "reacquire_candidate_large_innovation"
    assert before_latest is not None
    assert after_latest is not None
    assert np.allclose(after_latest.position, before_latest.position)
    assert np.allclose(after_prediction.position, before_prediction.position)


def test_unowned_boot_rigid_does_not_generic_boot_from_another_rigids_hints() -> None:
    wand_pattern = MarkerPattern(
        name="wand",
        marker_positions=WAIST_PATTERN.marker_positions + np.array([0.25, 0.0, 0.0]),
    )
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN, wand_pattern],
        object_gating_config=ObjectGatingConfig(enabled=True, enforce=True),
        rigid_candidate_separation_enabled=True,
        subset_diagnostics_mode="off",
    )
    waist_tracker = estimator.trackers["waist"]
    waist_tracker.mode_config = TrackModeConfig(boot_consecutive_accepts=1)
    waist_tracker.update(_test_pose(1_000_000, [0.0, 0.0, 2.5]))

    points = WAIST_PATTERN.marker_positions + np.array([0.02, 0.0, 2.5], dtype=np.float64)
    rigid_hints = [
        {
            "rigid_name": "waist",
            "marker_idx": marker_idx,
            "point": point.copy(),
            "contributing_rays": 2,
        }
        for marker_idx, point in enumerate(points)
    ]

    poses = estimator.process_context(
        points,
        1_016_000,
        camera_params=None,
        observations_by_camera=None,
        rigid_hint_triangulated_points=rigid_hints,
    )

    assert estimator.estimate_pose(points, wand_pattern, 1_016_000).valid is True
    assert poses["wand"].valid is False
    assert estimator.trackers["wand"].track_count == 0


def test_lost_previously_tracked_rigid_rejects_unscored_generic_recovery() -> None:
    estimator = RigidBodyEstimator(
        patterns=[WAIST_PATTERN],
        object_gating_config=ObjectGatingConfig(enabled=True, enforce=True),
        subset_diagnostics_mode="off",
    )
    tracker = estimator.trackers["waist"]
    tracker.mode_config = TrackModeConfig(
        boot_consecutive_accepts=1,
        reacquire_lost_frames=1,
    )
    tracker.update(_test_pose(1_000_000, [0.0, 0.0, 2.5]))
    tracker.update(_test_pose(1_016_000, [0.0, 0.0, 2.5], valid=False))
    tracker.update(_test_pose(1_032_000, [0.0, 0.0, 2.5], valid=False))
    assert tracker.mode == TrackMode.LOST

    wrong_cluster = WAIST_PATTERN.marker_positions[:3] + np.array(
        [1.0, 0.0, 2.5],
        dtype=np.float64,
    )
    before_latest = tracker.get_latest_pose()

    pose = estimator.process_context(
        wrong_cluster,
        1_048_000,
        camera_params=None,
        observations_by_camera=None,
    )["waist"]

    after_latest = tracker.get_latest_pose()
    assert pose.valid is False
    assert before_latest is not None
    assert after_latest is not None
    assert np.allclose(after_latest.position, before_latest.position)
    assert estimator.get_tracking_status()["waist"]["invalid_reason"].startswith(
        "strict_reacquire_rejected"
    )
    assert "score_not_available" in estimator.get_tracking_status()["waist"]["invalid_reason"]


def _test_pose(
    timestamp: int,
    position: list[float],
    *,
    valid: bool = True,
) -> RigidBodyPose:
    return RigidBodyPose(
        timestamp=timestamp,
        position=np.asarray(position, dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.001 if valid else 0.0,
        observed_markers=WAIST_PATTERN.num_markers if valid else 0,
        valid=valid,
    )
