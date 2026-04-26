from __future__ import annotations

import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.rigid import (
    PointClusterer,
    PositionContinuityGuardConfig,
    PoseContinuityGuardConfig,
    RigidBodyEstimator,
    RigidBodyPose,
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
