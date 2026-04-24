from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.rigid import RigidBodyPose, RigidBodyTracker, WAIST_PATTERN


def _pose(timestamp: int, x: float, *, valid: bool = True) -> RigidBodyPose:
    return RigidBodyPose(
        timestamp=timestamp,
        position=np.array([x, 0.0, 0.0], dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.001 if valid else 0.0,
        observed_markers=4 if valid else 0,
        valid=valid,
    )


def test_rigid_body_tracker_records_valid_lost_reacquire_stats() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)

    tracker.update(_pose(1_000_000, 0.0, valid=True))
    tracker.update(_pose(1_016_000, 0.01, valid=True))
    tracker.update(_pose(1_032_000, 0.01, valid=False))
    tracker.update(_pose(1_048_000, 0.15, valid=True))

    diagnostics = tracker.get_diagnostics()

    assert diagnostics["valid"] is True
    assert diagnostics["reacquire_count"] == 1
    assert diagnostics["short_valid_count"] == 1
    assert diagnostics["short_valid_ratio"] == 1.0
    assert diagnostics["pose_jump_count"] == 1
    assert diagnostics["last_pose_jump_m"] > 0.10
    assert diagnostics["current_valid_run_frames"] == 1
    assert diagnostics["current_lost_run_frames"] == 0


def test_rigid_body_tracker_reports_invalid_reason_for_lost_pose() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)

    tracker.update(_pose(1_000_000, 0.0, valid=True))
    tracker.update(_pose(1_016_000, 0.0, valid=False))

    diagnostics = tracker.get_diagnostics()

    assert diagnostics["valid"] is False
    assert diagnostics["invalid_reason"] == "no_valid_candidate"
    assert diagnostics["current_lost_run_frames"] == 1
    assert diagnostics["current_valid_run_frames"] == 0


def test_peek_prediction_is_side_effect_free_and_uses_timestamp() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)
    tracker.update(_pose(1_000_000, 0.0, valid=True))
    tracker.update(_pose(1_100_000, 0.10, valid=True))

    before = tracker.get_diagnostics()
    prediction = tracker.peek_prediction(1_200_000)
    after = tracker.get_diagnostics()

    assert prediction.valid is True
    assert prediction.timestamp == 1_200_000
    assert prediction.dt_s == 0.1
    assert np.allclose(prediction.position, [0.20, 0.0, 0.0])
    assert prediction.confidence > 0.0
    assert prediction.position_sigma_m > 0.0
    assert before["track_count"] == after["track_count"]
    assert before["total_frames"] == after["total_frames"]
    assert before["current_valid_run_frames"] == after["current_valid_run_frames"]


def test_rolling_confidence_drops_after_recent_lost_frames() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)
    for index in range(5):
        tracker.update(_pose(1_000_000 + index * 16_000, float(index) * 0.01, valid=True))
    confident = tracker.rolling_confidence

    for index in range(5, 10):
        tracker.update(_pose(1_000_000 + index * 16_000, 0.05, valid=False))

    assert tracker.rolling_confidence < confident
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["prediction"]["valid"] is True
    assert diagnostics["rolling_confidence"] == diagnostics["prediction"]["confidence"]


def test_tracker_mode_transitions_boot_continue_reacquire_lost() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)

    tracker.update(_pose(1_000_000, 0.00, valid=True))
    assert tracker.get_diagnostics()["mode"] == "boot"

    tracker.update(_pose(1_016_000, 0.01, valid=True))
    tracker.update(_pose(1_032_000, 0.02, valid=True))
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["mode"] == "continue"
    assert diagnostics["last_mode_transition"] == "boot->continue:boot_confirmed"

    tracker.update(_pose(1_048_000, 0.02, valid=False))
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["mode"] == "reacquire"
    assert diagnostics["last_mode_transition"] == "continue->reacquire:continue_measurement_rejected"

    tracker.update(_pose(1_064_000, 0.03, valid=False))
    tracker.update(_pose(1_080_000, 0.03, valid=False))
    tracker.update(_pose(1_096_000, 0.03, valid=False))
    tracker.update(_pose(1_112_000, 0.03, valid=False))
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["mode"] == "lost"
    assert diagnostics["last_mode_transition"] == "reacquire->lost:reacquire_timeout"

    tracker.update(_pose(1_128_000, 0.03, valid=True))
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["mode"] == "boot"
    assert diagnostics["last_mode_transition"] == "lost->boot:lost_shape_found"


def test_tracker_reacquire_requires_consistent_innovation_before_continue() -> None:
    tracker = RigidBodyTracker(WAIST_PATTERN)
    tracker.update(_pose(1_000_000, 0.00, valid=True))
    tracker.update(_pose(1_100_000, 0.10, valid=True))
    tracker.update(_pose(1_200_000, 0.20, valid=True))
    assert tracker.get_diagnostics()["mode"] == "continue"

    tracker.update(_pose(1_216_000, 0.20, valid=False))
    assert tracker.get_diagnostics()["mode"] == "reacquire"

    tracker.update(_pose(1_232_000, 1.50, valid=True))
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["mode"] == "reacquire"
    assert diagnostics["last_mode_reason"] == "reacquire_candidate_large_innovation"
    assert diagnostics["last_position_innovation_m"] > 0.25

    tracker.update(_pose(1_248_000, 2.15, valid=True))
    diagnostics = tracker.get_diagnostics()
    assert diagnostics["mode"] == "continue"
    assert diagnostics["last_mode_transition"] == "reacquire->continue:reacquire_confirmed"
