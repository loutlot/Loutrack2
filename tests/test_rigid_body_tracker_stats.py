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
