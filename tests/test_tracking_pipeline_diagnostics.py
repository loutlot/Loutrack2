from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.pipeline import TrackingPipeline
from host.receiver import Frame, PairedFrames
from host.rigid import RigidBodyPose
from host.wand_session import FIXED_PAIR_WINDOW_US


def _frame(camera_id: str, timestamp: int, received_at: float) -> Frame:
    return Frame(
        camera_id=camera_id,
        timestamp=timestamp,
        frame_index=None,
        blobs=[{"x": 10.0, "y": 20.0, "area": 4.0}],
        received_at=received_at,
        host_received_at_us=int(received_at * 1_000_000),
        timestamp_source="sensor_metadata",
        capture_to_process_ms=1.0,
        capture_to_send_ms=2.0,
    )


def _triangulation_quality(accepted_points: int = 1) -> dict[str, object]:
    return {
        "accepted_points": accepted_points,
        "contributing_rays": {
            "per_point": [2] * accepted_points,
            "summary": {
                "count": accepted_points,
                "mean": 2.0 if accepted_points else 0.0,
                "median": 2.0 if accepted_points else 0.0,
                "p90": 2.0 if accepted_points else 0.0,
                "p95": 2.0 if accepted_points else 0.0,
                "min": 2.0 if accepted_points else 0.0,
                "max": 2.0 if accepted_points else 0.0,
            },
        },
        "reprojection_error_px_summary": {
            "count": accepted_points,
            "mean": 0.25 if accepted_points else 0.0,
            "median": 0.25 if accepted_points else 0.0,
            "p90": 0.25 if accepted_points else 0.0,
            "p95": 0.25 if accepted_points else 0.0,
            "min": 0.25 if accepted_points else 0.0,
            "max": 0.25 if accepted_points else 0.0,
        },
        "epipolar_error_px_summary": {
            "count": accepted_points,
            "mean": 0.1 if accepted_points else 0.0,
            "median": 0.1 if accepted_points else 0.0,
            "p90": 0.1 if accepted_points else 0.0,
            "p95": 0.1 if accepted_points else 0.0,
            "min": 0.1 if accepted_points else 0.0,
            "max": 0.1 if accepted_points else 0.0,
        },
        "triangulation_angle_deg_summary": {
            "count": accepted_points,
            "mean": 3.0 if accepted_points else 0.0,
            "median": 3.0 if accepted_points else 0.0,
            "p90": 3.0 if accepted_points else 0.0,
            "p95": 3.0 if accepted_points else 0.0,
            "min": 3.0 if accepted_points else 0.0,
            "max": 3.0 if accepted_points else 0.0,
        },
        "assignment_diagnostics": {"assignment_matches": accepted_points},
    }


def test_tracking_pipeline_reports_stage_and_logger_diagnostics(tmp_path: Path) -> None:
    pipeline = TrackingPipeline(enable_logging=True, log_dir=str(tmp_path))
    pipeline._running = True
    pipeline._calibration_loaded = True
    assert pipeline.logger is not None
    log_path = Path(pipeline.logger.start_recording(session_name="pipeline_diagnostics"))
    pipeline._diagnostics_event_interval_s = 0.0

    class _Geometry:
        def process_paired_frames(self, _paired_frames, *, min_inlier_views=2, object_gating=None):
            _ = (min_inlier_views, object_gating)
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "observations_by_camera": {
                    "pi-cam-01": [
                        {
                            "camera_id": "pi-cam-01",
                            "blob_index": 0,
                            "raw_uv": [10.0, 20.0],
                            "undistorted_uv": [10.0, 20.0],
                            "area": 4.0,
                        }
                    ]
                },
                "triangulated_points": [
                    {
                        "point": [1.0, 2.0, 3.0],
                        "camera_ids": ["pi-cam-01", "pi-cam-02"],
                        "blob_indices": [0, 0],
                        "contributing_rays": 2,
                        "observations": [],
                        "reprojection_errors_px": [0.25, 0.25],
                        "epipolar_errors_px": [0.1],
                        "triangulation_angles_deg": [3.0],
                        "source": "generic",
                        "rigid_name": None,
                        "marker_idx": None,
                        "is_virtual": False,
                    }
                ],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {"assignment_matches": 1},
                "triangulation_quality": _triangulation_quality(),
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _Rigid:
        def process_points(self, _points, timestamp):
            return {
                "waist": RigidBodyPose(
                    timestamp=timestamp,
                    position=np.array([1.0, 2.0, 3.0]),
                    rotation=np.eye(3),
                    quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    rms_error=0.1,
                    observed_markers=1,
                    valid=True,
                )
            }

        def get_tracking_status(self):
            return {"ok": True}

    pipeline.geometry = _Geometry()  # type: ignore[assignment]
    pipeline.rigid_estimator = _Rigid()  # type: ignore[assignment]
    callbacks = []
    pipeline.set_pose_callback(callbacks.append)

    now = time.time()
    pair = PairedFrames(
        timestamp=1_000_000,
        frames={
            "pi-cam-01": _frame("pi-cam-01", 1_000_000, now),
            "pi-cam-02": _frame("pi-cam-02", 1_000_100, now + 0.001),
        },
        timestamp_range_us=100,
    )

    pipeline._on_paired_frames(pair)
    status = pipeline.get_status()
    metadata = pipeline.logger.stop_recording()
    pipeline._running = False

    stage_ms = status["diagnostics"]["pipeline_stage_ms"]
    assert stage_ms["triangulation_ms"]["max"] >= 0.0
    assert stage_ms["rigid_ms"]["max"] >= 0.0
    assert stage_ms["metrics_update_ms"]["max"] >= 0.0
    assert stage_ms["log_enqueue_ms"]["max"] >= 0.0
    assert stage_ms["pose_callback_ms"]["max"] >= 0.0
    assert stage_ms["pipeline_pair_ms"]["max"] >= 0.0
    assert status["diagnostics"]["logger"]["recording"] is True
    assert status["diagnostics"]["tracking"] == {"ok": True}
    assert callbacks
    assert status["triangulation_quality"]["accepted_points"] == 1
    snapshot = pipeline.get_latest_triangulation_snapshot()
    assert snapshot["observations_by_camera"]["pi-cam-01"][0]["blob_index"] == 0
    assert snapshot["triangulated_points"][0]["point"] == [1.0, 2.0, 3.0]
    assert metadata["total_frames"] == 2

    events = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert any(
        entry.get("_type") == "event" and entry.get("event_type") == "tracking_diagnostics"
        for entry in events
    )
    diagnostics_events = [
        entry
        for entry in events
        if entry.get("_type") == "event" and entry.get("event_type") == "tracking_diagnostics"
    ]
    assert diagnostics_events[-1]["data"]["tracking"] == {"ok": True}


def test_tracking_pipeline_logs_reacquire_guard_events(tmp_path: Path) -> None:
    pipeline = TrackingPipeline(
        enable_logging=True,
        log_dir=str(tmp_path),
        reacquire_guard_event_logging=True,
    )
    pipeline._running = True
    pipeline._calibration_loaded = True
    assert pipeline.logger is not None
    log_path = Path(pipeline.logger.start_recording(session_name="guard_events"))

    class _Geometry:
        camera_params = {}

        def process_paired_frames(self, _paired_frames, *, min_inlier_views=2, object_gating=None):
            _ = (min_inlier_views, object_gating)
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {"assignment_matches": 1},
                "triangulation_quality": _triangulation_quality(),
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _Rigid:
        def process_points(self, _points, timestamp):
            return {
                "waist": RigidBodyPose(
                    timestamp=timestamp,
                    position=np.array([1.0, 2.0, 3.0]),
                    rotation=np.eye(3),
                    quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    valid=True,
                )
            }

        def get_tracking_status(self):
            return {
                "waist": {
                    "valid": True,
                    "mode": "reacquire",
                    "reacquire_guard": {
                        "evaluated": True,
                        "would_reject": True,
                        "passed": False,
                        "reason": "mean_reprojection_error_too_high",
                        "score": {"mean_error_px": 9.0},
                        "position_innovation_m": 0.01,
                        "rotation_innovation_deg": 2.0,
                        "enforced": False,
                        "rejected_count": 0,
                    },
                }
            }

    pipeline.geometry = _Geometry()  # type: ignore[assignment]
    pipeline.rigid_estimator = _Rigid()  # type: ignore[assignment]

    now = time.time()
    pair = PairedFrames(
        timestamp=1_000_000,
        frames={
            "pi-cam-01": _frame("pi-cam-01", 1_000_000, now),
            "pi-cam-02": _frame("pi-cam-02", 1_000_100, now + 0.001),
        },
        timestamp_range_us=100,
    )
    pipeline._on_paired_frames(pair)
    metadata = pipeline.logger.stop_recording()
    pipeline._running = False

    assert metadata["total_frames"] == 2
    guard_events = pipeline.get_reacquire_guard_events()
    assert len(guard_events) == 1
    assert guard_events[0]["would_reject"] is True

    events = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert any(
        entry.get("_type") == "event" and entry.get("event_type") == "reacquire_guard"
        for entry in events
    )


def test_tracking_pipeline_uses_half_frame_timestamp_pairing_window() -> None:
    pipeline = TrackingPipeline(enable_logging=False)

    assert pipeline.frame_processor.pairer.timestamp_tolerance_us == FIXED_PAIR_WINDOW_US
    assert pipeline.frame_processor.pairer.frame_index_fallback is False


def test_tracking_pipeline_keeps_fixed_pair_window_and_omits_sync_status() -> None:
    pipeline = TrackingPipeline(enable_logging=False)
    pipeline._running = True
    pipeline._calibration_loaded = True

    min_inlier_view_calls: list[int] = []

    class _Geometry:
        def process_paired_frames(self, _paired_frames, *, min_inlier_views=2, object_gating=None):
            _ = object_gating
            min_inlier_view_calls.append(int(min_inlier_views))
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {
                    "assignment_matches": 1,
                    "dropped_views_for_inlier_fit": 0,
                },
                "triangulation_quality": _triangulation_quality(),
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _Rigid:
        def process_points(self, _points, _timestamp):
            return {}

        def get_tracking_status(self):
            return {"ok": True}

    pipeline.geometry = _Geometry()  # type: ignore[assignment]
    pipeline.rigid_estimator = _Rigid()  # type: ignore[assignment]

    now = time.time()
    spreads = [200, 1500, 2200, 9000]
    for index, spread in enumerate(spreads):
        pair = PairedFrames(
            timestamp=1_000_000 + index * 10_000,
            frames={
                "pi-cam-01": _frame("pi-cam-01", 1_000_000 + index * 10_000, now + index * 0.01),
                "pi-cam-02": _frame(
                    "pi-cam-02",
                    1_000_000 + index * 10_000 + spread,
                    now + index * 0.01 + 0.001,
                ),
                "pi-cam-03": _frame(
                    "pi-cam-03",
                    1_000_000 + index * 10_000 + min(spread + 50, 2500),
                    now + index * 0.01 + 0.002,
                ),
            },
            timestamp_range_us=spread,
        )
        pipeline._on_paired_frames(pair)

    status = pipeline.get_status()
    snapshot = pipeline.get_latest_triangulation_snapshot()

    assert pipeline.frame_processor.pairer.timestamp_tolerance_us == FIXED_PAIR_WINDOW_US
    assert min_inlier_view_calls == [2, 2, 2, 2]
    assert "sync" not in status
    assert status["triangulation_quality"]["reprojection_error_px_summary"]["count"] == 1
    assert "sync_precision_mode" not in snapshot
    assert "epipolar_error_px_summary" in snapshot
    assert "triangulation_angle_deg_summary" in snapshot
