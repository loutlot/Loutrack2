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


def test_tracking_pipeline_reports_stage_and_logger_diagnostics(tmp_path: Path) -> None:
    pipeline = TrackingPipeline(enable_logging=True, log_dir=str(tmp_path))
    pipeline._running = True
    pipeline._calibration_loaded = True
    assert pipeline.logger is not None
    log_path = Path(pipeline.logger.start_recording(session_name="pipeline_diagnostics"))
    pipeline._diagnostics_event_interval_s = 0.0

    class _Geometry:
        def process_paired_frames(self, _paired_frames):
            return {"points_3d": [np.array([1.0, 2.0, 3.0])], "reprojection_errors": [0.25]}

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
    assert callbacks
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
