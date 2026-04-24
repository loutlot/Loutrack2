from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host import tracking_replay_harness as harness


@dataclass
class _Entry:
    received_at: str
    data: dict


class _FakeReplay:
    def __init__(self, _log_path: str) -> None:
        self.entries = [
            _Entry(
                received_at="2026-04-24T00:00:00",
                data={
                    "camera_id": "pi-cam-01",
                    "timestamp": 1000,
                    "host_received_at_us": 10_000,
                    "blobs": [{"x": 1.0, "y": 2.0, "area": 3.0}],
                },
            ),
            _Entry(
                received_at="2026-04-24T00:00:00.001000",
                data={
                    "camera_id": "pi-cam-02",
                    "timestamp": 1100,
                    "host_received_at_us": 11_000,
                    "blobs": [{"x": 4.0, "y": 5.0, "area": 6.0}],
                },
            ),
        ]

    def replay(self, *, realtime: bool = False):
        assert realtime is False
        yield from self.entries


class _FakeFrameProcessor:
    def __init__(self) -> None:
        self.callback = None
        self.frames = []

    def set_paired_callback(self, callback):
        self.callback = callback

    def _on_frame_received(self, frame):
        self.frames.append(frame)
        if len(self.frames) == 2 and self.callback is not None:
            self.callback({"paired": True})

    def _process_pairs(self):
        return None

    def get_stats(self):
        return {"pairer": {"pairs_emitted": 1}}


class _FakePipeline:
    def __init__(self, **_kwargs) -> None:
        self._calibration_loaded = True
        self._running = False
        self.frame_processor = _FakeFrameProcessor()
        self.frames_processed = 0
        self.poses_estimated = 0

    def _on_paired_frames(self, _paired_frames):
        self.frames_processed += 1
        self.poses_estimated += 1

    def get_status(self):
        return {
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
            "receiver": self.frame_processor.get_stats(),
            "tracking": {"waist": {"valid": True, "reacquire_count": 0}},
            "diagnostics": {
                "geometry": {"quality": {"accepted_points": 4}},
                "pipeline_stage_ms": {"rigid_ms": {"p95": 0.1}},
            },
        }


def test_replay_tracking_log_injects_frames_through_frame_processor(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)

    summary = harness.replay_tracking_log(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=tmp_path,
        patterns=["waist"],
    ).to_dict()

    assert summary["frame_count"] == 2
    assert summary["pair_count"] == 1
    assert summary["frames_processed"] == 1
    assert summary["poses_estimated"] == 1
    assert summary["tracking"]["waist"]["valid"] is True
