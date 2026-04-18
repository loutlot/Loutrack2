from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.metrics import MetricsCollector
from host.receiver import Frame, FrameBuffer, FramePairer


def _frame(camera_id: str, index: int, timestamp: int | None = None, received_at: float | None = None) -> Frame:
    return Frame(
        camera_id=camera_id,
        timestamp=timestamp if timestamp is not None else 1_000_000 + index * 1_000,
        frame_index=index,
        blobs=[{"x": 10.0 + index, "y": 20.0, "area": 4.0}],
        received_at=received_at if received_at is not None else time.time(),
    )


def test_frame_pairer_does_not_emit_buffered_pairs_twice() -> None:
    buffer = FrameBuffer(buffer_size=10)
    pairer = FramePairer(timestamp_tolerance_us=5000, min_cameras=2)

    for index in range(3):
        buffer.add_frame(_frame("pi-cam-01", index))
        buffer.add_frame(_frame("pi-cam-02", index, timestamp=1_000_000 + index * 1_000 + 50))

    first_pairs = pairer.pair_frames(buffer)
    second_pairs = pairer.pair_frames(buffer)

    assert len(first_pairs) == 3
    assert [pair.frames["pi-cam-01"].frame_index for pair in first_pairs] == [0, 1, 2]
    assert second_pairs == []

    buffer.add_frame(_frame("pi-cam-01", 3))
    buffer.add_frame(_frame("pi-cam-02", 3, timestamp=1_003_050))

    next_pairs = pairer.pair_frames(buffer)

    assert len(next_pairs) == 1
    assert {frame.frame_index for frame in next_pairs[0].frames.values()} == {3}


def test_metrics_can_use_receiver_timestamp_for_fps_and_latency() -> None:
    metrics = MetricsCollector(history_size=10)
    start = 2_000.0

    for index in range(3):
        received_at = start + index * 0.02
        metrics.record_frame(
            camera_id="pi-cam-01",
            timestamp=int((received_at - 0.005) * 1_000_000),
            blob_count=1,
            frame_index=index,
            received_at=received_at,
        )

    summary = metrics.get_summary()

    assert summary["cameras"]["pi-cam-01"]["frame_count"] == 3
    assert 45.0 <= summary["cameras"]["pi-cam-01"]["fps"] <= 55.0
    assert 4.0 <= summary["cameras"]["pi-cam-01"]["latency_ms"] <= 6.0


def test_metrics_missing_frames_ignores_out_of_order_delivery() -> None:
    metrics = MetricsCollector(history_size=10)

    for frame_index in (10, 9, 11):
        metrics.record_frame(
            camera_id="pi-cam-01",
            timestamp=1_000_000 + frame_index,
            blob_count=1,
            frame_index=frame_index,
            received_at=2_000.0 + frame_index * 0.001,
        )

    summary = metrics.get_summary()

    assert summary["cameras"]["pi-cam-01"]["frame_count"] == 3
    assert summary["cameras"]["pi-cam-01"]["last_frame_index"] == 11
    assert summary["cameras"]["pi-cam-01"]["missing_frames"] == 0
