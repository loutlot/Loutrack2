from __future__ import annotations

import os
import sys
import time
import json
import socket

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.metrics import MetricsCollector
from host.receiver import Frame, FrameBuffer, FramePairer, FrameProcessor, UDPReceiver


def _frame(camera_id: str, index: int, timestamp: int | None = None, received_at: float | None = None) -> Frame:
    return Frame(
        camera_id=camera_id,
        timestamp=timestamp if timestamp is not None else 1_000_000 + index * 1_000,
        frame_index=index,
        blobs=[{"x": 10.0 + index, "y": 20.0, "area": 4.0}],
        received_at=received_at if received_at is not None else time.time(),
        host_received_at_us=int((received_at if received_at is not None else time.time()) * 1_000_000),
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


def test_frame_processor_emits_completed_pairs_without_frame_batch_delay() -> None:
    emitted = []
    processor = FrameProcessor(udp_port=0)
    processor.set_paired_callback(emitted.append)

    now = time.time()
    processor._on_frame_received(_frame("pi-cam-01", 1, timestamp=1_000_000, received_at=now))
    assert emitted == []

    processor._on_frame_received(_frame("pi-cam-02", 1, timestamp=1_000_050, received_at=now + 0.001))

    assert len(emitted) == 1
    assert {frame.frame_index for frame in emitted[0].frames.values()} == {1}


def test_frame_pairer_can_disable_frame_index_fallback_for_tracking() -> None:
    buffer = FrameBuffer(buffer_size=10)
    pairer = FramePairer(timestamp_tolerance_us=5000, min_cameras=2, frame_index_fallback=False)

    buffer.add_frame(_frame("pi-cam-01", 1, timestamp=1_000_000))
    buffer.add_frame(_frame("pi-cam-02", 1, timestamp=1_100_000))

    assert pairer.pair_frames(buffer) == []
    assert pairer.get_stats()["timestamp_unmatched_frames"] >= 1
    assert pairer.get_stats()["frame_index_fallback_pairs"] == 0


def test_frame_buffer_preserves_timestamp_order_for_out_of_order_inserts() -> None:
    buffer = FrameBuffer(buffer_size=10)

    buffer.add_frame(_frame("pi-cam-01", 2, timestamp=1_002_000))
    buffer.add_frame(_frame("pi-cam-01", 0, timestamp=1_000_000))
    buffer.add_frame(_frame("pi-cam-01", 1, timestamp=1_001_000))

    frames = buffer.get_all_frames("pi-cam-01")

    assert [frame.frame_index for frame in reversed(frames)] == [0, 1, 2]


def test_frame_pairer_drops_stale_reference_frames_and_emits_later_pairs() -> None:
    buffer = FrameBuffer(buffer_size=10)
    pairer = FramePairer(timestamp_tolerance_us=5000, min_cameras=2, frame_index_fallback=False)

    buffer.add_frame(_frame("pi-cam-01", 1, timestamp=1_000_000))
    buffer.add_frame(_frame("pi-cam-02", 9, timestamp=1_020_000))

    assert pairer.pair_frames(buffer) == []
    assert pairer.get_stats()["stale_frames_dropped"] == 1

    buffer.add_frame(_frame("pi-cam-01", 2, timestamp=1_030_000))
    buffer.add_frame(_frame("pi-cam-02", 2, timestamp=1_030_100))

    pairs = pairer.pair_frames(buffer)

    assert len(pairs) == 1
    assert {frame.frame_index for frame in pairs[0].frames.values()} == {2}


def test_frame_pairer_drops_stale_reference_frames_once_other_cameras_advance() -> None:
    buffer = FrameBuffer(buffer_size=10)
    pairer = FramePairer(timestamp_tolerance_us=5000, min_cameras=2, frame_index_fallback=False)

    now = time.time()
    buffer.add_frame(_frame("pi-cam-01", 1, timestamp=1_000_000, received_at=now))
    buffer.add_frame(_frame("pi-cam-02", 10, timestamp=1_020_000, received_at=now + 0.01))

    assert pairer.pair_frames(buffer) == []
    assert buffer.get_oldest_frame("pi-cam-01") is None
    assert pairer.get_stats()["stale_frames_dropped"] == 1


def test_udp_receiver_accepts_payload_without_frame_index_and_records_diagnostics() -> None:
    callback_frames = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    _host, port = probe.getsockname()
    probe.close()

    receiver = UDPReceiver(host="127.0.0.1", port=port)
    receiver.set_frame_callback(callback_frames.append)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.start()
        payload = {
            "camera_id": "pi-cam-01",
            "timestamp": 1_700_000_000_000_000,
            "timestamp_source": "sensor_metadata",
            "sensor_timestamp_ns": 123456789,
            "capture_to_process_ms": 3.5,
            "capture_to_send_ms": 4.25,
            "blobs": [{"x": 10.0, "y": 20.0, "area": 5.0}],
        }
        sender.sendto(json.dumps(payload).encode("utf-8"), ("127.0.0.1", port))

        deadline = time.time() + 1.0
        while time.time() < deadline and not callback_frames:
            time.sleep(0.01)
    finally:
        sender.close()
        receiver.stop()

    assert len(callback_frames) == 1
    frame = callback_frames[0]
    assert frame.frame_index is None
    assert frame.timestamp_source == "sensor_metadata"
    assert frame.sensor_timestamp_ns == 123456789
    assert frame.capture_to_process_ms == 3.5
    assert frame.capture_to_send_ms == 4.25
    assert frame.host_received_at_us > 0
    logged = frame.to_dict()
    assert "frame_index" not in logged
    assert logged["host_received_at_us"] == frame.host_received_at_us


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
            timestamp_source="sensor_metadata",
            capture_to_process_ms=1.0,
            capture_to_send_ms=2.0,
        )

    summary = metrics.get_summary()

    assert summary["cameras"]["pi-cam-01"]["frame_count"] == 3
    assert 45.0 <= summary["cameras"]["pi-cam-01"]["fps"] <= 55.0
    assert 4.0 <= summary["cameras"]["pi-cam-01"]["latency_ms"] <= 6.0
    assert summary["cameras"]["pi-cam-01"]["timestamp_sources"]["sensor_metadata"] == 3
    assert summary["cameras"]["pi-cam-01"]["capture_to_send_ms"]["last"] == 2.0


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
