"""
Test script for verifying Phase 0 implementation.

Run with: python tests/test_phase0.py
"""

import sys
import os
import time
import json
import pytest

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from host.logger import FrameLogger
from host.replay import FrameReplay, validate_log_integrity
from host.metrics import MetricsCollector, MetricsExporter


@pytest.fixture()
def recorded_log_file(tmp_path) -> str:
    """Create a short JSONL log for replay tests."""
    logger = FrameLogger(log_dir=str(tmp_path))

    log_file = logger.start_recording(session_name="test_session")

    test_frames = [
        {
            "camera_id": "pi-cam-01",
            "timestamp": 1708594800000000 + i * 16667,  # ~60fps spacing
            "frame_index": i,
            "blobs": [
                {"x": 100.0 + i, "y": 200.0, "area": 50.0},
                {"x": 300.0, "y": 400.0, "area": 45.0},
            ],
        }
        for i in range(10)
    ]

    for frame in test_frames:
        logger.log_frame(frame)

    logger.log_event("calibration_start", {"type": "intrinsic"})
    logger.stop_recording()

    assert os.path.exists(log_file), "Log file should exist"
    return log_file


def test_logger_creates_log(recorded_log_file: str) -> None:
    assert os.path.exists(recorded_log_file)


def test_replay(recorded_log_file: str) -> None:
    """Test log replay functionality."""
    print("=== Testing FrameReplay ===")
    
    # Validate log integrity
    validation = validate_log_integrity(recorded_log_file)
    print(f"Validation result: {json.dumps(validation, indent=2)}")
    
    assert validation["valid"], "Log should be valid"
    
    # Load and replay
    replay = FrameReplay(recorded_log_file)
    
    print(f"Header: {replay.header}")
    print(f"Frame count: {replay.frame_count}")
    print(f"Cameras: {replay.get_cameras()}")
    print(f"Duration: {replay.get_duration_seconds():.3f} seconds")
    
    # Replay all frames at max speed
    frame_count = 0
    for entry in replay.replay(realtime=False):
        frame_count += 1
        assert entry.camera_id == "pi-cam-01"
        assert len(entry.blobs) == 2
    
    assert frame_count == 10, f"Expected 10 frames, got {frame_count}"
    
    # Replay with realtime timing (short test)
    frame_count = 0
    for entry in replay.replay(realtime=True, speed=10.0):  # 10x speed
        frame_count += 1
    
    assert frame_count == 10
    
    print("✓ Replay test passed\n")


def test_metrics(tmp_path) -> None:
    """Test metrics collection."""
    print("=== Testing MetricsCollector ===")
    
    metrics = MetricsCollector()
    
    # Simulate frame receipts from two cameras
    current_time = int(time.time() * 1_000_000)  # Current time in us
    
    for i in range(20):
        # Camera 1
        metrics.record_frame(
            camera_id="pi-cam-01",
            timestamp=current_time + i * 16667,
            blob_count=4,
            frame_index=i
        )
        
        # Camera 2 (slightly offset)
        metrics.record_frame(
            camera_id="pi-cam-02",
            timestamp=current_time + i * 16667 + 100,
            blob_count=3,
            frame_index=i
        )
        
        time.sleep(0.016)  # ~60fps
    
    # Simulate triangulation results
    metrics.record_triangulation(12, [0.5, 0.3, 0.8, 0.2, 0.4])
    metrics.record_triangulation(10, [0.4, 0.2, 0.6])
    
    # Get summary
    summary = metrics.get_summary()
    print(f"Metrics summary: {json.dumps(summary, indent=2)}")
    
    assert summary["global"]["camera_count"] == 2
    assert summary["cameras"]["pi-cam-01"]["frame_count"] == 20
    assert summary["triangulation"]["total_points"] == 22
    
    # Test Prometheus export
    prom_output = metrics.export_prometheus()
    print(f"\nPrometheus output:\n{prom_output}")
    
    # Export to file
    MetricsExporter.to_json(summary, str(tmp_path / "metrics_test.json"))
    
    print("✓ Metrics test passed\n")


def test_end_to_end(tmp_path) -> None:
    """Test complete flow: log -> replay -> verify same output."""
    print("=== Testing End-to-End Reproducibility ===")
    
    # Create test data
    test_frames = [
        {
            "camera_id": "pi-cam-01",
            "timestamp": 1708594800000000 + i * 16667,
            "frame_index": i,
            "blobs": [{"x": float(100 + i), "y": 200.0, "area": 50.0}]
        }
        for i in range(5)
    ]
    
    # Log frames
    logger = FrameLogger(log_dir=str(tmp_path))
    log_file = logger.start_recording(session_name="e2e_test")
    
    for frame in test_frames:
        logger.log_frame(frame)
    
    logger.stop_recording()
    
    # Replay and collect
    replay = FrameReplay(log_file)
    replayed_frames = list(replay.replay(realtime=False))
    
    # Verify same count
    assert len(replayed_frames) == len(test_frames), \
        f"Frame count mismatch: {len(replayed_frames)} vs {len(test_frames)}"
    
    # Verify same content
    for original, entry in zip(test_frames, replayed_frames):
        assert entry.data["camera_id"] == original["camera_id"]
        assert entry.data["timestamp"] == original["timestamp"]
        assert entry.data["frame_index"] == original["frame_index"]
        assert entry.blobs == original["blobs"]
    
    print("✓ End-to-end test passed\n")
