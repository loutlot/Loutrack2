from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.pi.capture import (  # noqa: E402
    ClockSyncSnapshot,
    ControlServer,
    ControlServerConfig,
    DebugPreview,
    DummyBackend,
    DummyBackendConfig,
    STATE_READY,
    detect_blobs,
    get_default_backend,
    resolve_debug_preview_enabled,
)
from src.pi import capture as capture_mod  # noqa: E402


def _expected_centers(cfg: DummyBackendConfig, frame_index: int) -> list[tuple[int, int]]:
    radius = max(1, int(cfg.dot_radius))
    x_low = radius
    y_low = radius
    x_high = max(x_low + 1, cfg.width - radius)
    y_high = max(y_low + 1, cfg.height - radius)

    rng = np.random.default_rng(cfg.seed + frame_index)
    dot_count = max(0, int(cfg.num_dots))
    min_center_distance_sq = float((2 * radius + 1) ** 2)
    centers: list[tuple[int, int]] = []
    max_attempts = max(1, dot_count * 50)
    attempts = 0
    while len(centers) < dot_count and attempts < max_attempts:
        attempts += 1
        x = int(rng.integers(x_low, x_high))
        y = int(rng.integers(y_low, y_high))
        if any((x - cx) ** 2 + (y - cy) ** 2 < min_center_distance_sq for cx, cy in centers):
            continue
        centers.append((x, y))

    centers.sort(key=lambda pt: (pt[1], pt[0]))
    return centers


def test_detect_blobs_finds_three_dots_with_expected_centroids() -> None:
    config = DummyBackendConfig(width=160, height=120, num_dots=3, seed=42, dot_radius=2)

    backend_a = DummyBackend(config)
    backend_b = DummyBackend(config)
    frame = backend_a.next_frame()
    frame_again = backend_b.next_frame()
    assert np.array_equal(frame, frame_again)

    blobs, diagnostics = detect_blobs(frame, threshold=200)
    assert len(blobs) == 3
    assert diagnostics["accepted_blob_count"] == 3

    expected = _expected_centers(config, frame_index=0)
    for blob, (expected_x, expected_y) in zip(blobs, expected):
        assert abs(blob["x"] - float(expected_x)) <= 1.0
        assert abs(blob["y"] - float(expected_y)) <= 1.0
        assert blob["area"] > 0.0


def test_detect_blobs_threshold_255_returns_zero() -> None:
    backend = DummyBackend(DummyBackendConfig(width=120, height=90, num_dots=3, seed=123, dot_radius=2))
    frame = backend.next_frame()
    blobs, diagnostics = detect_blobs(frame, threshold=255)
    assert blobs == []
    assert diagnostics["accepted_blob_count"] == 0


def test_control_server_ping_includes_blob_diagnostics_and_runtime() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    response = server._handle_ping("req-1", "pi-cam-01")

    assert response["ack"] is True
    result = response["result"]
    assert isinstance(result, dict)
    assert result["debug_preview_enabled"] is False
    assert result["debug_preview_active"] is False
    assert result["blob_diagnostics"]["threshold"] == 200
    assert result["clock_sync"]["status"] in {"locked", "degraded", "unknown"}
    assert result["clock_sync"]["role"] in {"master", "slave", "unknown"}
    assert result["clock_sync"]["timestamping_mode"] in {"software", "hardware", "unknown"}
    assert result["timestamping"]["active_source"] == "capture_dequeue"
    assert result["timestamping"]["sensor_timestamp_available"] is False
    assert result["runtime"]["capture_fps"] == 0.0
    assert result["runtime"]["processing_queue_depth"] == 0


def test_control_server_ping_caches_clock_sync_probe(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    calls = {"count": 0}

    def _probe() -> ClockSyncSnapshot:
        calls["count"] += 1
        return ClockSyncSnapshot(
            status="locked",
            offset_us=12.0,
            source="pmc",
            role="slave",
            timestamping_mode="software",
        )

    times = iter([1_000_000, 1_500_000, 62_000_000])
    monkeypatch.setattr(server, "_probe_clock_sync", _probe)
    monkeypatch.setattr("src.pi.capture._clock_realtime_us", lambda: next(times))

    first = server._handle_ping("req-1", "pi-cam-01")
    second = server._handle_ping("req-2", "pi-cam-01")
    third = server._handle_ping("req-3", "pi-cam-01")

    assert first["result"]["clock_sync"]["status"] == "locked"
    assert second["result"]["clock_sync"]["status"] == "locked"
    assert third["result"]["clock_sync"]["status"] == "locked"
    assert first["result"]["clock_sync"]["role"] == "slave"
    assert first["result"]["clock_sync"]["timestamping_mode"] == "software"
    assert calls["count"] == 2


def test_parse_pmc_time_status_normalizes_ns_to_us_for_slave() -> None:
    snapshot = capture_mod._parse_pmc_time_status(
        """
        master_offset              -250000
        gmPresent                  true
        """,
        role="slave",
        timestamping_mode="software",
    )

    assert snapshot.status == "locked"
    assert snapshot.offset_us == -250.0
    assert snapshot.role == "slave"
    assert snapshot.timestamping_mode == "software"


def test_parse_pmc_time_status_allows_software_master_gm_present_false() -> None:
    snapshot = capture_mod._parse_pmc_time_status(
        """
        master_offset              0
        gmPresent                  false
        """,
        role="master",
        timestamping_mode="software",
    )

    assert snapshot.status == "locked"
    assert snapshot.offset_us == 0.0


def test_mask_start_can_refresh_existing_mask() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    try:
        first = server._handle_mask_start("req-1", "pi-cam-01", {"frames": 2, "threshold": 200})
        second = server._handle_mask_start("req-2", "pi-cam-01", {"frames": 2, "threshold": 200})
        assert first["ack"] is True
        assert second["ack"] is True
    finally:
        server.shutdown()


def test_mask_start_timeout_resets_state(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))

    class _FakePipeline:
        def build_mask(self, **kwargs):
            _ = kwargs
            raise TimeoutError("mask_init_timed_out")

        def set_mask(self, mask):
            _ = mask

        def set_state_label(self, state):
            _ = state

    monkeypatch.setattr(server, "_ensure_pipeline_started", lambda: _FakePipeline())
    response = server._handle_mask_start("req-1", "pi-cam-01", {"frames": 2, "threshold": 200})

    assert response["ack"] is False
    assert "mask_start_timeout" in response["error_message"]
    ping = server._handle_ping("req-2", "pi-cam-01")
    assert ping["result"]["state"] == "IDLE"


def test_start_capture_requires_ready_mask_for_all_modes() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))

    for mode in ("capture", "pose_capture", "wand_metric_capture"):
        response = server._handle_start("req-1", "pi-cam-01", mode, {})
        assert response["ack"] is False
        assert response["error_message"] == "invalid_request: mask_required_for_mode"


def test_start_pose_capture_always_applies_static_mask_and_uses_pipeline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    server._state = STATE_READY
    server._static_mask = np.ones((8, 8), dtype=bool)
    captured: dict[str, object] = {}

    class _FakePipeline:
        def set_mask(self, mask):
            captured["mask"] = mask

        def set_state_label(self, state):
            captured["state"] = state

        def start_stream(self, mode):
            captured["mode"] = mode

        def stop_stream(self):
            captured["stopped"] = True

    monkeypatch.setattr(server, "_ensure_pipeline_started", lambda: _FakePipeline())

    response = server._handle_start("req-1", "pi-cam-01", "pose_capture", {})

    assert response["ack"] is True
    assert response["result"]["mask_active"] is True
    assert captured["mode"] == "pose_capture"
    assert captured["mask"] is server._static_mask
    assert captured["state"] == "RUNNING"


def test_handle_stop_returns_ready_and_keeps_mask(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    server._state = "RUNNING"
    server._static_mask = np.ones((4, 4), dtype=bool)
    captured: dict[str, object] = {}

    class _FakePipeline:
        def get_last_detection_stats(self):
            return {"threshold": 200, "accepted_blob_count": 1}

        def get_last_timestamping_status(self):
            return {"active_source": "sensor_metadata", "sensor_timestamp_available": True}

        def stop_stream(self):
            captured["stop_stream"] = True

        def set_state_label(self, state):
            captured["state"] = state

    server._pipeline = _FakePipeline()  # type: ignore[assignment]

    response = server._handle_stop("req-1", "pi-cam-01")

    assert response["ack"] is True
    assert server._state == STATE_READY
    assert captured["stop_stream"] is True
    assert captured["state"] == STATE_READY


def test_resolve_debug_preview_enabled_requires_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    assert resolve_debug_preview_enabled(True) is False


def test_resolve_debug_preview_enabled_accepts_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert resolve_debug_preview_enabled(True) is True


def test_get_default_backend_prefers_dummy_off_pi(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("src.pi.capture.running_on_raspberry_pi", lambda: False)
    assert get_default_backend() == "dummy"


def test_ping_reports_open_debug_preview_window_as_active() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=True))
    preview = server._debug_preview
    assert preview is not None
    preview._initialized = True

    response = server._handle_ping("req-1", "pi-cam-01")

    assert response["ack"] is True
    assert response["result"]["debug_preview_active"] is True


def test_debug_preview_close_window_preserves_enabled() -> None:
    preview = DebugPreview()
    preview._initialized = True
    preview.close_window()
    assert preview.enabled is True
