from __future__ import annotations

import sys
import threading
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.pi.capture import (
    ClockSyncSnapshot,
    ControlServer,
    ControlServerConfig,
    DebugPreview,
    DummyBackend,
    DummyBackendConfig,
    STATE_MASK_INIT,
    STATE_READY,
    detect_blobs,
    get_default_backend,
    resolve_debug_preview_enabled,
)
from src.pi import capture as capture_mod


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


def test_control_server_ping_includes_blob_diagnostics() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    response = server._handle_ping("req-1", "pi-cam-01")

    assert response["ack"] is True
    result = response["result"]
    assert isinstance(result, dict)
    assert result["debug_preview_enabled"] is False
    assert result["debug_preview_active"] is False
    assert result["blob_diagnostics"]["threshold"] == 200
    assert result["clock_sync"]["status"] in {"locked", "degraded", "unknown"}
    assert "offset_us" in result["clock_sync"]
    assert "source" in result["clock_sync"]
    assert result["clock_sync"]["role"] in {"master", "slave", "unknown"}
    assert result["clock_sync"]["timestamping_mode"] in {"software", "hardware", "unknown"}
    assert result["timestamping"]["active_source"] == "capture_dequeue"
    assert result["timestamping"]["sensor_timestamp_available"] is False


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
    first = server._handle_mask_start("req-1", "pi-cam-01", {"frames": 2, "threshold": 200})
    second = server._handle_mask_start("req-2", "pi-cam-01", {"frames": 2, "threshold": 200})
    assert first["ack"] is True
    assert second["ack"] is True


def test_mask_start_timeout_resets_state() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))

    def _raise_timeout(*args, **kwargs):
        raise TimeoutError("mask_init_timed_out")

    server._build_static_mask = _raise_timeout  # type: ignore[method-assign]
    response = server._handle_mask_start("req-1", "pi-cam-01", {"frames": 2, "threshold": 200})

    assert response["ack"] is False
    assert "mask_start_timeout" in response["error_message"]
    ping = server._handle_ping("req-2", "pi-cam-01")
    assert ping["result"]["state"] == "IDLE"


def test_mask_start_reuses_preview_backend_when_available(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    preview_backend = DummyBackend(DummyBackendConfig(width=32, height=24, num_dots=0))

    monkeypatch.setattr(server, "_take_preview_backend_for_mask", lambda: preview_backend)

    def _unexpected_make_backend() -> DummyBackend:
        raise AssertionError("mask_start should reuse preview backend")

    monkeypatch.setattr(server, "_make_backend", _unexpected_make_backend)
    response = server._handle_mask_start("req-1", "pi-cam-01", {"frames": 2, "threshold": 200})

    assert response["ack"] is True


def test_start_wand_metric_capture_reuses_preview_backend_and_disables_emitter_preview(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=True))
    server._state = STATE_READY
    server._static_mask = np.ones((8, 8), dtype=bool)
    preview_backend = DummyBackend(DummyBackendConfig(width=32, height=24, num_dots=0))
    captured: dict[str, object] = {}

    monkeypatch.setattr(server, "_take_preview_backend_for_mask", lambda: preview_backend)
    monkeypatch.setattr(
        server,
        "_stop_preview_loop",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("start should not stop preview loop")),
    )

    class FakeEmitter:
        def __init__(
            self,
            *,
            camera_id: str,
            udp_host: str,
            udp_port: int,
            target_fps: float,
            backend: DummyBackend,
            threshold: int,
            min_diameter_px: float | None,
            max_diameter_px: float | None,
            circularity_min: float,
            time_us_fn=None,
            max_frames=None,
            mask=None,
            debug_preview=None,
            capture_mode="capture",
        ) -> None:
            captured["camera_id"] = camera_id
            captured["backend"] = backend
            captured["debug_preview"] = debug_preview
            captured["capture_mode"] = capture_mode
            captured["started"] = False

        def set_mask(self, mask: np.ndarray | None) -> None:
            captured["mask"] = mask

        def start(self) -> None:
            captured["started"] = True

        def stop(self) -> None:
            return

    monkeypatch.setattr("src.pi.capture.UDPFrameEmitter", FakeEmitter)

    response = server._handle_start("req-1", "pi-cam-01", "wand_metric_capture", {})

    assert response["ack"] is True
    assert captured["backend"] is preview_backend
    assert captured["debug_preview"] is None
    assert captured["capture_mode"] == "wand_metric_capture"
    assert captured["mask"] is server._static_mask
    assert captured["started"] is True


def test_mask_start_does_not_touch_debug_preview_during_mask_init(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=True))
    preview = server._debug_preview
    assert preview is not None
    preview_backend = DummyBackend(DummyBackendConfig(width=32, height=24, num_dots=0))

    monkeypatch.setattr(server, "_take_preview_backend_for_mask", lambda: preview_backend)
    monkeypatch.setattr(server, "_start_preview_loop", lambda: None)
    monkeypatch.setattr(
        preview,
        "show",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("mask_start should not render preview")),
    )

    response = server._handle_mask_start("req-1", "pi-cam-01", {"frames": 2, "threshold": 200})

    assert response["ack"] is True


def test_resolve_debug_preview_enabled_requires_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    assert resolve_debug_preview_enabled(True) is False


def test_resolve_debug_preview_enabled_accepts_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert resolve_debug_preview_enabled(True) is True


def test_get_default_backend_prefers_dummy_off_pi(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("src.pi.capture.running_on_raspberry_pi", lambda: False)
    assert get_default_backend() == "dummy"


def test_start_preview_loop_reuses_existing_debug_preview(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=True))
    original_preview = server._debug_preview
    server._running = True
    monkeypatch.setattr(server, "_preview_loop", lambda stop_event: None)

    server._start_preview_loop()
    if server._preview_thread is not None:
        server._preview_thread.join(timeout=1.0)

    assert server._debug_preview is original_preview


def test_start_preview_loop_resumes_existing_preview_thread() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=True))
    server._running = True

    class _AliveThread:
        def is_alive(self) -> bool:
            return True

    server._preview_thread = _AliveThread()  # type: ignore[assignment]
    server._preview_backend = None
    server._preview_resume_event = threading.Event()

    server._start_preview_loop()

    assert server._preview_resume_event.is_set()


def test_start_preview_loop_does_not_resume_during_mask_init() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=True))
    server._running = True
    server._state = STATE_MASK_INIT

    class _AliveThread:
        def is_alive(self) -> bool:
            return True

    server._preview_thread = _AliveThread()  # type: ignore[assignment]
    server._preview_backend = None
    server._preview_resume_event = threading.Event()

    server._start_preview_loop()

    assert server._preview_resume_event.is_set() is False


def test_preview_handoff_keeps_backend_running() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    server._running = True
    stop_event = threading.Event()
    handoff_ready_event = threading.Event()
    resume_event = threading.Event()

    class TrackingBackend(DummyBackend):
        def __init__(self) -> None:
            super().__init__(DummyBackendConfig(width=32, height=24, num_dots=0))
            self.stop_calls = 0

        def stop(self) -> None:
            self.stop_calls += 1

    backend = TrackingBackend()
    server._preview_handoff_requested = True
    server._preview_handoff_ready_event = handoff_ready_event
    server._preview_resume_event = resume_event
    server._make_backend = lambda: backend  # type: ignore[method-assign]

    thread = threading.Thread(target=server._preview_loop, args=(stop_event,), daemon=True)
    server._preview_thread = thread
    server._preview_stop_event = stop_event
    thread.start()

    assert handoff_ready_event.wait(timeout=1.0) is True
    assert backend.stop_calls == 0

    stop_event.set()
    thread.join(timeout=1.0)


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
