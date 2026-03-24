from __future__ import annotations

import importlib.util
import queue
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.pi.service.capture_runtime import (  # noqa: E402
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
import src.pi.service.capture_runtime as capture_mod  # noqa: E402


def test_pi_intrinsics_loader_registers_module_for_dataclass_annotations() -> None:
    module_path = ROOT / "src" / "pi" / "service" / "intrinsics_capture.py"
    spec = importlib.util.spec_from_file_location("pi_intrinsics_capture_test", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["pi_intrinsics_capture_test"] = module
    spec.loader.exec_module(module)

    loaded = module._load_intrinsics_calibrate_module()

    assert loaded.__name__ == "_pi_intrinsics_calibrate"
    assert "CalibrateConfig" in loaded.__dict__


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
    assert result["mjpeg_server_enabled"] is True
    assert result["mjpeg_render_enabled"] is False
    assert result["preview_overlays"] == {
        "blob": True,
        "mask": True,
        "text": True,
        "charuco": True,
    }
    assert result["charuco_config"]["dictionary"] == "DICT_6X6_250"
    assert "intrinsics_start" in result["supported_commands"]
    assert "intrinsics_status" in result["supported_commands"]
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
    monkeypatch.setattr("src.pi.service.capture_runtime._clock_realtime_us", lambda: next(times))

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


def test_set_preview_updates_ping_and_pipeline(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    calls: list[bool] = []

    class _FakePipeline:
        def set_mjpeg_render_enabled(self, enabled: bool) -> None:
            calls.append(bool(enabled))

    fake_pipeline = _FakePipeline()
    monkeypatch.setattr(server, "_ensure_pipeline_started", lambda: fake_pipeline)

    response = server._dispatch_set_preview(
        "req-1",
        "pi-cam-01",
        {
            "render_enabled": True,
            "overlays": {"charuco": False},
            "charuco": {"squares_x": 7},
        },
    )
    assert response["ack"] is True
    assert calls == [True]

    ping = server._handle_ping("req-2", "pi-cam-01")
    result = ping["result"]
    assert result["mjpeg_render_enabled"] is True
    assert result["preview_overlays"]["charuco"] is False
    assert result["charuco_config"]["squares_x"] == 7

    server._pipeline = fake_pipeline  # type: ignore[assignment]
    response = server._dispatch_set_preview("req-3", "pi-cam-01", {"render_enabled": False})
    assert response["ack"] is True
    assert calls[-1] is False


def test_set_preview_rejects_invalid_params(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    monkeypatch.setattr(server._charuco_renderer, "is_dictionary_supported", lambda _value: False)

    invalid_dictionary = server._dispatch_set_preview(
        "req-1",
        "pi-cam-01",
        {"charuco": {"dictionary": "INVALID_DICT"}},
    )
    assert invalid_dictionary["ack"] is False
    assert invalid_dictionary["error_code"] == 2

    invalid_shape = server._dispatch_set_preview(
        "req-2",
        "pi-cam-01",
        {"charuco": {"squares_x": 1}},
    )
    assert invalid_shape["ack"] is False
    assert invalid_shape["error_code"] == 2

    invalid_type = server._dispatch_set_preview(
        "req-3",
        "pi-cam-01",
        {"overlays": {"mask": "on"}},
    )
    assert invalid_type["ack"] is False
    assert invalid_type["error_code"] == 2


def test_resolve_debug_preview_enabled_requires_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    assert resolve_debug_preview_enabled(True) is False


def test_resolve_debug_preview_enabled_accepts_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert resolve_debug_preview_enabled(True) is True


def test_get_default_backend_prefers_dummy_off_pi(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("src.pi.service.capture_runtime.running_on_raspberry_pi", lambda: False)
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


def test_intrinsics_start_dispatch_sets_pipeline_session(monkeypatch: pytest.MonkeyPatch) -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    captured: dict[str, object] = {}

    class _FakePipeline:
        def set_intrinsics_session(self, session):
            captured["session"] = session

    def _fake_start(config) -> None:
        captured["config"] = config

    def _fake_status_payload() -> dict[str, object]:
        return {
            "phase": "capturing",
            "camera_id": "pi-cam-01",
            "frames_captured": 0,
            "frames_needed": 25,
            "frames_target": 50,
            "frames_rejected_cooldown": 0,
            "frames_rejected_spatial": 0,
            "frames_rejected_detection": 0,
            "grid_coverage": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            "last_error": None,
            "calibration_result": None,
        }

    monkeypatch.setattr(server, "_ensure_pipeline_started", lambda: _FakePipeline())
    monkeypatch.setattr(server._intrinsics_session, "start", _fake_start)
    monkeypatch.setattr(server._intrinsics_session, "status_payload", _fake_status_payload)

    response = server._dispatch_intrinsics_start(
        "req-1",
        "pi-cam-01",
        {
            "camera_id": "pi-cam-01",
            "square_length_mm": 30.0,
            "marker_length_mm": 22.5,
            "squares_x": 6,
            "squares_y": 8,
            "min_frames": 25,
            "cooldown_s": 1.5,
        },
    )

    assert response["ack"] is True
    assert "config" in captured
    assert "session" in captured


def test_processing_worker_uses_raw_frame_for_detection_and_intrinsics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    frame_queue: queue.Queue[object] = queue.Queue(maxsize=2)
    calls: dict[str, int] = {}

    class _SpyIntrinsics:
        def consume_frame(self, frame: np.ndarray) -> None:
            calls["intrinsics_frame_id"] = id(frame)

    def _fake_detect_blobs(
        frame: np.ndarray,
        *,
        threshold: int,
        mask: np.ndarray | None,
        min_diameter_px: float | None,
        max_diameter_px: float | None,
        circularity_min: float,
    ):
        _ = (threshold, mask, min_diameter_px, max_diameter_px, circularity_min)
        calls["detect_frame_id"] = id(frame)
        return [], {
            "threshold": 200,
            "min_diameter_px": None,
            "max_diameter_px": None,
            "circularity_min": 0.0,
            "raw_contour_count": 0,
            "accepted_blob_count": 0,
            "rejected_by_diameter": 0,
            "rejected_by_circularity": 0,
            "last_blob_count": 0,
        }

    monkeypatch.setattr(capture_mod, "detect_blobs", _fake_detect_blobs)

    worker = capture_mod._ProcessingWorker(
        camera_id="pi-cam-01",
        udp_host="127.0.0.1",
        udp_port=5000,
        frame_queue=frame_queue,
        preview_worker=None,
        log_fn=lambda _msg: None,
    )
    worker.set_intrinsics_session(_SpyIntrinsics())
    worker.start()
    frame = np.zeros((24, 32, 3), dtype=np.uint8)
    try:
        frame_queue.put(
            capture_mod._FramePacket(
                captured_frame=capture_mod.CapturedFrame(
                    image=frame,
                    timestamp_us=1,
                    timestamp_source="capture_dequeue",
                ),
                captured_monotonic_ns=time.monotonic_ns(),
            )
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if "detect_frame_id" in calls and "intrinsics_frame_id" in calls:
                break
            time.sleep(0.01)
    finally:
        worker.stop()

    assert calls.get("detect_frame_id") == id(frame)
    assert calls.get("intrinsics_frame_id") == id(frame)


def test_charuco_overlay_draw_requires_minimum_six_corners(monkeypatch: pytest.MonkeyPatch) -> None:
    renderer = capture_mod._CharucoOverlayRenderer()
    draw_calls = {"count": 0}

    class _FakeAruco:
        def detectMarkers(self, gray, dictionary, parameters=None):
            _ = (gray, dictionary, parameters)
            corners = [np.zeros((4, 1, 2), dtype=np.float32) for _ in range(4)]
            ids = np.arange(4, dtype=np.int32).reshape(-1, 1)
            return corners, ids, None

        def interpolateCornersCharuco(self, marker_corners, marker_ids, gray, board):
            _ = (marker_corners, marker_ids, gray, board)
            corners = np.zeros((5, 1, 2), dtype=np.float32)
            ids = np.arange(5, dtype=np.int32).reshape(-1, 1)
            return corners, ids, None

        def drawDetectedCornersCharuco(self, image, corners, ids):
            _ = (image, corners, ids)
            draw_calls["count"] += 1

    fake_aruco = _FakeAruco()
    monkeypatch.setattr(renderer, "_resources", lambda _cfg: (fake_aruco, object(), object()))
    monkeypatch.setattr(capture_mod, "_create_aruco_detector_parameters", lambda _aruco: object())
    canvas = np.zeros((64, 64, 3), dtype=np.uint8)

    renderer.draw(canvas, capture_mod.PreviewCharucoConfig())
    assert draw_calls["count"] == 0

    class _FakeArucoSix(_FakeAruco):
        def interpolateCornersCharuco(self, marker_corners, marker_ids, gray, board):
            _ = (marker_corners, marker_ids, gray, board)
            corners = np.zeros((6, 1, 2), dtype=np.float32)
            ids = np.arange(6, dtype=np.int32).reshape(-1, 1)
            return corners, ids, None

    fake_aruco_six = _FakeArucoSix()
    monkeypatch.setattr(renderer, "_resources", lambda _cfg: (fake_aruco_six, object(), object()))
    renderer.draw(canvas, capture_mod.PreviewCharucoConfig())
    assert draw_calls["count"] == 1
