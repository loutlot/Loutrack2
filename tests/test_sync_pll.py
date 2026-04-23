import json
import socket
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pi.service.capture_runtime import (  # noqa: E402
    DEFAULT_TARGET_FPS,
    SYNC_LOCK_MIN_COUNT,
    SYNC_MAX_STEP_US,
    SYNC_MIN_FRAME_DURATION_US,
    SYNC_PLL_GAIN,
    Picamera2Backend,
    _SyncBeaconClient,
    _SyncBeaconServer,
    _SyncState,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _FakePicamera2:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def set_controls(self, controls: dict[str, object]) -> None:
        self.calls.append(dict(controls))


class _FailingPicamera2(_FakePicamera2):
    def set_controls(self, controls: dict[str, object]) -> None:
        self.calls.append(dict(controls))
        raise RuntimeError("FrameDurationLimits rejected")


# ---------------------------------------------------------------------------
# _SyncState
# ---------------------------------------------------------------------------

def test_sync_state_initial_values() -> None:
    state = _SyncState()
    assert state.get_frame_ts() == 0
    assert state.phase_error_us is None
    assert state.correction_count == 0
    assert state.locked is False


def test_sync_state_update_frame_ts() -> None:
    state = _SyncState()
    state.update_frame_ts(1_000_000)
    assert state.get_frame_ts() == 1_000_000


def test_sync_state_lock_after_min_corrections() -> None:
    state = _SyncState()
    for i in range(SYNC_LOCK_MIN_COUNT - 1):
        state.record_correction(float(i))
        assert state.locked is False
    state.record_correction(0.0)
    assert state.locked is True


def test_sync_state_diagnostics_keys() -> None:
    state = _SyncState()
    diag = state.get_diagnostics()
    assert "phase_error_us" in diag
    assert "correction_count" in diag
    assert "sync_locked" in diag


def test_sync_state_thread_safety() -> None:
    state = _SyncState()
    errors: list[Exception] = []

    def writer() -> None:
        try:
            for i in range(500):
                state.update_frame_ts(i)
        except Exception as exc:
            errors.append(exc)

    def corrector() -> None:
        try:
            for i in range(500):
                state.record_correction(float(i))
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=writer), threading.Thread(target=corrector)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors


# ---------------------------------------------------------------------------
# _SyncBeaconClient._apply_correction
# ---------------------------------------------------------------------------

def _make_client(
    apply_fn=None,
    nominal_period_us=int(round(1_000_000 / DEFAULT_TARGET_FPS)),
) -> tuple[_SyncBeaconClient, _SyncState]:
    state = _SyncState()
    applied: list[int] = []

    def _apply(d: int) -> None:
        applied.append(d)
        if apply_fn:
            apply_fn(d)

    client = _SyncBeaconClient(
        beacon_port=_free_udp_port(),
        sync_state=state,
        apply_sync_duration_us=_apply,
        get_nominal_period_us=lambda: nominal_period_us,
        log_fn=lambda msg: None,
    )
    client._applied = applied  # type: ignore[attr-defined]
    return client, state


def test_pll_skips_before_first_frame() -> None:
    client, state = _make_client()
    client._apply_correction(1_000_500, int(round(1_000_000 / DEFAULT_TARGET_FPS)))
    assert client._applied == []  # type: ignore[attr-defined]


def test_pll_phase_error_positive() -> None:
    client, state = _make_client()
    state.update_frame_ts(1_000_000)
    client._apply_correction(1_000_500, int(round(1_000_000 / DEFAULT_TARGET_FPS)))
    # phase_error = 500, correction = 500*0.15 = 75, per-frame = 75/10, adjusted = 8475+8 = 8483
    assert client._applied == [8483]  # type: ignore[attr-defined]
    assert state.phase_error_us == pytest.approx(500.0)


def test_pll_phase_error_negative() -> None:
    nominal_period = int(round(1_000_000 / DEFAULT_TARGET_FPS))
    # client is 500µs AHEAD of server → server_ts appears period-500 ahead mod period
    client, state = _make_client()
    state.update_frame_ts(1_000_000)
    # server_ts = 999_500: server is 500µs behind client
    # raw_err = (999_500 - 1_000_000) % period = period-500
    # period-500 > period/2 → phase_error = -500
    # correction = -500 * 0.15 = -75, per-frame = -75/10, adjusted = 8475 - 8 = 8467
    client._apply_correction(999_500, nominal_period)
    assert client._applied == [8467]  # type: ignore[attr-defined]
    assert state.phase_error_us == pytest.approx(-500.0)


def test_pll_correction_clamp() -> None:
    client, state = _make_client()
    state.update_frame_ts(1_000_000)
    # phase_error = 2000, correction = 2000*0.15 = 300 → clamped to 200, per-frame = 20
    client._apply_correction(1_002_000, int(round(1_000_000 / DEFAULT_TARGET_FPS)))
    assert client._applied == [8495]  # type: ignore[attr-defined]


def test_pll_correction_clamp_negative() -> None:
    client, state = _make_client()
    state.update_frame_ts(1_000_000)
    # client ahead by 2000: server_ts = 998_000 → error maps to -2000 → clamped to -200
    client._apply_correction(1_000_000 - 2000, int(round(1_000_000 / DEFAULT_TARGET_FPS)))
    assert client._applied == [8455]  # type: ignore[attr-defined]


def test_pll_uses_nominal_period_over_beacon() -> None:
    # nominal_period_us=10000 takes precedence over beacon_period=8333
    client, state = _make_client(nominal_period_us=10000)
    state.update_frame_ts(1_000_000)
    client._apply_correction(1_000_500, 8333)
    # correction = 500*0.15=75, per-frame = 75/10, adjusted = 10000+8 = 10008
    assert client._applied == [10008]  # type: ignore[attr-defined]


def test_pll_never_requests_faster_than_120fps_sensor_limit() -> None:
    client, state = _make_client(nominal_period_us=SYNC_MIN_FRAME_DURATION_US + 1)
    state.update_frame_ts(1_000_000)
    client._apply_correction(998_000, SYNC_MIN_FRAME_DURATION_US + 1)
    assert client._applied == [SYNC_MIN_FRAME_DURATION_US]  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# _SyncBeaconServer
# ---------------------------------------------------------------------------

def test_beacon_server_broadcasts_every_n_frames() -> None:
    port = _free_udp_port()
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", port))
    recv_sock.settimeout(1.0)

    server = _SyncBeaconServer(
        camera_id="pi-cam-01",
        broadcast_host="127.0.0.1",
        beacon_port=port,
        interval_n=3,
        log_fn=lambda msg: None,
    )
    server.start()

    try:
        for _ in range(6):
            server.maybe_broadcast(1_000_000, 8333)

        received = []
        for _ in range(2):
            try:
                data, _ = recv_sock.recvfrom(1024)
                received.append(json.loads(data.decode()))
            except TimeoutError:
                break

        assert len(received) == 2
        assert received[0]["type"] == "sync_beacon"
        assert received[0]["camera_id"] == "pi-cam-01"
        assert received[0]["seq"] == 0
        assert received[1]["seq"] == 1
    finally:
        server.stop()
        recv_sock.close()


def test_beacon_server_no_extra_broadcast() -> None:
    port = _free_udp_port()
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", port))
    recv_sock.settimeout(0.1)

    server = _SyncBeaconServer(
        camera_id="pi-cam-01",
        broadcast_host="127.0.0.1",
        beacon_port=port,
        interval_n=5,
        log_fn=lambda msg: None,
    )
    server.start()

    try:
        for _ in range(4):
            server.maybe_broadcast(1_000_000, 8333)

        count = 0
        while True:
            try:
                recv_sock.recvfrom(1024)
                count += 1
            except (TimeoutError, OSError):
                break
        assert count == 0
    finally:
        server.stop()
        recv_sock.close()


# ---------------------------------------------------------------------------
# Picamera2Backend sync_duration_us
# ---------------------------------------------------------------------------

def test_picamera2_sync_duration_overrides_nominal() -> None:
    backend = Picamera2Backend()
    fake = _FakePicamera2()
    backend._picam2 = fake  # type: ignore[assignment]
    backend._running = True

    backend.set_fps(DEFAULT_TARGET_FPS)
    nominal_calls = [c for c in fake.calls if "FrameDurationLimits" in c]
    expected_us = int(round(1_000_000 / DEFAULT_TARGET_FPS))
    assert nominal_calls[-1]["FrameDurationLimits"] == (expected_us, expected_us)

    backend.set_sync_duration_us(8500)
    sync_calls = [c for c in fake.calls if "FrameDurationLimits" in c]
    assert sync_calls[-1]["FrameDurationLimits"] == (8500, 8500)


def test_picamera2_sync_duration_none_reverts_to_nominal() -> None:
    backend = Picamera2Backend()
    fake = _FakePicamera2()
    backend._picam2 = fake  # type: ignore[assignment]
    backend._running = True

    backend.set_fps(DEFAULT_TARGET_FPS)
    backend.set_sync_duration_us(8500)
    backend.set_sync_duration_us(None)
    fdl_calls = [c["FrameDurationLimits"] for c in fake.calls if "FrameDurationLimits" in c]
    expected_us = int(round(1_000_000 / DEFAULT_TARGET_FPS))
    assert fdl_calls[-1] == (expected_us, expected_us)


def test_picamera2_set_fps_clears_sync_override() -> None:
    backend = Picamera2Backend()
    fake = _FakePicamera2()
    backend._picam2 = fake  # type: ignore[assignment]
    backend._running = True

    backend.set_fps(DEFAULT_TARGET_FPS)
    backend.set_sync_duration_us(8500)
    assert backend._sync_duration_us == 8500

    backend.set_fps(60)
    assert backend._sync_duration_us is None
    fdl_calls = [c["FrameDurationLimits"] for c in fake.calls if "FrameDurationLimits" in c]
    assert fdl_calls[-1] == (16667, 16667)


def test_picamera2_control_diagnostics_records_frame_duration_success() -> None:
    backend = Picamera2Backend()
    fake = _FakePicamera2()
    backend._picam2 = fake  # type: ignore[assignment]
    backend._running = True

    backend.set_fps(DEFAULT_TARGET_FPS)
    backend.set_sync_duration_us(8500)

    diag = backend.get_control_diagnostics()
    assert diag["backend_control_supported"] is True
    assert diag["requested_fps"] == DEFAULT_TARGET_FPS
    assert diag["requested_sync_duration_us"] == 8500
    assert diag["frame_duration_limits_requested"] == [8500, 8500]
    assert diag["frame_duration_limits_applied"] == [8500, 8500]
    assert diag["control_last_ok"] is True
    assert diag["control_last_error"] is None
    assert diag["control_apply_count"] == 2
    assert diag["control_apply_error_count"] == 0


def test_picamera2_control_diagnostics_records_frame_duration_failure() -> None:
    backend = Picamera2Backend()
    fake = _FailingPicamera2()
    backend._picam2 = fake  # type: ignore[assignment]
    backend._running = True

    backend.set_fps(DEFAULT_TARGET_FPS)

    expected_us = int(round(1_000_000 / DEFAULT_TARGET_FPS))
    diag = backend.get_control_diagnostics()
    assert diag["frame_duration_limits_requested"] == [expected_us, expected_us]
    assert diag["frame_duration_limits_applied"] is None
    assert diag["control_last_ok"] is False
    assert diag["control_last_error"] == "FrameDurationLimits rejected"
    assert diag["control_apply_count"] == 1
    assert diag["control_apply_error_count"] == 1


# ---------------------------------------------------------------------------
# parse_args sync-role resolution
# ---------------------------------------------------------------------------

def test_parse_args_sync_role_default_off(monkeypatch) -> None:
    from pi.service.capture_runtime import parse_args

    monkeypatch.setattr("sys.argv", ["capture_runtime.py", "--camera-id", "test-cam"])
    config = parse_args()
    assert config.sync_role == "off"


def test_parse_args_sync_role_server(monkeypatch) -> None:
    from pi.service.capture_runtime import parse_args

    monkeypatch.setattr("sys.argv", ["capture_runtime.py", "--camera-id", "test-cam", "--sync-role", "server"])
    config = parse_args()
    assert config.sync_role == "server"


def test_parse_args_sync_role_client(monkeypatch) -> None:
    from pi.service.capture_runtime import parse_args

    monkeypatch.setattr("sys.argv", ["capture_runtime.py", "--camera-id", "test-cam", "--sync-role", "client"])
    config = parse_args()
    assert config.sync_role == "client"


def test_parse_args_sync_role_auto_master(monkeypatch) -> None:
    from pi.service.capture_runtime import parse_args

    monkeypatch.setattr("sys.argv", ["capture_runtime.py", "--camera-id", "test-cam", "--sync-role", "auto"])
    monkeypatch.setattr("pi.service.capture_runtime._read_linuxptp_setting", lambda path, allowed: "master")
    config = parse_args()
    assert config.sync_role == "server"


def test_parse_args_sync_role_auto_slave(monkeypatch) -> None:
    from pi.service.capture_runtime import parse_args

    monkeypatch.setattr("sys.argv", ["capture_runtime.py", "--camera-id", "test-cam", "--sync-role", "auto"])
    monkeypatch.setattr("pi.service.capture_runtime._read_linuxptp_setting", lambda path, allowed: "slave")
    config = parse_args()
    assert config.sync_role == "client"


def test_parse_args_sync_role_auto_unknown(monkeypatch) -> None:
    from pi.service.capture_runtime import parse_args

    monkeypatch.setattr("sys.argv", ["capture_runtime.py", "--camera-id", "test-cam", "--sync-role", "auto"])
    monkeypatch.setattr("pi.service.capture_runtime._read_linuxptp_setting", lambda path, allowed: "unknown")
    config = parse_args()
    assert config.sync_role == "off"


# ---------------------------------------------------------------------------
# Integration: _SyncBeaconClient receives UDP and applies correction
# ---------------------------------------------------------------------------

def test_client_receives_beacon_and_applies_correction() -> None:
    port = _free_udp_port()
    nominal_period = int(round(1_000_000 / DEFAULT_TARGET_FPS))
    state = _SyncState()
    state.update_frame_ts(1_000_000)
    applied: list[int] = []

    client = _SyncBeaconClient(
        beacon_port=port,
        sync_state=state,
        apply_sync_duration_us=applied.append,
        get_nominal_period_us=lambda: nominal_period,
        log_fn=lambda msg: None,
    )
    client.start()

    try:
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        beacon = {"type": "sync_beacon", "camera_id": "pi-cam-01", "frame_ts_us": 1_000_500, "period_us": 8333, "seq": 0}
        send_sock.sendto(json.dumps(beacon).encode(), ("127.0.0.1", port))

        deadline = time.monotonic() + 2.0
        while not applied and time.monotonic() < deadline:
            time.sleep(0.02)

        assert applied, "No correction applied within 2s"
        assert applied[0] == 8483
    finally:
        client.stop()
        send_sock.close()


def test_client_ignores_non_beacon_udp() -> None:
    port = _free_udp_port()
    state = _SyncState()
    state.update_frame_ts(1_000_000)
    applied: list[int] = []

    client = _SyncBeaconClient(
        beacon_port=port,
        sync_state=state,
        apply_sync_duration_us=applied.append,
        get_nominal_period_us=lambda: 8333,
        log_fn=lambda msg: None,
    )
    client.start()

    try:
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        send_sock.sendto(b"not json", ("127.0.0.1", port))
        send_sock.sendto(json.dumps({"type": "other"}).encode(), ("127.0.0.1", port))
        time.sleep(0.2)
        assert not applied
    finally:
        client.stop()
        send_sock.close()
