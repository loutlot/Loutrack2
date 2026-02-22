import os
import sys
import socket
import subprocess
import threading
import time
import importlib
from pathlib import Path
from typing import Callable, Protocol, TypedDict, cast
from collections.abc import Generator

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


class ControlModule(Protocol):
    def ping(
        self,
        ip: str,
        port: int,
        camera_id: str,
        request_id: str | None = None,
        timeout: float = 5.0,
    ) -> dict[str, object]:
        ...

    def start(
        self,
        ip: str,
        port: int,
        camera_id: str,
        mode: str,
        request_id: str | None = None,
        timeout: float = 5.0,
    ) -> dict[str, object]:
        ...

    def stop(
        self,
        ip: str,
        port: int,
        camera_id: str,
        request_id: str | None = None,
        timeout: float = 5.0,
    ) -> dict[str, object]:
        ...


class UDPReceiverClass(Protocol):
    def __init__(self, host: str = "0.0.0.0", port: int = 5000, buffer_size: int = 65536) -> None:
        ...

    def set_frame_callback(self, callback: Callable[[object], None]) -> None:
        ...

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...


class ReceiverModule(Protocol):
    UDPReceiver: type[UDPReceiverClass]


class PiCaptureServerInfo(TypedDict):
    ip: str
    tcp_port: int
    udp_port: int
    camera_id: str
    proc: subprocess.Popen[str]


control = cast(ControlModule, cast(object, importlib.import_module("host.control")))
receiver_mod = cast(ReceiverModule, cast(object, importlib.import_module("host.receiver")))
UDPReceiver = receiver_mod.UDPReceiver


def _get_free_tcp_port(host: str = "127.0.0.1") -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        addr = cast(tuple[str, int], sock.getsockname())
        return int(addr[1])


def _get_free_udp_port(host: str = "127.0.0.1") -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, 0))
        addr = cast(tuple[str, int], sock.getsockname())
        return int(addr[1])


@pytest.fixture(scope="module")
def pi_capture_server() -> Generator[PiCaptureServerInfo, None, None]:
    root = ROOT
    python_exe = (root / ".venv/bin/python").resolve()
    if not python_exe.exists():
        pytest.skip(f"missing venv python: {python_exe}")

    tcp_port = _get_free_tcp_port()
    udp_port = _get_free_udp_port()
    camera_id = "pi-cam-01"
    ip = "127.0.0.1"

    env = dict(os.environ)
    env["PYTHONUNBUFFERED"] = "1"

    cmd = [
        str(python_exe),
        str(root / "src/pi/capture.py"),
        "--camera-id",
        camera_id,
        "--tcp-host",
        ip,
        "--tcp-port",
        str(tcp_port),
        "--udp-dest",
        f"{ip}:{udp_port}",
    ]

    proc = subprocess.Popen(
        cmd,
        cwd=str(root),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    try:
        deadline = time.monotonic() + 2.0
        last_exc: Exception | None = None
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                out, err = proc.communicate(timeout=1.0)
                msg = "\n".join(
                    [
                        "src/pi/capture.py exited early",
                        f"cmd={cmd!r}",
                        f"returncode={proc.returncode}",
                        "stdout=",
                        out,
                        "stderr=",
                        err,
                    ]
                )
                raise AssertionError(msg)

            try:
                resp = control.ping(ip, tcp_port, camera_id=camera_id, timeout=0.2)
                if resp.get("ack") is True:
                    break
            except Exception as exc:  # noqa: BLE001
                last_exc = exc

            time.sleep(0.05)
        else:
            raise AssertionError(f"src/pi/capture.py did not become ready: {last_exc!r}")

        yield {
            "ip": ip,
            "tcp_port": tcp_port,
            "udp_port": udp_port,
            "camera_id": camera_id,
            "proc": proc,
        }
    finally:
        try:
            proc.terminate()
        except Exception:  # noqa: BLE001
            pass

        try:
            _out, _err = proc.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            try:
                proc.kill()
            except Exception:  # noqa: BLE001
                pass
            _out, _err = proc.communicate(timeout=1.0)


def test_pi_dummy_control_e2e(pi_capture_server: PiCaptureServerInfo) -> None:
    ip = pi_capture_server["ip"]
    tcp_port = pi_capture_server["tcp_port"]
    udp_port = pi_capture_server["udp_port"]
    camera_id = pi_capture_server["camera_id"]

    frames_lock = threading.Lock()
    frames: list[object] = []

    def on_frame(frame: object) -> None:
        with frames_lock:
            frames.append(frame)

    def frame_count() -> int:
        with frames_lock:
            return len(frames)

    receiver = UDPReceiver(host="127.0.0.1", port=udp_port)
    receiver.set_frame_callback(on_frame)
    receiver.start()
    try:
        resp = control.ping(ip, tcp_port, camera_id=camera_id, timeout=1.0)
        assert resp.get("ack") is True

        resp = control.start(ip, tcp_port, camera_id=camera_id, mode="capture", timeout=1.0)
        assert resp.get("ack") is True

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and frame_count() < 10:
            time.sleep(0.02)
        assert frame_count() >= 10

        resp = control.stop(ip, tcp_port, camera_id=camera_id, timeout=1.0)
        assert resp.get("ack") is True

        count0 = frame_count()
        time.sleep(0.3)
        assert frame_count() == count0
    finally:
        receiver.stop()


def test_pi_control_wrong_camera_id_returns_error(pi_capture_server: PiCaptureServerInfo) -> None:
    ip = pi_capture_server["ip"]
    tcp_port = pi_capture_server["tcp_port"]

    resp = control.ping(ip, tcp_port, camera_id="pi-cam-wrong", timeout=1.0)
    assert resp.get("ack") is False
    assert resp.get("error_code") == 2
