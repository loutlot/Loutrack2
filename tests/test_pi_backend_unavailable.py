import importlib
import importlib.util
import os
import socket
import subprocess
import sys
import time
from collections.abc import Generator
from pathlib import Path
from typing import Protocol, TypedDict, cast

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


class PiCaptureServerInfo(TypedDict):
    ip: str
    tcp_port: int
    udp_port: int
    camera_id: str
    proc: subprocess.Popen[str]


control = cast(ControlModule, cast(object, importlib.import_module("host.control")))


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


@pytest.fixture()
def pi_capture_server_picamera2_requested() -> Generator[PiCaptureServerInfo, None, None]:
    if importlib.util.find_spec("picamera2") is not None:
        pytest.skip("picamera2 is available; backend_unavailable behavior not applicable")

    python_exe = (ROOT / ".venv/bin/python").resolve()
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
        str(ROOT / "src/pi/capture.py"),
        "--backend",
        "picamera2",
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
        cwd=str(ROOT),
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


def test_picamera2_backend_unavailable_is_graceful(
    pi_capture_server_picamera2_requested: PiCaptureServerInfo,
) -> None:
    ip = pi_capture_server_picamera2_requested["ip"]
    tcp_port = pi_capture_server_picamera2_requested["tcp_port"]
    camera_id = pi_capture_server_picamera2_requested["camera_id"]

    resp = control.ping(ip, tcp_port, camera_id=camera_id, timeout=1.0)
    assert resp.get("ack") is True

    resp = control.start(ip, tcp_port, camera_id=camera_id, mode="capture", timeout=1.0)
    assert resp.get("ack") is False
    assert resp.get("error_code") == 6

    resp = control.ping(ip, tcp_port, camera_id=camera_id, timeout=1.0)
    assert resp.get("ack") is True
