#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import json
import socket
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Protocol, cast

import cv2
import numpy as np


ERROR_INVALID_JSON = 1
ERROR_INVALID_REQUEST = 2
ERROR_UNKNOWN_CMD = 3
ERROR_NOT_RUNNING = 4
ERROR_ALREADY_RUNNING = 5
ERROR_BACKEND_UNAVAILABLE = 6
ERROR_INTERNAL = 7

STATE_IDLE = "IDLE"
STATE_RUNNING = "RUNNING"

MAX_LINE_BYTES = 65536
LINE_TIMEOUT_SECONDS = 2.0

SCHEMA_COMMANDS = {
    "start",
    "stop",
    "set_exposure",
    "set_gain",
    "set_fps",
    "mask_start",
    "mask_stop",
    "led_on",
    "led_off",
    "set_resolution",
    "ping",
}

MVP_SUPPORTED_COMMANDS = {
    "ping",
    "start",
    "stop",
    "set_exposure",
    "set_gain",
    "set_fps",
}


def parse_udp_dest(value: str) -> tuple[str, int]:
    raw = value.strip()
    if not raw:
        raise ValueError("udp_dest is empty")

    host: str
    port_str: str
    if raw.startswith("["):
        close = raw.find("]")
        if close < 0:
            raise ValueError("udp_dest: missing closing ']' for ipv6")
        host = raw[1:close]
        rest = raw[close + 1 :]
        if not rest.startswith(":"):
            raise ValueError("udp_dest: expected :port after ]")
        port_str = rest[1:]
    else:
        if ":" not in raw:
            raise ValueError("udp_dest: expected host:port")
        host, port_str = raw.rsplit(":", 1)

    host = host.strip()
    port_str = port_str.strip()
    if not host:
        raise ValueError("udp_dest: host is empty")
    if not port_str:
        raise ValueError("udp_dest: port is empty")

    try:
        port = int(port_str)
    except ValueError as exc:
        raise ValueError("udp_dest: port must be integer") from exc
    if port <= 0 or port > 65535:
        raise ValueError("udp_dest: port out of range")

    return host, port


class UDPFrameEmitter:
    def __init__(
        self,
        *,
        camera_id: str,
        udp_host: str,
        udp_port: int,
        target_fps: float,
        backend: "FrameBackend",
        threshold: int = 200,
        time_us_fn: Callable[[], int] | None = None,
        max_frames: int | None = None,
    ):
        self._camera_id: str = camera_id
        self._udp_host: str = udp_host
        self._udp_port: int = udp_port
        self._threshold: int = int(threshold)
        self._backend: FrameBackend = backend
        self._time_us_fn: Callable[[], int] = (
            time_us_fn if time_us_fn is not None else (lambda: time.time_ns() // 1000)
        )

        self._lock: threading.Lock = threading.Lock()
        self._target_fps: float = float(target_fps)
        self._max_frames: int | None = max_frames

        self._stop_event: threading.Event = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._frame_index: int = 0

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("udp emitter already running")

            self._stop_event.clear()
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self._udp_host in ("255.255.255.255", "<broadcast>"):
                self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        sock: socket.socket | None
        thread: threading.Thread | None
        with self._lock:
            sock = self._socket
            thread = self._thread
            self._socket = None
            self._thread = None

        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if thread is not None:
            thread.join(timeout=1.0)

    def join(self, timeout: float | None = None) -> None:
        with self._lock:
            thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout)

    def set_target_fps(self, fps: float) -> None:
        with self._lock:
            self._target_fps = float(fps)

    def _loop(self) -> None:
        sent = 0
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                sock = self._socket
                fps = self._target_fps
                max_frames = self._max_frames

            if sock is None:
                return

            frame = self._backend.next_frame()
            blobs = detect_blobs(frame, threshold=self._threshold)
            msg = {
                "camera_id": self._camera_id,
                "timestamp": int(self._time_us_fn()),
                "frame_index": int(self._frame_index & 0xFFFFFFFF),
                "blobs": blobs,
            }
            self._frame_index = (self._frame_index + 1) & 0xFFFFFFFF

            payload = json.dumps(msg, separators=(",", ":")).encode("utf-8")
            try:
                _ = sock.sendto(payload, (self._udp_host, self._udp_port))
            except OSError:
                return

            sent += 1
            if max_frames is not None and sent >= max_frames:
                return

            period = 1.0 / fps if fps > 0.0 else 0.0
            if period > 0.0:
                next_tick += period
                now = time.perf_counter()
                wait_s = next_tick - now
                if wait_s > 0:
                    _ = self._stop_event.wait(wait_s)
                else:
                    next_tick = now


@dataclass
class DummyBackendConfig:
    width: int = 640
    height: int = 480
    num_dots: int = 3
    seed: int = 0
    dot_radius: int = 3
    background_value: int = 0
    dot_value: int = 255


class BackendUnavailableError(RuntimeError):
    pass


class FrameBackend(Protocol):
    def start(self) -> None: ...

    def stop(self) -> None: ...

    def next_frame(self) -> np.ndarray: ...

    def set_exposure_us(self, value_us: int) -> None: ...

    def set_gain(self, value: float) -> None: ...

    def set_fps(self, fps: int) -> None: ...


class _Picamera2Api(Protocol):
    def create_video_configuration(self, *, main: dict[str, object]) -> object: ...

    def configure(self, config: object) -> None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...

    def capture_array(self) -> object: ...

    def set_controls(self, controls: dict[str, object]) -> object: ...


class DummyBackend:
    def __init__(self, config: DummyBackendConfig | None = None):
        self._config: DummyBackendConfig = config if config is not None else DummyBackendConfig()
        self._frame_index: int = 0
        self._exposure_us: int | None = None
        self._gain: float | None = None
        self._fps: int | None = None

    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def set_exposure_us(self, value_us: int) -> None:
        self._exposure_us = int(value_us)

    def set_gain(self, value: float) -> None:
        self._gain = float(value)

    def set_fps(self, fps: int) -> None:
        self._fps = int(fps)

    @property
    def frame_index(self) -> int:
        return self._frame_index

    def next_frame(self) -> np.ndarray:
        cfg = self._config
        frame = np.full(
            (cfg.height, cfg.width),
            fill_value=cfg.background_value,
            dtype=np.uint8,
        )

        radius = max(1, int(cfg.dot_radius))
        x_low = radius
        y_low = radius
        x_high = max(x_low + 1, cfg.width - radius)
        y_high = max(y_low + 1, cfg.height - radius)

        rng = np.random.default_rng(cfg.seed + self._frame_index)
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

        for x, y in centers:
            _ = cv2.circle(frame, (x, y), radius, int(cfg.dot_value), thickness=-1)

        self._frame_index += 1
        return frame


@dataclass
class Picamera2BackendConfig:
    width: int = 640
    height: int = 480
    format: str = "RGB888"


class Picamera2Backend:
    def __init__(self, config: Picamera2BackendConfig | None = None):
        self._config: Picamera2BackendConfig = (
            config if config is not None else Picamera2BackendConfig()
        )
        self._picam2: _Picamera2Api | None = None
        self._running: bool = False

        self._exposure_us: int | None = None
        self._gain: float | None = None
        self._fps: int | None = None

    def set_exposure_us(self, value_us: int) -> None:
        self._exposure_us = int(value_us)
        self._apply_controls_if_running()

    def set_gain(self, value: float) -> None:
        self._gain = float(value)
        self._apply_controls_if_running()

    def set_fps(self, fps: int) -> None:
        fps_value = int(fps)
        if fps_value <= 0:
            return
        self._fps = fps_value
        self._apply_controls_if_running()

    def start(self) -> None:
        if self._running:
            return

        try:
            picamera2_mod = importlib.import_module("picamera2")
            picamera2_obj = getattr(picamera2_mod, "Picamera2", None)
            if picamera2_obj is None:
                raise BackendUnavailableError("picamera2_missing_Picamera2")
        except Exception as exc:
            raise BackendUnavailableError(f"picamera2_import_failed: {exc}") from exc

        try:
            Picamera2_cls = cast(type[_Picamera2Api], picamera2_obj)
            picam2 = Picamera2_cls()
            size = (int(self._config.width), int(self._config.height))

            try:
                cfg: object = picam2.create_video_configuration(
                    main={"size": size, "format": str(self._config.format)}
                )
            except Exception:
                cfg = picam2.create_video_configuration(main={"size": size})

            picam2.configure(cfg)
            picam2.start()
        except Exception as exc:
            raise BackendUnavailableError(f"picamera2_start_failed: {exc}") from exc

        self._picam2 = picam2
        self._running = True
        self._apply_controls_if_running()

    def stop(self) -> None:
        picam2 = self._picam2
        self._picam2 = None
        self._running = False
        if picam2 is None:
            return

        try:
            picam2.stop()
        except Exception:
            pass
        try:
            picam2.close()
        except Exception:
            pass

    def next_frame(self) -> np.ndarray:
        picam2 = self._picam2
        if picam2 is None or not self._running:
            raise RuntimeError("picamera2 backend not running")

        frame = picam2.capture_array()
        if not isinstance(frame, np.ndarray):
            raise RuntimeError("picamera2 capture_array returned non-ndarray")
        return cast(np.ndarray, frame)

    def _apply_controls_if_running(self) -> None:
        if not self._running or self._picam2 is None:
            return
        picam2 = self._picam2

        controls: dict[str, object] = {}
        if self._exposure_us is not None:
            controls["ExposureTime"] = int(self._exposure_us)
        if self._gain is not None:
            controls["AnalogueGain"] = float(self._gain)
        if self._fps is not None and self._fps > 0:
            frame_us = int(round(1_000_000 / float(self._fps)))
            controls["FrameDurationLimits"] = (frame_us, frame_us)

        if not controls:
            return

        try:
            _ = picam2.set_controls(controls)
        except Exception:
            return


def detect_blobs(frame: np.ndarray, threshold: int) -> list[dict[str, float]]:
    if frame.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    threshold_value = max(0, min(255, int(threshold)))
    _retval, binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)
    contours_info = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

    blobs: list[dict[str, float]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area <= 0.0:
            continue
        moments = cv2.moments(contour)
        m00 = float(moments.get("m00", 0.0))
        if m00 <= 0.0:
            continue
        m10 = float(moments.get("m10", 0.0))
        m01 = float(moments.get("m01", 0.0))
        x = float(m10 / m00)
        y = float(m01 / m00)
        blobs.append({"x": x, "y": y, "area": area})

    blobs.sort(key=lambda blob: (blob["y"], blob["x"]))
    return blobs


@dataclass
class ControlServerConfig:
    camera_id: str = "pi-cam-01"
    tcp_host: str = "0.0.0.0"
    tcp_port: int = 8554
    udp_dest: str = "255.255.255.255:5000"
    udp_host: str = "255.255.255.255"
    udp_port: int = 5000
    backend: str = "dummy"
    target_fps: int = 60
    threshold: int = 200


class ControlServer:
    def __init__(self, config: ControlServerConfig):
        self._config: ControlServerConfig = config
        self._state_lock: threading.Lock = threading.Lock()
        self._state: str = STATE_IDLE
        self._server_socket: socket.socket | None = None
        self._running: bool = False

        self._backend: FrameBackend | None = None
        self._udp_emitter: UDPFrameEmitter | None = None

        self._desired_exposure_us: int | None = None
        self._desired_gain: float | None = None
        self._desired_fps: int | None = int(self._config.target_fps)

    def serve_forever(self) -> None:
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self._config.tcp_host, self._config.tcp_port))
        self._server_socket.listen()
        self._running = True

        try:
            while self._running:
                try:
                    accept_result = self._server_socket.accept()
                    conn = accept_result[0]
                except OSError:
                    break

                thread = threading.Thread(
                    target=self._handle_connection,
                    args=(conn,),
                    daemon=True,
                )
                thread.start()
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._running = False
        emitter = self._udp_emitter
        self._udp_emitter = None
        if emitter is not None:
            emitter.stop()
        backend = self._backend
        self._backend = None
        if backend is not None:
            backend.stop()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None

    def _handle_connection(self, conn: socket.socket) -> None:
        buffer = bytearray()
        conn.settimeout(LINE_TIMEOUT_SECONDS)

        try:
            while self._running:
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    break

                if not chunk:
                    break

                buffer.extend(chunk)

                if len(buffer) > MAX_LINE_BYTES and b"\n" not in buffer:
                    response = self._error_response(
                        request_id="",
                        request_camera_id=self._config.camera_id,
                        error_code=ERROR_INVALID_JSON,
                        error_message="invalid_json: line_too_long",
                    )
                    self._send_response(conn, response)
                    buffer.clear()
                    continue

                while True:
                    newline_idx = buffer.find(b"\n")
                    if newline_idx < 0:
                        break

                    raw_line = bytes(buffer[:newline_idx])
                    del buffer[: newline_idx + 1]

                    if len(raw_line) > MAX_LINE_BYTES:
                        response = self._error_response(
                            request_id="",
                            request_camera_id=self._config.camera_id,
                            error_code=ERROR_INVALID_JSON,
                            error_message="invalid_json: line_too_long",
                        )
                        self._send_response(conn, response)
                        continue

                    response = self._process_line(raw_line)
                    self._send_response(conn, response)
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _process_line(self, raw_line: bytes) -> dict[str, object]:
        if not raw_line:
            return self._error_response(
                request_id="",
                request_camera_id=self._config.camera_id,
                error_code=ERROR_INVALID_JSON,
                error_message="invalid_json: empty_line",
            )

        try:
            raw_request = cast(object, json.loads(raw_line.decode("utf-8")))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return self._error_response(
                request_id="",
                request_camera_id=self._config.camera_id,
                error_code=ERROR_INVALID_JSON,
                error_message="invalid_json",
            )

        if not isinstance(raw_request, dict):
            return self._error_response(
                request_id="",
                request_camera_id=self._config.camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: expected_object",
            )

        req = cast(dict[str, object], raw_request)

        request_id_obj: object = req.get("request_id")
        request_camera_id_obj: object = req.get("camera_id")
        cmd_obj: object = req.get("cmd")
        params_obj: object = req.get("params", {})

        if not isinstance(request_id_obj, str) or not request_id_obj:
            return self._error_response(
                request_id="",
                request_camera_id=self._config.camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: missing_or_invalid_request_id",
            )

        request_id = request_id_obj

        if not isinstance(request_camera_id_obj, str) or not request_camera_id_obj:
            return self._error_response(
                request_id=request_id,
                request_camera_id=self._config.camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: missing_or_invalid_camera_id",
            )

        request_camera_id = request_camera_id_obj

        if request_camera_id not in (self._config.camera_id, "broadcast"):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=(
                    f"invalid_request: camera_id_mismatch ({request_camera_id} != {self._config.camera_id})"
                ),
            )

        if not isinstance(cmd_obj, str) or not cmd_obj:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: missing_or_invalid_cmd",
            )

        cmd = cmd_obj

        if cmd not in SCHEMA_COMMANDS:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: cmd_not_in_schema ({cmd})",
            )

        if cmd not in MVP_SUPPORTED_COMMANDS:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_UNKNOWN_CMD,
                error_message=f"unknown_cmd: {cmd}",
            )

        if params_obj is None:
            params_obj = {}

        if not isinstance(params_obj, dict):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: params_must_be_object",
            )

        params = cast(dict[str, object], params_obj)

        try:
            if cmd == "ping":
                return self._ok_response(request_id, request_camera_id)

            if cmd == "start":
                return self._handle_start(request_id, request_camera_id)

            if cmd == "stop":
                return self._handle_stop(request_id, request_camera_id)

            if cmd == "set_exposure":
                value = params.get("value")
                if not isinstance(value, int):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_exposure.value must be integer",
                    )
                self._set_exposure_us(value)
                return self._ok_response(request_id, request_camera_id)

            if cmd == "set_gain":
                value = params.get("value")
                if not isinstance(value, (int, float)):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_gain.value must be number",
                    )
                self._set_gain(float(value))
                return self._ok_response(request_id, request_camera_id)

            if cmd == "set_fps":
                value = params.get("value")
                if not isinstance(value, int):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_fps.value must be integer",
                    )
                self._set_target_fps(value)
                return self._ok_response(request_id, request_camera_id)

            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_UNKNOWN_CMD,
                error_message=f"unknown_cmd: {cmd}",
            )
        except Exception as exc:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: {exc}",
            )

    def _handle_start(self, request_id: str, request_camera_id: str) -> dict[str, object]:
        with self._state_lock:
            if self._state == STATE_RUNNING:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_ALREADY_RUNNING,
                    error_message="already_running",
                )
            self._state = STATE_RUNNING

        backend: FrameBackend | None = None
        emitter: UDPFrameEmitter | None = None
        try:
            backend = self._ensure_backend()

            if self._desired_exposure_us is not None:
                backend.set_exposure_us(int(self._desired_exposure_us))
            if self._desired_gain is not None:
                backend.set_gain(float(self._desired_gain))
            if self._desired_fps is not None:
                backend.set_fps(int(self._desired_fps))

            backend.start()

            emitter = UDPFrameEmitter(
                camera_id=self._config.camera_id,
                udp_host=self._config.udp_host,
                udp_port=self._config.udp_port,
                target_fps=float(self._config.target_fps),
                backend=backend,
                threshold=int(self._config.threshold),
            )
            emitter.start()
        except BackendUnavailableError as exc:
            if emitter is not None:
                emitter.stop()
            if backend is not None:
                backend.stop()
            with self._state_lock:
                self._state = STATE_IDLE
                self._udp_emitter = None
                self._backend = None
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_BACKEND_UNAVAILABLE,
                error_message=f"backend_unavailable: {exc}",
            )
        except Exception as exc:
            if emitter is not None:
                emitter.stop()
            if backend is not None:
                backend.stop()
            with self._state_lock:
                self._state = STATE_IDLE
                self._udp_emitter = None
                self._backend = None
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: udp_start_failed: {exc}",
            )

        with self._state_lock:
            self._backend = backend
            self._udp_emitter = emitter

        return self._ok_response(request_id, request_camera_id)

    def _handle_stop(self, request_id: str, request_camera_id: str) -> dict[str, object]:
        with self._state_lock:
            if self._state == STATE_IDLE:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_NOT_RUNNING,
                    error_message="not_running",
                )

            self._state = STATE_IDLE

        try:
            emitter = self._udp_emitter
            self._udp_emitter = None
            if emitter is not None:
                emitter.stop()
            backend = self._backend
            self._backend = None
            if backend is not None:
                backend.stop()
        except Exception as exc:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: udp_stop_failed: {exc}",
            )

        return self._ok_response(request_id, request_camera_id)

    def _set_target_fps(self, fps: int) -> None:
        fps_value = int(fps)
        if fps_value <= 0:
            return
        with self._state_lock:
            self._config.target_fps = fps_value
            self._desired_fps = fps_value
            emitter = self._udp_emitter
            backend = self._backend
        if emitter is not None:
            emitter.set_target_fps(float(fps_value))
        if backend is not None:
            try:
                backend.set_fps(fps_value)
            except Exception:
                pass

    def _set_exposure_us(self, value_us: int) -> None:
        value = int(value_us)
        with self._state_lock:
            self._desired_exposure_us = value
            backend = self._backend
        if backend is not None:
            try:
                backend.set_exposure_us(value)
            except Exception:
                pass

    def _set_gain(self, value: float) -> None:
        gain_value = float(value)
        with self._state_lock:
            self._desired_gain = gain_value
            backend = self._backend
        if backend is not None:
            try:
                backend.set_gain(gain_value)
            except Exception:
                pass

    def _ensure_backend(self) -> FrameBackend:
        with self._state_lock:
            backend = self._backend
            backend_name = str(self._config.backend)

        if backend is not None:
            return backend

        if backend_name == "dummy":
            return DummyBackend()
        if backend_name == "picamera2":
            return Picamera2Backend()
        raise BackendUnavailableError(f"unknown_backend: {backend_name}")

    def _ok_response(self, request_id: str, request_camera_id: str) -> dict[str, object]:
        response_camera_id = (
            self._config.camera_id if request_camera_id == "broadcast" else request_camera_id
        )
        return {
            "request_id": request_id,
            "camera_id": response_camera_id,
            "ack": True,
        }

    def _error_response(
        self,
        request_id: str,
        request_camera_id: str,
        error_code: int,
        error_message: str,
    ) -> dict[str, object]:
        response_camera_id = (
            self._config.camera_id if request_camera_id == "broadcast" else request_camera_id
        )
        return {
            "request_id": request_id,
            "camera_id": response_camera_id,
            "ack": False,
            "error_code": error_code,
            "error_message": error_message,
        }

    @staticmethod
    def _send_response(conn: socket.socket, response: Mapping[str, object]) -> None:
        payload = json.dumps(response, separators=(",", ":")) + "\n"
        conn.sendall(payload.encode("utf-8"))


def parse_args() -> ControlServerConfig:
    parser = argparse.ArgumentParser(description="Loutrack Pi capture service (MVP control only)")
    _ = parser.add_argument("--camera-id", default="pi-cam-01", help="Camera ID handled by this Pi")
    _ = parser.add_argument("--tcp-host", default="0.0.0.0", help="TCP bind host")
    _ = parser.add_argument("--tcp-port", type=int, default=8554, help="TCP bind port")
    _ = parser.add_argument(
        "--backend",
        choices=("dummy", "picamera2"),
        default="dummy",
        help="Capture backend to use (default: dummy)",
    )
    _ = parser.add_argument(
        "--udp-dest",
        default="255.255.255.255:5000",
        help="UDP destination as host:port (one JSON per datagram)",
    )

    namespace = parser.parse_args()
    camera_id = getattr(namespace, "camera_id", "pi-cam-01")
    tcp_host = getattr(namespace, "tcp_host", "0.0.0.0")
    tcp_port = getattr(namespace, "tcp_port", 8554)
    backend = getattr(namespace, "backend", "dummy")
    udp_dest = getattr(namespace, "udp_dest", "255.255.255.255:5000")

    if not isinstance(camera_id, str):
        raise SystemExit("--camera-id must be a string")
    if not isinstance(tcp_host, str):
        raise SystemExit("--tcp-host must be a string")
    if not isinstance(tcp_port, int):
        raise SystemExit("--tcp-port must be an integer")
    if not isinstance(backend, str):
        raise SystemExit("--backend must be a string")
    if not isinstance(udp_dest, str):
        raise SystemExit("--udp-dest must be a string")

    try:
        udp_host, udp_port = parse_udp_dest(udp_dest)
    except ValueError as exc:
        raise SystemExit(f"--udp-dest invalid: {exc}") from exc

    return ControlServerConfig(
        camera_id=camera_id,
        tcp_host=tcp_host,
        tcp_port=tcp_port,
        udp_dest=udp_dest,
        udp_host=udp_host,
        udp_port=udp_port,
        backend=backend,
    )


def main() -> int:
    config = parse_args()
    server = ControlServer(config)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
