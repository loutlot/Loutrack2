#!/usr/bin/env python3
from __future__ import annotations

import argparse
import queue
import importlib
import importlib.util
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Optional, Protocol, cast

import cv2
import numpy as np

# Ensure the service directory is on sys.path so sibling modules are importable
# whether this file is run as a script or imported as src.pi.service.capture_runtime.
import os as _os, sys as _sys
_SERVICE_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _SERVICE_DIR not in _sys.path:
    _sys.path.insert(0, _SERVICE_DIR)
del _os, _sys
from intrinsics_capture import (
    _IntrinsicsCaptureConfig,
    _IntrinsicsCaptureSession,
    DEFAULT_CHARUCO_DICTIONARY,
    DEFAULT_CHARUCO_SQUARES_X,
    DEFAULT_CHARUCO_SQUARES_Y,
    DEFAULT_CHARUCO_SQUARE_LENGTH_MM,
    DEFAULT_CHARUCO_MARKER_LENGTH_MM,
    DEFAULT_INTRINSICS_TARGET_FRAMES,
    DEFAULT_INTRINSICS_SPATIAL_THRESHOLD_PX,
    CHARUCO_DETECTION_MIN_CORNERS,
)


ERROR_INVALID_JSON = 1
ERROR_INVALID_REQUEST = 2
ERROR_UNKNOWN_CMD = 3
ERROR_NOT_RUNNING = 4
ERROR_ALREADY_RUNNING = 5
ERROR_BACKEND_UNAVAILABLE = 6
ERROR_INTERNAL = 7

STATE_IDLE = "IDLE"
STATE_MASK_INIT = "MASK_INIT"
STATE_READY = "READY"
STATE_RUNNING = "RUNNING"

MASK_INIT_FRAMES = 30
MASK_THRESHOLD = 200
MASK_HIT_RATIO = 0.7
MASK_MIN_AREA_PX = 4
MASK_DILATE_PX = 2
MASK_MAX_RATIO_WARNING = 0.4
MASK_INIT_TIMEOUT_SECONDS = 10.0
DEFAULT_CIRCULARITY_MIN = 0.0
IDLE_PREVIEW_FPS = 15.0
DEFAULT_CAPTURE_WIDTH = 2304
DEFAULT_CAPTURE_HEIGHT = 1296
DEFAULT_TARGET_FPS = 56
DEFAULT_MJPEG_PORT = 8555
PREVIEW_STOP_TIMEOUT_SECONDS = 2.0
PROCESSING_QUEUE_MAXSIZE = 2
PREVIEW_QUEUE_MAXSIZE = 1
PREVIEW_MAX_DIMENSION = 1280
PTP_SANITY_CACHE_US = 60_000_000
PTP_LOCK_OFFSET_THRESHOLD_US = 500.0
PTP_RO_SOCKET_PATH = "/var/run/ptp4lro"
LINUXPTP_ROLE_PATH = "/etc/linuxptp/loutrack-role"
LINUXPTP_TIMESTAMPING_MODE_PATH = "/etc/linuxptp/loutrack-timestamping-mode"

MAX_LINE_BYTES = 65536
LINE_TIMEOUT_SECONDS = 2.0

SCHEMA_COMMANDS = {
    "start",
    "stop",
    "set_exposure",
    "set_gain",
    "set_fps",
    "set_focus",
    "set_threshold",
    "set_blob_diameter",
    "set_circularity_min",
    "mask_start",
    "mask_stop",
    "set_preview",
    "intrinsics_start",
    "intrinsics_stop",
    "intrinsics_clear",
    "intrinsics_calibrate",
    "intrinsics_status",
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
    "set_focus",
    "set_threshold",
    "set_blob_diameter",
    "set_circularity_min",
    "mask_start",
    "mask_stop",
    "set_preview",
    "intrinsics_start",
    "intrinsics_stop",
    "intrinsics_clear",
    "intrinsics_calibrate",
    "intrinsics_status",
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


def get_default_camera_id() -> str:
    try:
        completed = subprocess.run(
            ["hostnamectl", "--static"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        if completed.returncode == 0:
            value = completed.stdout.strip()
            if value:
                return value
    except Exception:
        pass

    fallback = socket.gethostname().strip()
    if fallback:
        return fallback
    return "pi-cam-01"


def resolve_debug_preview_enabled(requested: bool) -> bool:
    if not requested:
        return False

    display = os.environ.get("DISPLAY", "").strip()
    if display:
        return True

    print(
        (
            "warning: --debug-preview requested but DISPLAY is not set; "
            "preview disabled. Run from the Pi desktop terminal, or export "
            "DISPLAY=:0 and XAUTHORITY=/home/<PI_USER>/.Xauthority before starting capture."
        ),
        file=sys.stderr,
    )
    return False


def running_on_raspberry_pi() -> bool:
    model_path = "/proc/device-tree/model"
    try:
        model = open(model_path, "r", encoding="utf-8").read().strip()
    except Exception:
        return False
    return "Raspberry Pi" in model


def get_default_backend() -> str:
    return "picamera2" if running_on_raspberry_pi() else "dummy"


def _latest_queue_put(target_queue: queue.Queue[object], item: object) -> int:
    dropped = 0
    while True:
        try:
            target_queue.put_nowait(item)
            return dropped
        except queue.Full:
            try:
                _ = target_queue.get_nowait()
                dropped += 1
            except queue.Empty:
                time.sleep(0)


def _pose_capture_quality(blobs: list[dict[str, object]]) -> float:
    if not blobs:
        return 0.0
    quality = 1.0 / float(max(1, len(blobs)))
    max_area = max(float(blob.get("area", 0.0)) for blob in blobs)
    if max_area <= 0.0:
        quality *= 0.25
    return float(max(0.0, min(1.0, quality)))


def _resize_preview_payload(
    frame: np.ndarray,
    blobs: list[dict[str, float]],
    mask: np.ndarray | None,
) -> tuple[np.ndarray, list[dict[str, float]], np.ndarray | None]:
    height, width = frame.shape[:2]
    max_dim = max(height, width)
    if max_dim <= PREVIEW_MAX_DIMENSION:
        return frame, blobs, mask

    scale = float(PREVIEW_MAX_DIMENSION) / float(max_dim)
    target_width = max(1, int(round(width * scale)))
    target_height = max(1, int(round(height * scale)))
    resized_frame = cv2.resize(
        frame,
        (target_width, target_height),
        interpolation=cv2.INTER_AREA,
    )
    resized_blobs = [
        {
            "x": float(blob["x"]) * scale,
            "y": float(blob["y"]) * scale,
            "area": float(blob["area"]) * scale * scale,
        }
        for blob in blobs
    ]
    resized_mask: np.ndarray | None = None
    if mask is not None:
        resized_mask = cv2.resize(
            mask.astype(np.uint8),
            (target_width, target_height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
    return resized_frame, resized_blobs, resized_mask


def _finalize_static_mask_from_hits(
    hit_counts: np.ndarray,
    *,
    frames: int,
    hit_ratio: float,
    min_area: int,
    dilate: int,
) -> tuple[np.ndarray, int]:
    required = max(1, math.ceil(float(hit_ratio) * float(frames)))
    mask_uint8 = (hit_counts >= required).astype(np.uint8) * 255

    if min_area > 0:
        contours_info = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                cv2.drawContours(mask_uint8, [contour], -1, 0, thickness=-1)

    if dilate > 0:
        kernel_size = max(1, dilate) * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask_uint8 = cv2.dilate(mask_uint8, kernel)

    mask = mask_uint8.astype(bool)
    return mask, int(np.count_nonzero(mask))


@dataclass(frozen=True)
class _FramePacket:
    captured_frame: CapturedFrame
    captured_monotonic_ns: int


@dataclass(frozen=True)
class _PreviewPacket:
    frame: np.ndarray
    blobs: list[dict[str, float]]
    mask: np.ndarray | None
    stats: dict[str, object]
    extra_lines: tuple[str, ...] = ()


@dataclass(frozen=True)
class PreviewOverlayOptions:
    blob: bool = True
    mask: bool = True
    text: bool = True
    charuco: bool = True

    def to_dict(self) -> dict[str, bool]:
        return {
            "blob": bool(self.blob),
            "mask": bool(self.mask),
            "text": bool(self.text),
            "charuco": bool(self.charuco),
        }


@dataclass(frozen=True)
class PreviewCharucoConfig:
    dictionary: str = DEFAULT_CHARUCO_DICTIONARY
    squares_x: int = DEFAULT_CHARUCO_SQUARES_X
    squares_y: int = DEFAULT_CHARUCO_SQUARES_Y
    square_length_mm: float = DEFAULT_CHARUCO_SQUARE_LENGTH_MM
    marker_length_mm: float = DEFAULT_CHARUCO_MARKER_LENGTH_MM

    def to_dict(self) -> dict[str, object]:
        return {
            "dictionary": str(self.dictionary),
            "squares_x": int(self.squares_x),
            "squares_y": int(self.squares_y),
            "square_length_mm": float(self.square_length_mm),
            "marker_length_mm": float(self.marker_length_mm),
        }


@dataclass(frozen=True)
class PreviewRenderConfig:
    render_enabled: bool = False
    overlays: PreviewOverlayOptions = field(default_factory=PreviewOverlayOptions)
    charuco: PreviewCharucoConfig = field(default_factory=PreviewCharucoConfig)


class _MaskBuildRequest:
    def __init__(
        self,
        *,
        frames: int,
        threshold: int,
        hit_ratio: float,
        min_area: int,
        dilate: int,
        deadline_monotonic: float | None,
    ) -> None:
        self.frames = int(frames)
        self.threshold = int(threshold)
        self.hit_ratio = float(hit_ratio)
        self.min_area = int(min_area)
        self.dilate = int(dilate)
        self.deadline_monotonic = deadline_monotonic
        self._lock = threading.Lock()
        self._completed = threading.Event()
        self._hit_counts: np.ndarray | None = None
        self._frames_seen = 0
        self._mask: np.ndarray | None = None
        self._mask_pixels = 0
        self._error: Exception | None = None

    @property
    def completed(self) -> bool:
        return self._completed.is_set()

    def consume_frame(self, frame: np.ndarray) -> None:
        with self._lock:
            if self._completed.is_set():
                return
            if (
                self.deadline_monotonic is not None
                and time.perf_counter() > self.deadline_monotonic
            ):
                self._error = TimeoutError("mask_init_timed_out")
                self._completed.set()
                return

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            if self._hit_counts is None:
                self._hit_counts = np.zeros(gray.shape, dtype=np.uint16)
            self._hit_counts += (gray > self.threshold).astype(np.uint16)
            self._frames_seen += 1

            if self._frames_seen < self.frames:
                return

            try:
                self._mask, self._mask_pixels = _finalize_static_mask_from_hits(
                    self._hit_counts,
                    frames=self.frames,
                    hit_ratio=self.hit_ratio,
                    min_area=self.min_area,
                    dilate=self.dilate,
                )
            except Exception as exc:  # noqa: BLE001
                self._error = exc
            self._completed.set()

    def wait(self, timeout: float | None = None) -> tuple[np.ndarray, int]:
        if not self._completed.wait(timeout=timeout):
            raise TimeoutError("mask_init_timed_out")
        with self._lock:
            if self._error is not None:
                raise self._error
            if self._mask is None:
                raise RuntimeError("mask_init_incomplete")
            return self._mask, int(self._mask_pixels)


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
        min_diameter_px: float | None = None,
        max_diameter_px: float | None = None,
        circularity_min: float = DEFAULT_CIRCULARITY_MIN,
        time_us_fn: Callable[[], int] | None = None,
        max_frames: int | None = None,
        mask: np.ndarray | None = None,
        debug_preview: Optional["DebugPreview"] = None,
        capture_mode: str = "capture",
    ):
        self._camera_id: str = camera_id
        self._udp_host: str = udp_host
        self._udp_port: int = udp_port
        self._threshold: int = int(threshold)
        self._min_diameter_px: float | None = (
            float(min_diameter_px) if min_diameter_px is not None else None
        )
        self._max_diameter_px: float | None = (
            float(max_diameter_px) if max_diameter_px is not None else None
        )
        self._circularity_min: float = float(circularity_min)
        self._last_detection_stats: dict[str, object] = {
            "threshold": self._threshold,
            "min_diameter_px": self._min_diameter_px,
            "max_diameter_px": self._max_diameter_px,
            "circularity_min": self._circularity_min,
            "raw_contour_count": 0,
            "accepted_blob_count": 0,
            "rejected_by_diameter": 0,
            "rejected_by_circularity": 0,
            "last_blob_count": 0,
        }
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
        self._mask: np.ndarray | None = (
            mask.astype(bool, copy=False) if mask is not None else None
        )
        self._debug_preview: DebugPreview | None = debug_preview
        self._capture_mode: str = str(capture_mode)
        self._time_override_enabled: bool = time_us_fn is not None
        self._last_timestamping_status: dict[str, object] = {
            "active_source": "capture_dequeue",
            "sensor_timestamp_available": False,
        }

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

    def set_threshold(self, threshold: int) -> None:
        threshold_value = max(0, min(255, int(threshold)))
        with self._lock:
            self._threshold = threshold_value

    def set_blob_diameter(
        self,
        min_diameter_px: float | None,
        max_diameter_px: float | None,
    ) -> None:
        with self._lock:
            self._min_diameter_px = (
                float(min_diameter_px) if min_diameter_px is not None else None
            )
            self._max_diameter_px = (
                float(max_diameter_px) if max_diameter_px is not None else None
            )

    def set_circularity_min(self, value: float) -> None:
        with self._lock:
            self._circularity_min = float(value)

    def get_last_detection_stats(self) -> dict[str, object]:
        with self._lock:
            return dict(self._last_detection_stats)

    def get_last_timestamping_status(self) -> dict[str, object]:
        with self._lock:
            return dict(self._last_timestamping_status)

    def set_mask(self, mask: np.ndarray | None) -> None:
        with self._lock:
            self._mask = mask.astype(bool, copy=False) if mask is not None else None

    def _loop(self) -> None:
        sent = 0
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                sock = self._socket
                fps = self._target_fps
                max_frames = self._max_frames
                mask = self._mask
                threshold = self._threshold
                min_diameter_px = self._min_diameter_px
                max_diameter_px = self._max_diameter_px
                circularity_min = self._circularity_min

            if sock is None:
                return

            try:
                captured_frame = self._backend.next_captured_frame()
                frame = captured_frame.image
                blobs, detection_stats = detect_blobs(
                    frame,
                    threshold=threshold,
                    mask=mask,
                    min_diameter_px=min_diameter_px,
                    max_diameter_px=max_diameter_px,
                    circularity_min=circularity_min,
                )
                with self._lock:
                    self._last_detection_stats = detection_stats
                    self._last_timestamping_status = {
                        "active_source": str(captured_frame.timestamp_source),
                        "sensor_timestamp_available": bool(captured_frame.sensor_timestamp_ns is not None),
                    }
                if self._debug_preview is not None:
                    self._debug_preview.show(
                        frame=frame,
                        blobs=blobs,
                        mask=mask,
                        stats=detection_stats,
                        camera_id=self._camera_id,
                    )
            except Exception:
                return
            msg = {
                "camera_id": self._camera_id,
                "timestamp": self._public_timestamp_us(captured_frame),
                "timestamp_source": str(captured_frame.timestamp_source),
                "frame_index": int(self._frame_index & 0xFFFFFFFF),
                "blobs": blobs,
            }
            if self._capture_mode == "pose_capture":
                msg["blob_count"] = len(blobs)
                msg["quality"] = _pose_capture_quality(cast(list[dict[str, object]], blobs))
            msg["capture_mode"] = self._capture_mode
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

    def _public_timestamp_us(self, captured_frame: "CapturedFrame") -> int:
        if (
            self._time_override_enabled
            and str(captured_frame.timestamp_source) == "capture_dequeue"
        ):
            return int(self._time_us_fn())
        value = int(captured_frame.timestamp_us)
        if value >= 0:
            return value
        return int(self._time_us_fn())


@dataclass
class DummyBackendConfig:
    width: int = DEFAULT_CAPTURE_WIDTH
    height: int = DEFAULT_CAPTURE_HEIGHT
    num_dots: int = 3
    seed: int = 0
    dot_radius: int = 3
    background_value: int = 0
    dot_value: int = 255


class BackendUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class CapturedFrame:
    image: np.ndarray
    timestamp_us: int
    timestamp_source: Literal["sensor_metadata", "capture_dequeue"]
    sensor_timestamp_ns: int | None = None


@dataclass(frozen=True)
class ClockSyncSnapshot:
    status: Literal["locked", "degraded", "unknown"]
    offset_us: float | None
    source: Literal["pmc", "unavailable"]
    role: Literal["master", "slave", "unknown"] = "unknown"
    timestamping_mode: Literal["software", "hardware", "unknown"] = "unknown"


def _get_aruco_module() -> Any | None:
    return getattr(cv2, "aruco", None)


def _create_aruco_dictionary(aruco: Any, dictionary_name: str) -> Any | None:
    dictionary_id = getattr(aruco, dictionary_name, None)
    if dictionary_id is None:
        return None
    get_predefined = getattr(aruco, "getPredefinedDictionary", None)
    if callable(get_predefined):
        try:
            return get_predefined(dictionary_id)
        except Exception:
            return None
    dictionary_get = getattr(aruco, "Dictionary_get", None)
    if callable(dictionary_get):
        try:
            return dictionary_get(dictionary_id)
        except Exception:
            return None
    return None


def _create_aruco_detector_parameters(aruco: Any) -> Any | None:
    ctor = getattr(aruco, "DetectorParameters", None)
    if callable(ctor):
        try:
            return ctor()
        except Exception:
            return None
    legacy_ctor = getattr(aruco, "DetectorParameters_create", None)
    if callable(legacy_ctor):
        try:
            return legacy_ctor()
        except Exception:
            return None
    return None


def _create_charuco_board(
    aruco: Any,
    *,
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
    dictionary: Any,
) -> Any | None:
    board_cls = getattr(aruco, "CharucoBoard", None)
    if board_cls is not None:
        try:
            return board_cls((squares_x, squares_y), square_length_m, marker_length_m, dictionary)
        except TypeError:
            create_fn = getattr(board_cls, "create", None)
            if callable(create_fn):
                try:
                    return create_fn(squares_x, squares_y, square_length_m, marker_length_m, dictionary)
                except Exception:
                    return None
        except Exception:
            return None

    legacy_create = getattr(aruco, "CharucoBoard_create", None)
    if callable(legacy_create):
        try:
            return legacy_create(squares_x, squares_y, square_length_m, marker_length_m, dictionary)
        except Exception:
            return None
    return None


def _preview_text_lines(
    *,
    camera_id: str,
    stats: dict[str, object],
    extra_lines: list[str] | None,
) -> list[str]:
    lines = [
        f"{camera_id}",
        f"accepted={stats.get('accepted_blob_count', 0)} raw={stats.get('raw_contour_count', 0)}",
        (
            f"rej_diam={stats.get('rejected_by_diameter', 0)} "
            f"rej_circ={stats.get('rejected_by_circularity', 0)}"
        ),
        (
            f"thr={stats.get('threshold')} "
            f"diam=[{stats.get('min_diameter_px')},{stats.get('max_diameter_px')}] "
            f"circ>={stats.get('circularity_min')}"
        ),
    ]
    if extra_lines:
        lines.extend(extra_lines)
    return lines


def _render_preview_canvas(
    *,
    frame: np.ndarray,
    blobs: list[dict[str, float]],
    mask: np.ndarray | None,
    stats: dict[str, object],
    camera_id: str,
    extra_lines: list[str] | None,
    overlays: PreviewOverlayOptions,
    charuco_renderer: "_CharucoOverlayRenderer" | None = None,
    charuco_config: PreviewCharucoConfig | None = None,
) -> np.ndarray:
    if frame.ndim == 2:
        canvas = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:
        canvas = frame.copy()

    if overlays.mask and mask is not None and mask.shape[:2] == canvas.shape[:2]:
        mask_bool = mask.astype(bool, copy=False)
        if mask_bool.any():
            mask_overlay = np.zeros_like(canvas)
            mask_overlay[:, :, 2] = 255
            blended = cv2.addWeighted(canvas, 0.72, mask_overlay, 0.28, 0)
            canvas[mask_bool] = blended[mask_bool]

    if overlays.blob:
        for blob in blobs:
            center = (int(round(float(blob["x"]))), int(round(float(blob["y"]))))
            radius = max(4, int(round(math.sqrt(max(float(blob["area"]), 1.0) / math.pi))))
            cv2.circle(canvas, center, radius, (0, 255, 0), 2)
            cv2.circle(canvas, center, 2, (0, 255, 255), -1)

    if overlays.charuco and charuco_renderer is not None and charuco_config is not None:
        charuco_renderer.draw(canvas, charuco_config)

    if overlays.text:
        lines = _preview_text_lines(camera_id=camera_id, stats=stats, extra_lines=extra_lines)
        y = 24
        for line in lines:
            cv2.putText(
                canvas,
                line,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y += 22
    return canvas


def _encode_preview_jpeg(image: np.ndarray, quality: int = 75) -> bytes | None:
    try:
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    except Exception:
        return None
    if not ok:
        return None
    return bytes(buf.tobytes())


def _build_placeholder_jpeg(
    message: str,
    *,
    width: int = 960,
    height: int = 540,
    quality: int = 70,
) -> bytes:
    canvas = np.zeros((max(64, height), max(64, width), 3), dtype=np.uint8)
    canvas[:] = (14, 18, 24)
    cv2.putText(
        canvas,
        message,
        (24, canvas.shape[0] // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (180, 220, 255),
        2,
        cv2.LINE_AA,
    )
    encoded = _encode_preview_jpeg(canvas, quality=quality)
    return encoded if encoded is not None else b""


class _CharucoOverlayRenderer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cache_key: tuple[str, int, int, float, float] | None = None
        self._cached_dictionary: Any = None
        self._cached_board: Any = None
        self._available: bool = True

    def is_dictionary_supported(self, dictionary_name: str) -> bool:
        aruco = _get_aruco_module()
        if aruco is None:
            return True
        return getattr(aruco, dictionary_name, None) is not None

    def draw(self, image: np.ndarray, config: PreviewCharucoConfig) -> None:
        resources = self._resources(config)
        if resources is None:
            return
        aruco, dictionary, board = resources
        detector_params = _create_aruco_detector_parameters(aruco)
        if detector_params is None:
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        try:
            if hasattr(aruco, "ArucoDetector"):
                detector = aruco.ArucoDetector(dictionary, detector_params)
                marker_corners, marker_ids, _ = detector.detectMarkers(gray)
            else:
                detect_markers = getattr(aruco, "detectMarkers", None)
                if not callable(detect_markers):
                    return
                marker_corners, marker_ids, _ = detect_markers(gray, dictionary, parameters=detector_params)
        except Exception:
            return
        if marker_ids is None or len(marker_ids) < 4:
            return

        interpolate = getattr(aruco, "interpolateCornersCharuco", None)
        if not callable(interpolate):
            return
        try:
            result = interpolate(marker_corners, marker_ids, gray, board)
        except Exception:
            return

        charuco_corners: Any = None
        charuco_ids: Any = None
        if isinstance(result, tuple):
            if len(result) >= 2 and isinstance(result[0], np.ndarray):
                charuco_corners = result[0]
                charuco_ids = result[1]
            elif len(result) >= 3:
                retval = bool(result[0])
                if retval:
                    charuco_corners = result[1]
                    charuco_ids = result[2]
        if charuco_corners is None or charuco_ids is None:
            return
        if len(charuco_corners) < CHARUCO_DETECTION_MIN_CORNERS:
            return

        draw_charuco = getattr(aruco, "drawDetectedCornersCharuco", None)
        if not callable(draw_charuco):
            return
        try:
            draw_charuco(image, charuco_corners, charuco_ids)
        except Exception:
            return

    def _resources(self, config: PreviewCharucoConfig) -> tuple[Any, Any, Any] | None:
        if not self._available:
            return None
        key = (
            str(config.dictionary),
            int(config.squares_x),
            int(config.squares_y),
            float(config.square_length_mm),
            float(config.marker_length_mm),
        )
        with self._lock:
            if self._cache_key == key and self._cached_dictionary is not None and self._cached_board is not None:
                aruco = _get_aruco_module()
                if aruco is None:
                    self._available = False
                    return None
                return aruco, self._cached_dictionary, self._cached_board

            aruco = _get_aruco_module()
            if aruco is None:
                self._available = False
                return None
            dictionary = _create_aruco_dictionary(aruco, str(config.dictionary))
            if dictionary is None:
                return None
            board = _create_charuco_board(
                aruco,
                squares_x=int(config.squares_x),
                squares_y=int(config.squares_y),
                square_length_m=float(config.square_length_mm) / 1000.0,
                marker_length_m=float(config.marker_length_mm) / 1000.0,
                dictionary=dictionary,
            )
            if board is None:
                return None
            self._cache_key = key
            self._cached_dictionary = dictionary
            self._cached_board = board
            return aruco, dictionary, board


class DebugPreview:
    def __init__(
        self,
        window_name: str = "loutrack2-debug-preview",
        log_fn: Callable[[str], None] | None = None,
    ) -> None:
        self._window_name = window_name
        self._enabled = True
        self._initialized = False
        self._lock = threading.Lock()
        self._last_canvas: np.ndarray | None = None
        self._log_fn = log_fn
        self._render_count = 0

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def window_open(self) -> bool:
        with self._lock:
            return self._initialized

    def show(
        self,
        *,
        frame: np.ndarray,
        blobs: list[dict[str, float]],
        mask: np.ndarray | None,
        stats: dict[str, object],
        camera_id: str,
        extra_lines: list[str] | None = None,
    ) -> None:
        if not self._enabled:
            return
        try:
            canvas = self._build_canvas(frame, blobs, mask, stats, camera_id, extra_lines)
            self._render_canvas(canvas)
        except Exception as exc:
            self._trace(f"debug preview show failed: {exc}")
            self.close_window()

    def close(self) -> None:
        if not self._enabled:
            return
        self._enabled = False
        self.close_window()

    def close_window(self) -> None:
        with self._lock:
            if self._initialized:
                try:
                    self._trace("debug preview destroyWindow begin")
                    cv2.destroyWindow(self._window_name)
                    _ = cv2.waitKey(1)
                    self._trace("debug preview destroyWindow ok")
                except Exception:
                    pass
                self._initialized = False

    def _render_canvas(self, canvas: np.ndarray) -> None:
        with self._lock:
            render_count = self._render_count + 1
            if not self._initialized:
                try:
                    cv2.startWindowThread()
                except Exception:
                    pass
                self._trace(
                    f"debug preview render {render_count} namedWindow begin"
                )
                cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
                self._initialized = True
                self._trace(
                    f"debug preview render {render_count} namedWindow ok"
                )
            self._trace(
                "debug preview render "
                f"{render_count} imshow begin shape={canvas.shape}"
            )
            cv2.imshow(self._window_name, canvas)
            self._trace(f"debug preview render {render_count} imshow ok")
            _ = cv2.waitKey(1)
            self._trace(f"debug preview render {render_count} waitKey ok")
            self._last_canvas = canvas.copy()
            self._render_count = render_count

    def _trace(self, message: str) -> None:
        if self._log_fn is None:
            return
        if self._render_count < 3 or "failed" in message or "destroyWindow" in message:
            self._log_fn(message)

    def _build_canvas(
        self,
        frame: np.ndarray,
        blobs: list[dict[str, float]],
        mask: np.ndarray | None,
        stats: dict[str, object],
        camera_id: str,
        extra_lines: list[str] | None,
    ) -> np.ndarray:
        return _render_preview_canvas(
            frame=frame,
            blobs=blobs,
            mask=mask,
            stats=stats,
            camera_id=camera_id,
            extra_lines=extra_lines,
            overlays=PreviewOverlayOptions(blob=True, mask=True, text=True, charuco=False),
        )


class FrameBackend(Protocol):
    def start(self) -> None: ...

    def stop(self) -> None: ...

    def next_captured_frame(self) -> CapturedFrame: ...

    def next_frame(self) -> np.ndarray: ...

    def capture_array(self) -> np.ndarray: ...

    def set_exposure_us(self, value_us: int) -> None: ...

    def set_gain(self, value: float) -> None: ...

    def set_fps(self, fps: int) -> None: ...

    def set_focus(self, value: float) -> None: ...


class _Picamera2Api(Protocol):
    def create_video_configuration(self, *, main: dict[str, object]) -> object: ...

    def configure(self, config: object) -> None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...

    def capture_array(self) -> object: ...

    def capture_request(self) -> object: ...

    def set_controls(self, controls: dict[str, object]) -> object: ...


class _Picamera2Request(Protocol):
    def make_array(self, stream: str = "main") -> object: ...

    def get_metadata(self) -> object: ...

    def release(self) -> None: ...


class DummyBackend:
    def __init__(self, config: DummyBackendConfig | None = None):
        self._config: DummyBackendConfig = config if config is not None else DummyBackendConfig()
        self._frame_index: int = 0
        self._exposure_us: int | None = None
        self._gain: float | None = None
        self._fps: int | None = None
        self._focus: float | None = None

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

    def set_focus(self, value: float) -> None:
        self._focus = float(value)

    @property
    def frame_index(self) -> int:
        return self._frame_index

    def next_captured_frame(self) -> CapturedFrame:
        frame = self._generate_frame()
        return CapturedFrame(
            image=frame,
            timestamp_us=int(time.time_ns() // 1000),
            timestamp_source="capture_dequeue",
            sensor_timestamp_ns=None,
        )

    def next_frame(self) -> np.ndarray:
        return self.next_captured_frame().image

    def _generate_frame(self) -> np.ndarray:
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

    def capture_array(self) -> np.ndarray:
        return self.next_captured_frame().image


@dataclass
class Picamera2BackendConfig:
    width: int = DEFAULT_CAPTURE_WIDTH
    height: int = DEFAULT_CAPTURE_HEIGHT
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
        self._focus: float | None = None

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

    def set_focus(self, value: float) -> None:
        self._focus = float(value)
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

    def next_captured_frame(self) -> CapturedFrame:
        picam2 = self._picam2
        if picam2 is None or not self._running:
            raise RuntimeError("picamera2 backend not running")

        capture_realtime_us: int | None = None
        capture_monotonic_us: int | None = None
        sensor_timestamp_ns: int | None = None

        if hasattr(picam2, "capture_request"):
            request_obj = None
            try:
                request_obj = picam2.capture_request()
                capture_realtime_us = _clock_realtime_us()
                capture_monotonic_us = _clock_monotonic_us()
                request = cast(_Picamera2Request, request_obj)
                try:
                    frame_obj = request.make_array("main")
                except TypeError:
                    frame_obj = request.make_array()
                metadata = request.get_metadata()
                sensor_timestamp_ns = _extract_sensor_timestamp_ns(metadata)
                if not isinstance(frame_obj, np.ndarray):
                    raise RuntimeError("picamera2 capture_request.make_array returned non-ndarray")
                timestamp_us = _sensor_timestamp_to_epoch_us(
                    sensor_timestamp_ns,
                    capture_realtime_us,
                    capture_monotonic_us,
                )
                try:
                    request.release()
                except Exception:
                    pass
                request_obj = None
                if timestamp_us is not None:
                    return CapturedFrame(
                        image=cast(np.ndarray, frame_obj),
                        timestamp_us=timestamp_us,
                        timestamp_source="sensor_metadata",
                        sensor_timestamp_ns=sensor_timestamp_ns,
                    )
                return CapturedFrame(
                    image=cast(np.ndarray, frame_obj),
                    timestamp_us=int(capture_realtime_us),
                    timestamp_source="capture_dequeue",
                    sensor_timestamp_ns=sensor_timestamp_ns,
                )
            except Exception:
                if request_obj is not None and hasattr(request_obj, "release"):
                    try:
                        cast(_Picamera2Request, request_obj).release()
                    except Exception:
                        pass
            else:
                if request_obj is not None and hasattr(request_obj, "release"):
                    try:
                        cast(_Picamera2Request, request_obj).release()
                    except Exception:
                        pass

        frame = picam2.capture_array()
        capture_realtime_us = _clock_realtime_us()
        if not isinstance(frame, np.ndarray):
            raise RuntimeError("picamera2 capture_array returned non-ndarray")
        return CapturedFrame(
            image=cast(np.ndarray, frame),
            timestamp_us=int(capture_realtime_us),
            timestamp_source="capture_dequeue",
            sensor_timestamp_ns=None,
        )

    def next_frame(self) -> np.ndarray:
        return self.next_captured_frame().image

    def capture_array(self) -> np.ndarray:
        return self.next_captured_frame().image

    def _apply_controls_if_running(self) -> None:
        if not self._running or self._picam2 is None:
            return
        picam2 = self._picam2

        controls: dict[str, object] = {}
        if self._exposure_us is not None or self._gain is not None:
            controls["AeEnable"] = False
        if self._exposure_us is not None:
            controls["ExposureTime"] = int(self._exposure_us)
        if self._gain is not None:
            controls["AnalogueGain"] = float(self._gain)
        if self._fps is not None and self._fps > 0:
            frame_us = int(round(1_000_000 / float(self._fps)))
            controls["FrameDurationLimits"] = (frame_us, frame_us)
        if self._focus is not None:
            controls["AfMode"] = self._manual_focus_mode()
            controls["LensPosition"] = float(self._focus)

        if not controls:
            return

        try:
            _ = picam2.set_controls(controls)
        except Exception:
            return

    @staticmethod
    def _manual_focus_mode() -> int:
        try:
            libcamera = importlib.import_module("libcamera")
        except Exception:
            return 0

        controls_mod = getattr(libcamera, "controls", None)
        if controls_mod is None:
            return 0
        af_mode_enum = getattr(controls_mod, "AfModeEnum", None)
        if af_mode_enum is None:
            return 0
        return int(getattr(af_mode_enum, "Manual", 0))


def _clock_realtime_us() -> int:
    if hasattr(time, "clock_gettime_ns") and hasattr(time, "CLOCK_REALTIME"):
        return int(time.clock_gettime_ns(time.CLOCK_REALTIME) // 1000)
    return int(time.time_ns() // 1000)


def _clock_monotonic_us() -> int:
    if hasattr(time, "clock_gettime_ns") and hasattr(time, "CLOCK_MONOTONIC"):
        return int(time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000)
    return int(time.monotonic_ns() // 1000)


def _sensor_timestamp_to_epoch_us(
    sensor_timestamp_ns: int | None,
    realtime_us: int | None = None,
    monotonic_us: int | None = None,
) -> int | None:
    if sensor_timestamp_ns is None:
        return None
    try:
        sensor_timestamp_us = int(sensor_timestamp_ns) // 1000
    except (TypeError, ValueError, OverflowError):
        return None
    if sensor_timestamp_us < 0:
        return None
    realtime_value = _clock_realtime_us() if realtime_us is None else int(realtime_us)
    monotonic_value = _clock_monotonic_us() if monotonic_us is None else int(monotonic_us)
    epoch_offset_us = realtime_value - monotonic_value
    timestamp_us = sensor_timestamp_us + epoch_offset_us
    if timestamp_us < 0:
        return None
    return int(timestamp_us)


def _extract_sensor_timestamp_ns(metadata: object) -> int | None:
    if not isinstance(metadata, Mapping):
        return None
    for key in ("SensorTimestamp", "sensor_timestamp_ns"):
        value = metadata.get(key)
        if isinstance(value, (int, float)):
            try:
                timestamp_ns = int(value)
            except (TypeError, ValueError, OverflowError):
                return None
            return timestamp_ns if timestamp_ns >= 0 else None
    return None


def _read_linuxptp_setting(
    path: str,
    allowed: set[str],
) -> str:
    try:
        value = Path(path).read_text(encoding="utf-8").strip().lower()
    except OSError:
        return "unknown"
    return value if value in allowed else "unknown"


def _parse_pmc_time_status(
    stdout: str,
    *,
    role: Literal["master", "slave", "unknown"] = "unknown",
    timestamping_mode: Literal["software", "hardware", "unknown"] = "unknown",
) -> ClockSyncSnapshot:
    offset_match = re.search(r"\bmaster_offset\s+(-?\d+(?:\.\d+)?)", stdout)
    if offset_match is None:
        return ClockSyncSnapshot(
            status="unknown",
            offset_us=None,
            source="unavailable",
            role=role,
            timestamping_mode=timestamping_mode,
        )
    try:
        offset_ns = float(offset_match.group(1))
    except ValueError:
        return ClockSyncSnapshot(
            status="unknown",
            offset_us=None,
            source="unavailable",
            role=role,
            timestamping_mode=timestamping_mode,
        )
    offset_us = offset_ns / 1000.0
    servo_healthy = True
    gm_present = re.search(r"\bgmPresent\s+(true|false)\b", stdout, flags=re.IGNORECASE)
    if gm_present is not None and gm_present.group(1).lower() == "false" and role != "master":
        servo_healthy = False
    port_state = re.search(r"\bportState\s+([A-Z_]+)\b", stdout)
    if port_state is not None and port_state.group(1) in {"FAULTY", "DISABLED", "LISTENING"}:
        servo_healthy = False
    status: Literal["locked", "degraded", "unknown"] = "locked"
    if not servo_healthy or abs(offset_us) > PTP_LOCK_OFFSET_THRESHOLD_US:
        status = "degraded"
    return ClockSyncSnapshot(
        status=status,
        offset_us=offset_us,
        source="pmc",
        role=role,
        timestamping_mode=timestamping_mode,
    )


from blob_detection import detect_blobs  # extracted to blob_detection.py

class _PreviewWorker:
    def __init__(
        self,
        *,
        preview: DebugPreview,
        camera_id: str,
        log_fn: Callable[[str], None],
    ) -> None:
        self._preview = preview
        self._camera_id = camera_id
        self._log = log_fn
        self._queue: queue.Queue[object] = queue.Queue(maxsize=PREVIEW_QUEUE_MAXSIZE)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._frames_rendered = 0
        self._frames_dropped = 0
        self._started_monotonic = 0.0

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._started_monotonic = time.monotonic()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._lock:
            thread = self._thread
            self._thread = None
        if thread is not None:
            thread.join(timeout=1.0)

    def submit(self, packet: _PreviewPacket) -> None:
        dropped = _latest_queue_put(self._queue, packet)
        if dropped <= 0:
            return
        with self._lock:
            self._frames_dropped += dropped

    def is_active(self) -> bool:
        with self._lock:
            thread = self._thread
        return bool(
            (thread is not None and thread.is_alive())
            or self._preview.window_open
        )

    def get_runtime_diagnostics(self) -> dict[str, object]:
        with self._lock:
            frames_rendered = self._frames_rendered
            frames_dropped = self._frames_dropped
            started_monotonic = self._started_monotonic
        elapsed = max(1e-6, time.monotonic() - started_monotonic) if started_monotonic > 0.0 else 0.0
        preview_fps = float(frames_rendered) / elapsed if elapsed > 0.0 else 0.0
        return {
            "preview_fps": preview_fps,
            "frames_dropped_preview": int(frames_dropped),
        }

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                packet = cast(_PreviewPacket, self._queue.get(timeout=0.1))
            except queue.Empty:
                continue
            try:
                self._preview.show(
                    frame=packet.frame,
                    blobs=packet.blobs,
                    mask=packet.mask,
                    stats=packet.stats,
                    camera_id=self._camera_id,
                    extra_lines=list(packet.extra_lines),
                )
            except Exception as exc:  # noqa: BLE001
                self._log(f"preview worker failed: {exc}")
            with self._lock:
                self._frames_rendered += 1


class _CameraWorker:
    def __init__(
        self,
        *,
        backend_factory: Callable[[], FrameBackend],
        apply_backend_settings: Callable[[FrameBackend], None],
        frame_queue: queue.Queue[object],
        active_event: threading.Event,
        get_loop_fps: Callable[[], float],
        log_fn: Callable[[str], None],
    ) -> None:
        self._backend_factory = backend_factory
        self._apply_backend_settings = apply_backend_settings
        self._frame_queue = frame_queue
        self._active_event = active_event
        self._get_loop_fps = get_loop_fps
        self._log = log_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._backend: FrameBackend | None = None
        self._frames_captured = 0
        self._frames_dropped_processing = 0
        self._started_monotonic = 0.0
        self._frame_buffer: deque[np.ndarray] = deque(maxlen=1)

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            backend = self._backend_factory()
            self._apply_backend_settings(backend)
            backend.start()
            self._backend = backend
            self._stop_event.clear()
            self._started_monotonic = time.monotonic()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._active_event.set()
        with self._lock:
            thread = self._thread
            backend = self._backend
            self._thread = None
            self._backend = None
        if thread is not None:
            thread.join(timeout=1.0)
        if backend is not None:
            try:
                backend.stop()
            except Exception:
                pass

    def get_runtime_diagnostics(self) -> dict[str, object]:
        with self._lock:
            frames_captured = self._frames_captured
            dropped = self._frames_dropped_processing
            started_monotonic = self._started_monotonic
            backend = self._backend
        elapsed = max(1e-6, time.monotonic() - started_monotonic) if started_monotonic > 0.0 else 0.0
        capture_fps = float(frames_captured) / elapsed if elapsed > 0.0 else 0.0
        return {
            "capture_fps": capture_fps,
            "frames_dropped_processing": int(dropped),
            "backend_active": bool(backend is not None),
        }

    def apply_control(self, fn: Callable[[FrameBackend], None]) -> None:
        with self._lock:
            backend = self._backend
        if backend is None:
            return
        fn(backend)

    def get_frame_snapshot(self) -> np.ndarray | None:
        buf = self._frame_buffer
        return buf[-1].copy() if buf else None

    def _loop(self) -> None:
        with self._lock:
            backend = self._backend
        if backend is None:
            return
        is_dummy_backend = isinstance(backend, DummyBackend)
        while not self._stop_event.is_set():
            if not self._active_event.wait(timeout=0.1):
                continue
            try:
                captured_frame = backend.next_captured_frame()
            except Exception as exc:  # noqa: BLE001
                self._log(f"camera worker capture failed: {exc}")
                self._stop_event.wait(0.05)
                continue
            self._frame_buffer.append(captured_frame.image)
            dropped = _latest_queue_put(
                self._frame_queue,
                _FramePacket(
                    captured_frame=captured_frame,
                    captured_monotonic_ns=time.monotonic_ns(),
                ),
            )
            with self._lock:
                self._frames_captured += 1
                self._frames_dropped_processing += dropped
            if is_dummy_backend:
                fps = max(1.0, float(self._get_loop_fps()))
                self._stop_event.wait(1.0 / fps)


class _ProcessingWorker:
    def __init__(
        self,
        *,
        camera_id: str,
        udp_host: str,
        udp_port: int,
        frame_queue: queue.Queue[object],
        preview_worker: _PreviewWorker | None,
        log_fn: Callable[[str], None],
    ) -> None:
        self._camera_id = camera_id
        self._udp_host = udp_host
        self._udp_port = udp_port
        self._frame_queue = frame_queue
        self._preview_worker = preview_worker
        self._log = log_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._lock = threading.Lock()
        self._threshold = MASK_THRESHOLD
        self._min_diameter_px: float | None = None
        self._max_diameter_px: float | None = None
        self._circularity_min = DEFAULT_CIRCULARITY_MIN
        self._mask: np.ndarray | None = None
        self._state_label = STATE_IDLE
        self._stream_mode: str | None = None
        self._mask_build_request: _MaskBuildRequest | None = None
        self._intrinsics_session: _IntrinsicsCaptureSession | None = None
        self._last_detection_stats: dict[str, object] = {
            "threshold": self._threshold,
            "min_diameter_px": self._min_diameter_px,
            "max_diameter_px": self._max_diameter_px,
            "circularity_min": self._circularity_min,
            "raw_contour_count": 0,
            "accepted_blob_count": 0,
            "rejected_by_diameter": 0,
            "rejected_by_circularity": 0,
            "last_blob_count": 0,
        }
        self._last_timestamping_status: dict[str, object] = {
            "active_source": "capture_dequeue",
            "sensor_timestamp_available": False,
        }
        self._latest_preview_packet: _PreviewPacket | None = None
        self._frame_index = 0
        self._frames_processed = 0
        self._frames_sent = 0
        self._capture_to_process_ms: deque[float] = deque(maxlen=256)
        self._capture_to_send_ms: deque[float] = deque(maxlen=256)
        self._started_monotonic = 0.0

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._started_monotonic = time.monotonic()
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self._udp_host in ("255.255.255.255", "<broadcast>"):
                self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._lock:
            thread = self._thread
            sock = self._socket
            self._thread = None
            self._socket = None
            self._mask_build_request = None
            self._stream_mode = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if thread is not None:
            thread.join(timeout=1.0)

    def set_detection_settings(
        self,
        *,
        threshold: int,
        min_diameter_px: float | None,
        max_diameter_px: float | None,
        circularity_min: float,
    ) -> None:
        with self._lock:
            self._threshold = int(threshold)
            self._min_diameter_px = min_diameter_px
            self._max_diameter_px = max_diameter_px
            self._circularity_min = float(circularity_min)

    def set_mask(self, mask: np.ndarray | None) -> None:
        with self._lock:
            self._mask = mask.astype(bool, copy=False) if mask is not None else None

    def set_state_label(self, state_label: str) -> None:
        with self._lock:
            self._state_label = str(state_label)

    def set_stream_mode(self, mode: str | None) -> None:
        with self._lock:
            self._stream_mode = str(mode) if mode is not None else None

    def set_mask_build_request(self, request: _MaskBuildRequest | None) -> None:
        with self._lock:
            self._mask_build_request = request

    def set_intrinsics_session(self, session: _IntrinsicsCaptureSession | None) -> None:
        with self._lock:
            self._intrinsics_session = session

    def get_last_detection_stats(self) -> dict[str, object]:
        with self._lock:
            return dict(self._last_detection_stats)

    def get_last_timestamping_status(self) -> dict[str, object]:
        with self._lock:
            return dict(self._last_timestamping_status)

    def get_latest_preview_packet(self) -> _PreviewPacket | None:
        with self._lock:
            packet = self._latest_preview_packet
        if packet is None:
            return None
        return _PreviewPacket(
            frame=packet.frame.copy(),
            blobs=[{"x": float(blob["x"]), "y": float(blob["y"]), "area": float(blob["area"])} for blob in packet.blobs],
            mask=packet.mask.copy() if packet.mask is not None else None,
            stats=dict(packet.stats),
            extra_lines=tuple(packet.extra_lines),
        )

    def get_runtime_diagnostics(self) -> dict[str, object]:
        with self._lock:
            frames_processed = self._frames_processed
            frames_sent = self._frames_sent
            capture_to_process_ms = list(self._capture_to_process_ms)
            capture_to_send_ms = list(self._capture_to_send_ms)
            started_monotonic = self._started_monotonic
            stream_mode = self._stream_mode
        elapsed = max(1e-6, time.monotonic() - started_monotonic) if started_monotonic > 0.0 else 0.0
        processing_fps = float(frames_processed) / elapsed if elapsed > 0.0 else 0.0
        send_fps = float(frames_sent) / elapsed if elapsed > 0.0 else 0.0
        return {
            "processing_fps": processing_fps,
            "send_fps": send_fps,
            "capture_to_process_ms_p50": float(np.percentile(capture_to_process_ms, 50))
            if capture_to_process_ms
            else 0.0,
            "capture_to_process_ms_p90": float(np.percentile(capture_to_process_ms, 90))
            if capture_to_process_ms
            else 0.0,
            "capture_to_send_ms_p50": float(np.percentile(capture_to_send_ms, 50))
            if capture_to_send_ms
            else 0.0,
            "capture_to_send_ms_p90": float(np.percentile(capture_to_send_ms, 90))
            if capture_to_send_ms
            else 0.0,
            "stream_active": bool(stream_mode is not None),
        }

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                packet = cast(_FramePacket, self._frame_queue.get(timeout=0.1))
            except queue.Empty:
                continue

            with self._lock:
                threshold = self._threshold
                min_diameter_px = self._min_diameter_px
                max_diameter_px = self._max_diameter_px
                circularity_min = self._circularity_min
                mask = self._mask
                state_label = self._state_label
                stream_mode = self._stream_mode
                mask_build_request = self._mask_build_request
                intrinsics_session = self._intrinsics_session
                sock = self._socket

            if mask_build_request is not None:
                try:
                    mask_build_request.consume_frame(packet.captured_frame.image)
                except Exception as exc:  # noqa: BLE001
                    self._log(f"mask build failed: {exc}")

            if intrinsics_session is not None:
                try:
                    # Invariant: intrinsics detection always consumes the raw frame.
                    intrinsics_session.consume_frame(packet.captured_frame.image)
                except Exception as exc:  # noqa: BLE001
                    self._log(f"intrinsics frame consume failed: {exc}")

            applied_mask = None if state_label == STATE_MASK_INIT else mask
            # Invariant: blob detection always consumes the raw frame (not preview overlays).
            blobs, stats = detect_blobs(
                packet.captured_frame.image,
                threshold=threshold,
                mask=applied_mask,
                min_diameter_px=min_diameter_px,
                max_diameter_px=max_diameter_px,
                circularity_min=circularity_min,
            )

            processed_monotonic_ns = time.monotonic_ns()
            capture_to_process_ms = (
                float(processed_monotonic_ns - packet.captured_monotonic_ns) / 1_000_000.0
            )
            timestamping_status = {
                "active_source": str(packet.captured_frame.timestamp_source),
                "sensor_timestamp_available": bool(
                    packet.captured_frame.sensor_timestamp_ns is not None
                ),
            }
            with self._lock:
                self._last_detection_stats = dict(stats)
                self._last_timestamping_status = dict(timestamping_status)
                self._frames_processed += 1
                self._capture_to_process_ms.append(capture_to_process_ms)

            if self._preview_worker is not None:
                preview_frame, preview_blobs, preview_mask = _resize_preview_payload(
                    packet.captured_frame.image,
                    cast(list[dict[str, float]], blobs),
                    applied_mask,
                )
                extra_lines = [f"state={state_label}"]
                if stream_mode is not None:
                    extra_lines.append(f"mode={stream_mode}")
                preview_packet = _PreviewPacket(
                    frame=preview_frame,
                    blobs=preview_blobs,
                    mask=preview_mask,
                    stats=stats,
                    extra_lines=tuple(extra_lines),
                )
                with self._lock:
                    self._latest_preview_packet = preview_packet
                self._preview_worker.submit(preview_packet)
            else:
                preview_frame, preview_blobs, preview_mask = _resize_preview_payload(
                    packet.captured_frame.image,
                    cast(list[dict[str, float]], blobs),
                    applied_mask,
                )
                extra_lines = [f"state={state_label}"]
                if stream_mode is not None:
                    extra_lines.append(f"mode={stream_mode}")
                with self._lock:
                    self._latest_preview_packet = _PreviewPacket(
                        frame=preview_frame,
                        blobs=preview_blobs,
                        mask=preview_mask,
                        stats=stats,
                        extra_lines=tuple(extra_lines),
                    )

            if stream_mode is None or sock is None:
                continue

            msg: dict[str, object] = {
                "camera_id": self._camera_id,
                "timestamp": int(packet.captured_frame.timestamp_us),
                "timestamp_source": str(packet.captured_frame.timestamp_source),
                "frame_index": int(self._frame_index & 0xFFFFFFFF),
                "blobs": blobs,
                "capture_mode": stream_mode,
            }
            if stream_mode == "pose_capture":
                msg["blob_count"] = len(blobs)
                msg["quality"] = _pose_capture_quality(cast(list[dict[str, object]], blobs))
            self._frame_index = (self._frame_index + 1) & 0xFFFFFFFF

            payload = json.dumps(msg, separators=(",", ":")).encode("utf-8")
            try:
                _ = sock.sendto(payload, (self._udp_host, self._udp_port))
            except OSError:
                continue
            sent_monotonic_ns = time.monotonic_ns()
            with self._lock:
                self._frames_sent += 1
                self._capture_to_send_ms.append(
                    float(sent_monotonic_ns - packet.captured_monotonic_ns) / 1_000_000.0
                )


class _CapturePipeline:
    def __init__(
        self,
        *,
        camera_id: str,
        udp_host: str,
        udp_port: int,
        backend_factory: Callable[[], FrameBackend],
        debug_preview: DebugPreview | None,
        log_fn: Callable[[str], None],
    ) -> None:
        self._camera_id = camera_id
        self._backend_factory = backend_factory
        self._debug_preview = debug_preview
        self._log = log_fn
        self._lock = threading.Lock()
        self._active_event = threading.Event()
        self._frame_queue: queue.Queue[object] = queue.Queue(maxsize=PROCESSING_QUEUE_MAXSIZE)
        self._started = False
        self._stream_mode: str | None = None
        self._mask_build_active = False
        self._intrinsics_active = False
        self._mjpeg_render_enabled: bool = False
        self._desired_fps = float(DEFAULT_TARGET_FPS)
        self._desired_exposure_us: int | None = None
        self._desired_gain: float | None = None
        self._desired_focus: float | None = 5.215
        self._state_label = STATE_IDLE
        self._preview_worker = (
            _PreviewWorker(preview=debug_preview, camera_id=camera_id, log_fn=log_fn)
            if debug_preview is not None and debug_preview.enabled
            else None
        )
        self._processing_worker = _ProcessingWorker(
            camera_id=camera_id,
            udp_host=udp_host,
            udp_port=udp_port,
            frame_queue=self._frame_queue,
            preview_worker=self._preview_worker,
            log_fn=log_fn,
        )
        self._camera_worker = _CameraWorker(
            backend_factory=backend_factory,
            apply_backend_settings=self._apply_backend_settings,
            frame_queue=self._frame_queue,
            active_event=self._active_event,
            get_loop_fps=self._get_loop_fps,
            log_fn=log_fn,
        )

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
        if self._preview_worker is not None:
            self._preview_worker.start()
        try:
            self._processing_worker.start()
            self._camera_worker.start()
        except Exception:
            self._processing_worker.stop()
            if self._preview_worker is not None:
                self._preview_worker.stop()
            raise
        with self._lock:
            self._started = True
        self._refresh_activity()

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
        self._active_event.set()
        self._camera_worker.stop()
        self._processing_worker.stop()
        if self._preview_worker is not None:
            self._preview_worker.stop()

    def set_backend_controls(
        self,
        *,
        exposure_us: int | None = None,
        gain: float | None = None,
        fps: int | None = None,
        focus: float | None = None,
    ) -> None:
        with self._lock:
            if exposure_us is not None:
                self._desired_exposure_us = int(exposure_us)
            if gain is not None:
                self._desired_gain = float(gain)
            if fps is not None and int(fps) > 0:
                self._desired_fps = float(int(fps))
            if focus is not None:
                self._desired_focus = float(focus)
        if exposure_us is not None:
            self._camera_worker.apply_control(lambda backend: backend.set_exposure_us(int(exposure_us)))
        if gain is not None:
            self._camera_worker.apply_control(lambda backend: backend.set_gain(float(gain)))
        if fps is not None and int(fps) > 0:
            self._camera_worker.apply_control(lambda backend: backend.set_fps(int(fps)))
        if focus is not None:
            self._camera_worker.apply_control(lambda backend: backend.set_focus(float(focus)))

    def set_detection_settings(
        self,
        *,
        threshold: int,
        min_diameter_px: float | None,
        max_diameter_px: float | None,
        circularity_min: float,
    ) -> None:
        self._processing_worker.set_detection_settings(
            threshold=threshold,
            min_diameter_px=min_diameter_px,
            max_diameter_px=max_diameter_px,
            circularity_min=circularity_min,
        )

    def set_mask(self, mask: np.ndarray | None) -> None:
        self._processing_worker.set_mask(mask)

    def set_state_label(self, state_label: str) -> None:
        with self._lock:
            self._state_label = str(state_label)
        self._processing_worker.set_state_label(state_label)

    def start_stream(self, mode: str) -> None:
        with self._lock:
            self._stream_mode = str(mode)
        self._processing_worker.set_stream_mode(mode)
        self._refresh_activity()

    def stop_stream(self) -> None:
        with self._lock:
            self._stream_mode = None
        self._processing_worker.set_stream_mode(None)
        self._refresh_activity()

    def build_mask(
        self,
        *,
        frames: int,
        threshold: int,
        hit_ratio: float,
        min_area: int,
        dilate: int,
        timeout_s: float,
    ) -> tuple[np.ndarray, int]:
        request = _MaskBuildRequest(
            frames=frames,
            threshold=threshold,
            hit_ratio=hit_ratio,
            min_area=min_area,
            dilate=dilate,
            deadline_monotonic=time.perf_counter() + max(0.1, float(timeout_s)),
        )
        with self._lock:
            self._mask_build_active = True
        self._processing_worker.set_mask_build_request(request)
        self._refresh_activity()
        try:
            return request.wait(timeout=max(0.1, float(timeout_s) + 0.5))
        finally:
            self._processing_worker.set_mask_build_request(None)
            with self._lock:
                self._mask_build_active = False
            self._refresh_activity()

    def get_last_detection_stats(self) -> dict[str, object]:
        return self._processing_worker.get_last_detection_stats()

    def get_last_timestamping_status(self) -> dict[str, object]:
        return self._processing_worker.get_last_timestamping_status()

    def get_frame_snapshot(self) -> np.ndarray | None:
        return self._camera_worker.get_frame_snapshot()

    def get_preview_snapshot(self) -> _PreviewPacket | None:
        return self._processing_worker.get_latest_preview_packet()

    def set_mjpeg_render_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._mjpeg_render_enabled = bool(enabled)
        self._refresh_activity()

    def set_mjpeg_enabled(self, enabled: bool) -> None:
        self.set_mjpeg_render_enabled(enabled)

    def set_intrinsics_session(self, session: _IntrinsicsCaptureSession | None) -> None:
        with self._lock:
            self._intrinsics_active = session is not None and session.is_active()
        self._processing_worker.set_intrinsics_session(session)
        self._refresh_activity()

    def get_runtime_diagnostics(self) -> dict[str, object]:
        runtime = {}
        runtime.update(self._camera_worker.get_runtime_diagnostics())
        runtime.update(self._processing_worker.get_runtime_diagnostics())
        if self._preview_worker is not None:
            runtime.update(self._preview_worker.get_runtime_diagnostics())
            runtime["preview_queue_depth"] = self._preview_worker._queue.qsize()
        else:
            runtime["preview_fps"] = 0.0
            runtime["frames_dropped_preview"] = 0
            runtime["preview_queue_depth"] = 0
        runtime["processing_queue_depth"] = self._frame_queue.qsize()
        return runtime

    def debug_preview_active(self) -> bool:
        with self._lock:
            stream_mode = self._stream_mode
        return bool(
            stream_mode is not None
            or (self._preview_worker is not None and self._preview_worker.is_active())
        )

    def _apply_backend_settings(self, backend: FrameBackend) -> None:
        with self._lock:
            desired_exposure_us = self._desired_exposure_us
            desired_gain = self._desired_gain
            desired_fps = self._desired_fps
            desired_focus = self._desired_focus
        if desired_exposure_us is not None:
            backend.set_exposure_us(int(desired_exposure_us))
        if desired_gain is not None:
            backend.set_gain(float(desired_gain))
        if desired_fps > 0.0:
            backend.set_fps(int(round(desired_fps)))
        if desired_focus is not None:
            backend.set_focus(float(desired_focus))

    def _get_loop_fps(self) -> float:
        with self._lock:
            desired_fps = max(1.0, float(self._desired_fps))
            stream_mode = self._stream_mode
            mask_build_active = self._mask_build_active
            intrinsics_active = self._intrinsics_active
            preview_enabled = self._preview_worker is not None
            mjpeg_render_enabled = self._mjpeg_render_enabled
        if stream_mode is not None or mask_build_active or intrinsics_active:
            return desired_fps
        if preview_enabled or mjpeg_render_enabled:
            return min(desired_fps, IDLE_PREVIEW_FPS)
        return desired_fps

    def _refresh_activity(self) -> None:
        with self._lock:
            should_run = bool(
                self._stream_mode is not None
                or self._mask_build_active
                or self._intrinsics_active
                or self._preview_worker is not None
                or self._mjpeg_render_enabled
            )
        if should_run:
            self._active_event.set()
        else:
            self._active_event.clear()


@dataclass
class ControlServerConfig:
    camera_id: str = field(default_factory=get_default_camera_id)
    tcp_host: str = "0.0.0.0"
    tcp_port: int = 8554
    udp_dest: str = "255.255.255.255:5000"
    udp_host: str = "255.255.255.255"
    udp_port: int = 5000
    backend: str = field(default_factory=get_default_backend)
    target_fps: int = DEFAULT_TARGET_FPS
    threshold: int = 200
    mask_init_frames: int = MASK_INIT_FRAMES
    mask_threshold: int = MASK_THRESHOLD
    mask_hit_ratio: float = MASK_HIT_RATIO
    mask_min_area_px: int = MASK_MIN_AREA_PX
    mask_dilate_px: int = MASK_DILATE_PX
    mask_max_ratio_warning: float = MASK_MAX_RATIO_WARNING
    mask_init_timeout_s: float = MASK_INIT_TIMEOUT_SECONDS
    debug_preview: bool = False
    mjpeg_port: int = DEFAULT_MJPEG_PORT



def parse_args() -> ControlServerConfig:
    default_backend = get_default_backend()
    parser = argparse.ArgumentParser(description="Loutrack Pi capture service (MVP control only)")
    _ = parser.add_argument("--camera-id", default=None, help="Camera ID handled by this Pi (default: device name)")
    _ = parser.add_argument("--tcp-host", default="0.0.0.0", help="TCP bind host")
    _ = parser.add_argument("--tcp-port", type=int, default=8554, help="TCP bind port")
    _ = parser.add_argument(
        "--backend",
        choices=("dummy", "picamera2"),
        default=default_backend,
        help=(
            "Capture backend to use "
            f"(default: {default_backend}; picamera2 on Raspberry Pi, dummy elsewhere)"
        ),
    )
    _ = parser.add_argument(
        "--udp-dest",
        default="255.255.255.255:5000",
        help="UDP destination as host:port (one JSON per datagram)",
    )
    _ = parser.add_argument(
        "--debug-preview",
        action="store_true",
        help="Show OpenCV debug preview on the Pi during idle, mask init, and streaming",
    )
    _ = parser.add_argument(
        "--mjpeg-port",
        type=int,
        default=DEFAULT_MJPEG_PORT,
        help=f"Port for MJPEG preview stream (0 = disabled, default: {DEFAULT_MJPEG_PORT})",
    )

    namespace = parser.parse_args()
    camera_id = getattr(namespace, "camera_id", None)
    tcp_host = getattr(namespace, "tcp_host", "0.0.0.0")
    tcp_port = getattr(namespace, "tcp_port", 8554)
    backend = getattr(namespace, "backend", default_backend)
    udp_dest = getattr(namespace, "udp_dest", "255.255.255.255:5000")
    debug_preview = getattr(namespace, "debug_preview", False)
    mjpeg_port = getattr(namespace, "mjpeg_port", DEFAULT_MJPEG_PORT)

    if camera_id is None:
        camera_id = get_default_camera_id()
    if not isinstance(camera_id, str):
        raise SystemExit("--camera-id must be a string")
    camera_id = camera_id.strip()
    if not camera_id:
        raise SystemExit("--camera-id must not be empty")
    if not isinstance(tcp_host, str):
        raise SystemExit("--tcp-host must be a string")
    if not isinstance(tcp_port, int):
        raise SystemExit("--tcp-port must be an integer")
    if not isinstance(backend, str):
        raise SystemExit("--backend must be a string")
    if not isinstance(udp_dest, str):
        raise SystemExit("--udp-dest must be a string")
    if not isinstance(debug_preview, bool):
        raise SystemExit("--debug-preview must be a boolean flag")
    if not isinstance(mjpeg_port, int):
        raise SystemExit("--mjpeg-port must be an integer")
    if int(mjpeg_port) < 0 or int(mjpeg_port) > 65535:
        raise SystemExit("--mjpeg-port must be in range [0,65535]")

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
        debug_preview=resolve_debug_preview_enabled(debug_preview),
        mjpeg_port=int(mjpeg_port) if isinstance(mjpeg_port, int) else 0,
    )


def main() -> int:
    config = parse_args()
    from control_server import ControlServer
    server = ControlServer(config)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
    return 0


# Re-export extracted classes so existing imports from capture_runtime continue to work.
# These are deferred to avoid circular imports (control_server imports from capture_runtime).
def __getattr__(name: str) -> object:
    if name == "ControlServer":
        from control_server import ControlServer
        return ControlServer
    if name == "MJPEGStreamer":
        from mjpeg_streamer import MJPEGStreamer
        return MJPEGStreamer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


if __name__ == "__main__":
    raise SystemExit(main())
