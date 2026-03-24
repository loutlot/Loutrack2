#!/usr/bin/env python3
from __future__ import annotations

import os as _os, sys as _sys
_SERVICE_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _SERVICE_DIR not in _sys.path:
    _sys.path.insert(0, _SERVICE_DIR)
del _os, _sys

import json
import math
import os
import re
import shutil
import socket
import subprocess
import threading
import time
from collections.abc import Callable, Mapping
from typing import Literal, cast

import cv2
import numpy as np

import capture_runtime as _capture_runtime_mod

from capture_runtime import (
    ControlServerConfig,
    # State constants
    STATE_IDLE,
    STATE_MASK_INIT,
    STATE_READY,
    STATE_RUNNING,
    # Mask constants
    MASK_INIT_FRAMES,
    MASK_THRESHOLD,
    MASK_HIT_RATIO,
    MASK_MIN_AREA_PX,
    MASK_DILATE_PX,
    MASK_MAX_RATIO_WARNING,
    MASK_INIT_TIMEOUT_SECONDS,
    # Capture constants
    DEFAULT_CIRCULARITY_MIN,
    DEFAULT_CAPTURE_WIDTH,
    DEFAULT_CAPTURE_HEIGHT,
    DEFAULT_TARGET_FPS,
    DEFAULT_MJPEG_PORT,
    # PTP constants
    PTP_SANITY_CACHE_US,
    PTP_LOCK_OFFSET_THRESHOLD_US,
    PTP_RO_SOCKET_PATH,
    LINUXPTP_ROLE_PATH,
    LINUXPTP_TIMESTAMPING_MODE_PATH,
    # Protocol constants
    MAX_LINE_BYTES,
    LINE_TIMEOUT_SECONDS,
    SCHEMA_COMMANDS,
    MVP_SUPPORTED_COMMANDS,
    # Error codes
    ERROR_INVALID_JSON,
    ERROR_INVALID_REQUEST,
    ERROR_UNKNOWN_CMD,
    ERROR_NOT_RUNNING,
    ERROR_ALREADY_RUNNING,
    ERROR_BACKEND_UNAVAILABLE,
    ERROR_INTERNAL,
    # Preview constants
    PREVIEW_STOP_TIMEOUT_SECONDS,
    PROCESSING_QUEUE_MAXSIZE,
    PREVIEW_QUEUE_MAXSIZE,
    IDLE_PREVIEW_FPS,
    # Classes
    _CapturePipeline,
    UDPFrameEmitter,
    FrameBackend,
    DummyBackend,
    Picamera2Backend,
    BackendUnavailableError,
    ClockSyncSnapshot,
    DebugPreview,
    _CharucoOverlayRenderer,
    PreviewRenderConfig,
    PreviewOverlayOptions,
    PreviewCharucoConfig,
    _MaskBuildRequest,
    # Functions
    _resize_preview_payload,
    _finalize_static_mask_from_hits,
    _build_placeholder_jpeg,
    _clock_realtime_us,
    _read_linuxptp_setting,
    _parse_pmc_time_status,
    _pose_capture_quality,
    _latest_queue_put,
    _render_preview_canvas,
    _encode_preview_jpeg,
)
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
from mjpeg_streamer import MJPEGStreamer
from blob_detection import detect_blobs


class ControlServer:
    def __init__(self, config: ControlServerConfig):
        self._config: ControlServerConfig = config
        self._state_lock: threading.Lock = threading.Lock()
        self._state: str = STATE_IDLE
        self._server_socket: socket.socket | None = None
        self._running: bool = False

        self._backend: FrameBackend | None = None
        self._udp_emitter: UDPFrameEmitter | None = None
        self._preview_backend: FrameBackend | None = None
        self._preview_thread: threading.Thread | None = None
        self._preview_stop_event: threading.Event | None = None
        self._preview_handoff_requested: bool = False
        self._preview_handoff_backend: FrameBackend | None = None
        self._preview_handoff_ready_event: threading.Event | None = None
        self._preview_resume_event: threading.Event | None = None
        self._pipeline: _CapturePipeline | None = None
        self._mjpeg: MJPEGStreamer | None = None

        self._desired_exposure_us: int | None = None
        self._desired_gain: float | None = None
        self._desired_fps: int | None = int(self._config.target_fps)
        self._desired_focus: float | None = 5.215
        self._desired_threshold: int = int(self._config.threshold)
        self._desired_blob_min_diameter_px: float | None = None
        self._desired_blob_max_diameter_px: float | None = None
        self._desired_circularity_min: float = DEFAULT_CIRCULARITY_MIN
        self._static_mask: np.ndarray | None = None
        self._mask_pixels: int = 0
        self._mask_ratio: float = 0.0
        self._mask_warning: str | None = None
        self._preview_render_config: PreviewRenderConfig = PreviewRenderConfig()
        self._charuco_renderer = _CharucoOverlayRenderer()
        self._intrinsics_session = _IntrinsicsCaptureSession(
            camera_id=self._config.camera_id,
            log_fn=self._log,
        )
        self._mjpeg_render_off_jpeg: bytes = _build_placeholder_jpeg("MJPEG render off")
        self._mjpeg_waiting_jpeg: bytes = _build_placeholder_jpeg("MJPEG waiting for frame")
        self._debug_preview: DebugPreview | None = (
            DebugPreview(log_fn=self._log) if self._config.debug_preview else None
        )
        self._last_blob_diagnostics: dict[str, object] = {
            "threshold": self._desired_threshold,
            "min_diameter_px": self._desired_blob_min_diameter_px,
            "max_diameter_px": self._desired_blob_max_diameter_px,
            "circularity_min": self._desired_circularity_min,
            "raw_contour_count": 0,
            "accepted_blob_count": 0,
            "rejected_by_diameter": 0,
            "rejected_by_circularity": 0,
            "last_blob_count": 0,
        }
        self._last_timestamping_diagnostics: dict[str, object] = {
            "active_source": "capture_dequeue",
            "sensor_timestamp_available": False,
        }
        self._clock_sync_cache_checked_at_us: int = 0
        self._clock_sync_cache: ClockSyncSnapshot = ClockSyncSnapshot(
            status="unknown",
            offset_us=None,
            source="unavailable",
        )

    def _log(self, message: str) -> None:
        print(f"[capture:{self._config.camera_id}] {message}", flush=True)

    def serve_forever(self) -> None:
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self._config.tcp_host, self._config.tcp_port))
        self._server_socket.listen()
        self._running = True
        self._log(
            "listening "
            f"tcp={self._config.tcp_host}:{self._config.tcp_port} "
            f"udp={self._config.udp_host}:{self._config.udp_port} "
            f"backend={self._config.backend} "
            f"resolution={DEFAULT_CAPTURE_WIDTH}x{DEFAULT_CAPTURE_HEIGHT} "
            f"fps={self._config.target_fps} "
            f"debug_preview={'on' if self._config.debug_preview else 'off'} "
            f"mjpeg_port={self._config.mjpeg_port}"
        )
        try:
            self._ensure_pipeline_started()
        except BackendUnavailableError as exc:
            self._log(f"capture pipeline unavailable: {exc}")
        except Exception as exc:  # noqa: BLE001
            self._log(f"capture pipeline start failed: {exc}")

        if self._config.mjpeg_port > 0:
            try:
                self._mjpeg = MJPEGStreamer(
                    port=self._config.mjpeg_port,
                    get_jpeg_fn=self._get_mjpeg_jpeg,
                )
                self._mjpeg.start()
                self._log(
                    "MJPEG streamer started "
                    f"port={self._config.mjpeg_port} render_default="
                    f"{'on' if self._preview_render_config.render_enabled else 'off'}"
                )
            except Exception as exc:  # noqa: BLE001
                self._log(f"mjpeg streamer start failed: {exc}")

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
        self._log("shutdown")
        if self._mjpeg is not None:
            self._mjpeg.stop()
            self._mjpeg = None
        pipeline = self._pipeline
        self._pipeline = None
        if pipeline is not None:
            pipeline.stop()
        if self._debug_preview is not None:
            self._debug_preview.close()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None

    def _ensure_pipeline_started(self) -> _CapturePipeline:
        with self._state_lock:
            pipeline = self._pipeline
        if pipeline is not None:
            return pipeline

        pipeline = _CapturePipeline(
            camera_id=self._config.camera_id,
            udp_host=self._config.udp_host,
            udp_port=self._config.udp_port,
            backend_factory=self._make_backend,
            debug_preview=self._debug_preview,
            log_fn=self._log,
        )
        pipeline.set_backend_controls(
            exposure_us=self._desired_exposure_us,
            gain=self._desired_gain,
            fps=self._desired_fps,
            focus=self._desired_focus,
        )
        pipeline.set_detection_settings(
            threshold=self._desired_threshold,
            min_diameter_px=self._desired_blob_min_diameter_px,
            max_diameter_px=self._desired_blob_max_diameter_px,
            circularity_min=self._desired_circularity_min,
        )
        pipeline.set_mask(self._static_mask)
        pipeline.set_state_label(self._state)
        pipeline.start()
        pipeline.set_mjpeg_render_enabled(self._preview_render_config.render_enabled)
        with self._state_lock:
            self._pipeline = pipeline
        return pipeline

    def _start_preview_loop(self) -> None:
        if not self._config.debug_preview:
            return

        thread_to_start: threading.Thread | None = None
        with self._state_lock:
            if not self._running or self._state in (STATE_RUNNING, STATE_MASK_INIT):
                return
            if self._preview_thread is not None and self._preview_thread.is_alive():
                if self._preview_backend is None and self._preview_resume_event is not None:
                    self._preview_resume_event.set()
                    self._log("preview loop resume requested")
                return
            stop_event = threading.Event()
            handoff_ready_event = threading.Event()
            resume_event = threading.Event()
            thread = threading.Thread(
                target=self._preview_loop,
                args=(stop_event,),
                daemon=True,
            )
            self._preview_stop_event = stop_event
            self._preview_handoff_ready_event = handoff_ready_event
            self._preview_resume_event = resume_event
            self._preview_thread = thread
            thread_to_start = thread

        if thread_to_start is not None:
            thread_to_start.start()

    def _stop_preview_loop(self, timeout_s: float = PREVIEW_STOP_TIMEOUT_SECONDS) -> bool:
        with self._state_lock:
            stop_event = self._preview_stop_event
            thread = self._preview_thread
            backend = self._preview_backend
            self._preview_stop_event = None
            resume_event = self._preview_resume_event

        if stop_event is not None:
            stop_event.set()
        if resume_event is not None:
            resume_event.set()
        if backend is not None:
            try:
                backend.stop()
            except Exception:
                pass
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout_s)))
        return thread is None or not thread.is_alive()

    def _preview_loop(self, stop_event: threading.Event) -> None:
        backend: FrameBackend | None = None
        preview_frame_index = 0
        try:
            next_tick = time.perf_counter()
            while self._running and not stop_event.is_set():
                if backend is None:
                    try:
                        backend = self._make_backend()
                        self._apply_backend_settings(backend)
                        backend.start()
                        self._log("preview backend started")
                    except BackendUnavailableError:
                        self._log("preview backend unavailable")
                        return
                    except Exception as exc:
                        self._log(f"preview backend start failed: {exc}")
                        return

                    with self._state_lock:
                        self._preview_backend = backend
                    preview_frame_index = 0
                    next_tick = time.perf_counter()

                with self._state_lock:
                    state_label = self._state
                    handoff_requested = self._preview_handoff_requested
                    threshold = self._desired_threshold
                    min_diameter_px = self._desired_blob_min_diameter_px
                    max_diameter_px = self._desired_blob_max_diameter_px
                    circularity_min = self._desired_circularity_min
                    mask = self._static_mask if self._static_mask is not None else None
                    fps = float(self._desired_fps or self._config.target_fps or IDLE_PREVIEW_FPS)

                if handoff_requested:
                    with self._state_lock:
                        self._preview_backend = None
                        self._preview_handoff_backend = backend
                        self._preview_handoff_requested = False
                        handoff_ready_event = self._preview_handoff_ready_event
                        resume_event = self._preview_resume_event
                    self._log("preview backend handed off for mask init")
                    if handoff_ready_event is not None:
                        handoff_ready_event.set()
                    backend = None
                    while self._running and not stop_event.is_set():
                        if resume_event is not None and resume_event.wait(0.1):
                            resume_event.clear()
                            self._log("preview loop resume acknowledged")
                            break
                    continue

                if state_label == STATE_RUNNING:
                    stop_event.wait(0.05)
                    continue

                frame_number = preview_frame_index + 1
                trace_frame = frame_number <= 3
                if trace_frame:
                    self._log(
                        f"preview loop frame {frame_number} capture begin state={state_label}"
                    )
                frame = backend.capture_array()
                if trace_frame:
                    self._log(
                        "preview loop frame "
                        f"{frame_number} capture ok shape={getattr(frame, 'shape', None)}"
                    )
                blobs, stats = detect_blobs(
                    frame,
                    threshold=threshold,
                    mask=mask,
                    min_diameter_px=min_diameter_px,
                    max_diameter_px=max_diameter_px,
                    circularity_min=circularity_min,
                )
                if trace_frame:
                    self._log(
                        "preview loop frame "
                        f"{frame_number} detect ok accepted={stats.get('accepted_blob_count', 0)}"
                    )
                self._record_blob_diagnostics(stats)
                if trace_frame:
                    self._log(f"preview loop frame {frame_number} show begin")
                self._show_debug_preview(
                    frame=frame,
                    blobs=blobs,
                    mask=mask,
                    stats=stats,
                    extra_lines=[
                        f"state={state_label}",
                        "preview=debug-idle",
                    ],
                )
                if trace_frame:
                    self._log(f"preview loop frame {frame_number} show ok")
                preview_frame_index = frame_number

                preview_fps = max(1.0, min(fps, IDLE_PREVIEW_FPS))
                next_tick += 1.0 / preview_fps
                wait_s = next_tick - time.perf_counter()
                if wait_s > 0.0:
                    stop_event.wait(wait_s)
                else:
                    next_tick = time.perf_counter()
        finally:
            if backend is not None:
                try:
                    backend.stop()
                except Exception:
                    pass
                self._log("preview backend stopped")
            with self._state_lock:
                self._preview_backend = None
                self._preview_handoff_requested = False
                self._preview_handoff_backend = None
                self._preview_handoff_ready_event = None
                self._preview_resume_event = None
                if self._preview_thread is threading.current_thread():
                    self._preview_thread = None
                    self._preview_stop_event = None

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
            return self._dispatch_command(request_id, request_camera_id, cmd, params)
        except Exception as exc:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: {exc}",
            )

    def _dispatch_command(
        self,
        request_id: str,
        request_camera_id: str,
        cmd: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        handlers: dict[str, Callable[[], dict[str, object]]] = {
            "ping": lambda: self._handle_ping(request_id, request_camera_id),
            "start": lambda: self._dispatch_start(request_id, request_camera_id, params),
            "stop": lambda: self._handle_stop(request_id, request_camera_id),
            "set_exposure": lambda: self._dispatch_set_int(
                request_id, request_camera_id, params, "set_exposure", self._set_exposure_us
            ),
            "set_gain": lambda: self._dispatch_set_number(
                request_id, request_camera_id, params, "set_gain", self._set_gain
            ),
            "set_fps": lambda: self._dispatch_set_int(
                request_id, request_camera_id, params, "set_fps", self._set_target_fps
            ),
            "set_focus": lambda: self._dispatch_set_number(
                request_id, request_camera_id, params, "set_focus", self._set_focus
            ),
            "set_threshold": lambda: self._dispatch_set_int(
                request_id, request_camera_id, params, "set_threshold", self._set_threshold
            ),
            "set_blob_diameter": lambda: self._dispatch_set_blob_diameter(
                request_id, request_camera_id, params
            ),
            "set_circularity_min": lambda: self._dispatch_set_circularity_min(
                request_id, request_camera_id, params
            ),
            "mask_start": lambda: self._handle_mask_start(request_id, request_camera_id, params),
            "mask_stop": lambda: self._handle_mask_stop(request_id, request_camera_id),
            "set_preview": lambda: self._dispatch_set_preview(request_id, request_camera_id, params),
            "intrinsics_start": lambda: self._dispatch_intrinsics_start(request_id, request_camera_id, params),
            "intrinsics_stop": lambda: self._dispatch_intrinsics_stop(request_id, request_camera_id, params),
            "intrinsics_clear": lambda: self._dispatch_intrinsics_clear(request_id, request_camera_id, params),
            "intrinsics_get_corners": lambda: self._dispatch_intrinsics_get_corners(request_id, request_camera_id, params),
            "intrinsics_status": lambda: self._dispatch_intrinsics_status(request_id, request_camera_id, params),
        }
        handler = handlers.get(cmd)
        if handler is None:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_UNKNOWN_CMD,
                error_message=f"unknown_cmd: {cmd}",
            )
        return handler()

    def _dispatch_start(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        mode = params.get("mode", "capture")
        if not isinstance(mode, str):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: start.mode must be string",
            )
        return self._handle_start(request_id, request_camera_id, mode, params)

    def _dispatch_set_int(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
        cmd_name: str,
        setter: Callable[[int], None],
    ) -> dict[str, object]:
        value = params.get("value")
        if not isinstance(value, int):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: {cmd_name}.value must be integer",
            )
        setter(value)
        return self._ok_response(request_id, request_camera_id)

    def _dispatch_set_number(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
        cmd_name: str,
        setter: Callable[[float], None],
    ) -> dict[str, object]:
        value = params.get("value")
        if not isinstance(value, (int, float)):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: {cmd_name}.value must be number",
            )
        setter(float(value))
        return self._ok_response(request_id, request_camera_id)

    def _dispatch_set_blob_diameter(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        min_px = params.get("min_px")
        max_px = params.get("max_px")
        if min_px is not None and not isinstance(min_px, (int, float)):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_blob_diameter.min_px must be number or null",
            )
        if max_px is not None and not isinstance(max_px, (int, float)):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_blob_diameter.max_px must be number or null",
            )
        min_value = float(min_px) if min_px is not None else None
        max_value = float(max_px) if max_px is not None else None
        if min_value is not None and min_value < 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_blob_diameter.min_px must be >= 0",
            )
        if max_value is not None and max_value <= 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_blob_diameter.max_px must be > 0",
            )
        if min_value is not None and max_value is not None and min_value > max_value:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_blob_diameter.min_px must be <= max_px",
            )
        self._set_blob_diameter(min_value, max_value)
        return self._ok_response(request_id, request_camera_id)

    def _dispatch_set_circularity_min(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        value = params.get("value")
        if not isinstance(value, (int, float)):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_circularity_min.value must be number",
            )
        circularity_value = float(value)
        if circularity_value < 0.0 or circularity_value > 1.0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: set_circularity_min.value must be in [0,1]",
            )
        self._set_circularity_min(circularity_value)
        return self._ok_response(request_id, request_camera_id)

    def _dispatch_set_preview(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        allowed_keys = {"render_enabled", "overlays", "charuco"}
        unknown_keys = sorted(set(params.keys()) - allowed_keys)
        if unknown_keys:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: set_preview unknown keys ({','.join(unknown_keys)})",
            )

        with self._state_lock:
            previous = self._preview_render_config

        render_enabled = previous.render_enabled
        overlays = previous.overlays
        charuco = previous.charuco

        if "render_enabled" in params:
            render_enabled_obj = params.get("render_enabled")
            if not isinstance(render_enabled_obj, bool):
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: set_preview.render_enabled must be boolean",
                )
            render_enabled = bool(render_enabled_obj)

        if "overlays" in params:
            overlays_obj = params.get("overlays")
            if not isinstance(overlays_obj, dict):
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: set_preview.overlays must be object",
                )
            overlay_allowed = {"blob", "mask", "text", "charuco"}
            overlay_unknown = sorted(set(overlays_obj.keys()) - overlay_allowed)
            if overlay_unknown:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message=(
                        "invalid_request: set_preview.overlays unknown keys "
                        f"({','.join(overlay_unknown)})"
                    ),
                )
            blob = overlays.blob
            mask = overlays.mask
            text = overlays.text
            charuco_overlay = overlays.charuco
            if "blob" in overlays_obj:
                if not isinstance(overlays_obj["blob"], bool):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.overlays.blob must be boolean",
                    )
                blob = bool(overlays_obj["blob"])
            if "mask" in overlays_obj:
                if not isinstance(overlays_obj["mask"], bool):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.overlays.mask must be boolean",
                    )
                mask = bool(overlays_obj["mask"])
            if "text" in overlays_obj:
                if not isinstance(overlays_obj["text"], bool):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.overlays.text must be boolean",
                    )
                text = bool(overlays_obj["text"])
            if "charuco" in overlays_obj:
                if not isinstance(overlays_obj["charuco"], bool):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.overlays.charuco must be boolean",
                    )
                charuco_overlay = bool(overlays_obj["charuco"])
            overlays = PreviewOverlayOptions(
                blob=blob,
                mask=mask,
                text=text,
                charuco=charuco_overlay,
            )

        if "charuco" in params:
            charuco_obj = params.get("charuco")
            if not isinstance(charuco_obj, dict):
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: set_preview.charuco must be object",
                )
            charuco_allowed = {
                "dictionary",
                "squares_x",
                "squares_y",
                "square_length_mm",
                "marker_length_mm",
            }
            charuco_unknown = sorted(set(charuco_obj.keys()) - charuco_allowed)
            if charuco_unknown:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message=(
                        "invalid_request: set_preview.charuco unknown keys "
                        f"({','.join(charuco_unknown)})"
                    ),
                )

            dictionary = charuco.dictionary
            squares_x = charuco.squares_x
            squares_y = charuco.squares_y
            square_length_mm = charuco.square_length_mm
            marker_length_mm = charuco.marker_length_mm

            if "dictionary" in charuco_obj:
                dictionary_obj = charuco_obj.get("dictionary")
                if not isinstance(dictionary_obj, str) or not dictionary_obj.strip():
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.charuco.dictionary must be non-empty string",
                    )
                dictionary = dictionary_obj.strip()
                if not self._charuco_renderer.is_dictionary_supported(dictionary):
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message=f"invalid_request: unsupported_charuco_dictionary ({dictionary})",
                    )
            if "squares_x" in charuco_obj:
                squares_x_obj = charuco_obj.get("squares_x")
                if not isinstance(squares_x_obj, int) or int(squares_x_obj) < 2:
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.charuco.squares_x must be integer >= 2",
                    )
                squares_x = int(squares_x_obj)
            if "squares_y" in charuco_obj:
                squares_y_obj = charuco_obj.get("squares_y")
                if not isinstance(squares_y_obj, int) or int(squares_y_obj) < 2:
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.charuco.squares_y must be integer >= 2",
                    )
                squares_y = int(squares_y_obj)
            if "square_length_mm" in charuco_obj:
                square_length_obj = charuco_obj.get("square_length_mm")
                if not isinstance(square_length_obj, (int, float)) or float(square_length_obj) <= 0.0:
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.charuco.square_length_mm must be > 0",
                    )
                square_length_mm = float(square_length_obj)
            if "marker_length_mm" in charuco_obj:
                marker_length_obj = charuco_obj.get("marker_length_mm")
                if not isinstance(marker_length_obj, (int, float)) or float(marker_length_obj) <= 0.0:
                    return self._error_response(
                        request_id=request_id,
                        request_camera_id=request_camera_id,
                        error_code=ERROR_INVALID_REQUEST,
                        error_message="invalid_request: set_preview.charuco.marker_length_mm must be > 0",
                    )
                marker_length_mm = float(marker_length_obj)
            if marker_length_mm >= square_length_mm:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message=(
                        "invalid_request: set_preview.charuco.marker_length_mm "
                        "must be smaller than square_length_mm"
                    ),
                )

            charuco = PreviewCharucoConfig(
                dictionary=dictionary,
                squares_x=squares_x,
                squares_y=squares_y,
                square_length_mm=square_length_mm,
                marker_length_mm=marker_length_mm,
            )

        if render_enabled and self._config.mjpeg_port <= 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: mjpeg_disabled",
            )

        updated_config = PreviewRenderConfig(
            render_enabled=render_enabled,
            overlays=overlays,
            charuco=charuco,
        )
        with self._state_lock:
            self._preview_render_config = updated_config
            pipeline = self._pipeline

        if updated_config.render_enabled:
            try:
                pipeline = self._ensure_pipeline_started()
                pipeline.set_mjpeg_render_enabled(True)
            except BackendUnavailableError as exc:
                with self._state_lock:
                    self._preview_render_config = previous
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_BACKEND_UNAVAILABLE,
                    error_message=f"backend_unavailable: {exc}",
                )
            except Exception as exc:  # noqa: BLE001
                with self._state_lock:
                    self._preview_render_config = previous
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INTERNAL,
                    error_message=f"internal_error: set_preview_failed: {exc}",
                )
        elif pipeline is not None:
            pipeline.set_mjpeg_render_enabled(False)

        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result={"preview": self._preview_config_payload()},
        )

    def _dispatch_intrinsics_start(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        allowed = {
            "camera_id",
            "square_length_mm",
            "marker_length_mm",
            "squares_x",
            "squares_y",
            "min_frames",
            "cooldown_s",
        }
        unknown = sorted(set(params.keys()) - allowed)
        if unknown:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: intrinsics_start unknown keys ({','.join(unknown)})",
            )

        camera_id_obj = params.get("camera_id", self._config.camera_id)
        if not isinstance(camera_id_obj, str) or camera_id_obj.strip() != self._config.camera_id:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=(
                    "invalid_request: intrinsics_start.camera_id must match this runtime "
                    f"({self._config.camera_id})"
                ),
            )

        square_raw = params.get("square_length_mm", DEFAULT_CHARUCO_SQUARE_LENGTH_MM)
        marker_raw = params.get("marker_length_mm")
        squares_x_raw = params.get("squares_x", DEFAULT_CHARUCO_SQUARES_X)
        squares_y_raw = params.get("squares_y", DEFAULT_CHARUCO_SQUARES_Y)
        min_frames_raw = params.get("min_frames", 25)
        cooldown_raw = params.get("cooldown_s", 1.5)

        if not isinstance(square_raw, (int, float)) or float(square_raw) <= 0.0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.square_length_mm must be > 0",
            )
        marker_value: float
        if marker_raw is None:
            marker_value = float(square_raw) * 0.75
        elif isinstance(marker_raw, (int, float)):
            marker_value = float(marker_raw)
        else:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.marker_length_mm must be number or null",
            )
        if marker_value <= 0.0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.marker_length_mm must be > 0",
            )
        if marker_value >= float(square_raw):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=(
                    "invalid_request: intrinsics_start.marker_length_mm "
                    "must be smaller than square_length_mm"
                ),
            )

        if not isinstance(squares_x_raw, int) or int(squares_x_raw) < 2:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.squares_x must be integer >= 2",
            )
        if not isinstance(squares_y_raw, int) or int(squares_y_raw) < 2:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.squares_y must be integer >= 2",
            )
        if not isinstance(min_frames_raw, int) or int(min_frames_raw) < 5:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.min_frames must be integer >= 5",
            )
        if not isinstance(cooldown_raw, (int, float)) or float(cooldown_raw) <= 0.0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_start.cooldown_s must be > 0",
            )

        try:
            pipeline = self._ensure_pipeline_started()
            config = _IntrinsicsCaptureConfig(
                camera_id=self._config.camera_id,
                square_length_mm=float(square_raw),
                marker_length_mm=float(marker_value),
                squares_x=int(squares_x_raw),
                squares_y=int(squares_y_raw),
                min_frames=int(min_frames_raw),
                cooldown_s=float(cooldown_raw),
            )
            self._intrinsics_session.start(config)
            pipeline.set_intrinsics_session(self._intrinsics_session)
        except BackendUnavailableError as exc:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_BACKEND_UNAVAILABLE,
                error_message=f"backend_unavailable: {exc}",
            )
        except ValueError as exc:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=str(exc),
            )
        except Exception as exc:  # noqa: BLE001
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: intrinsics_start_failed: {exc}",
            )
        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result=self._intrinsics_session.status_payload(),
        )

    def _dispatch_intrinsics_stop(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        if params:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_stop does not accept params",
            )
        self._intrinsics_session.stop()
        pipeline = self._pipeline
        if pipeline is not None:
            pipeline.set_intrinsics_session(None)
        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result=self._intrinsics_session.status_payload(),
        )

    def _dispatch_intrinsics_clear(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        if params:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_clear does not accept params",
            )
        self._intrinsics_session.clear()
        pipeline = self._pipeline
        if pipeline is not None:
            if self._intrinsics_session.is_active():
                pipeline.set_intrinsics_session(self._intrinsics_session)
            else:
                pipeline.set_intrinsics_session(None)
        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result=self._intrinsics_session.status_payload(),
        )

    def _dispatch_intrinsics_get_corners(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        allowed = {"start_index", "max_frames"}
        unknown = sorted(set(params.keys()) - allowed)
        if unknown:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: intrinsics_get_corners unknown keys ({','.join(unknown)})",
            )
        start_index_raw = params.get("start_index", 0)
        max_frames_raw = params.get("max_frames")
        if not isinstance(start_index_raw, int) or int(start_index_raw) < 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_get_corners.start_index must be integer >= 0",
            )
        if max_frames_raw is not None and (
            not isinstance(max_frames_raw, int) or int(max_frames_raw) <= 0
        ):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_get_corners.max_frames must be integer > 0",
            )
        result = self._intrinsics_session.get_pending_corners(
            start_index=int(start_index_raw),
            max_frames=int(max_frames_raw) if max_frames_raw is not None else None,
        )
        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result=result,
        )

    def _dispatch_intrinsics_status(
        self,
        request_id: str,
        request_camera_id: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        if params:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: intrinsics_status does not accept params",
            )
        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result=self._intrinsics_session.status_payload(),
        )

    def _handle_mask_start(self, request_id: str, request_camera_id: str, params: dict[str, object]) -> dict[str, object]:
        threshold = params.get("threshold", self._config.mask_threshold)
        frames = params.get("frames", self._config.mask_init_frames)
        seconds = params.get("seconds")
        hit_ratio = params.get("hit_ratio", self._config.mask_hit_ratio)
        min_area = params.get("min_area", self._config.mask_min_area_px)
        dilate = params.get("dilate", self._config.mask_dilate_px)

        if not isinstance(threshold, int) or threshold < 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: threshold must be non-negative integer",
            )
        if not isinstance(frames, int) or frames <= 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: frames must be positive integer",
            )
        if seconds is not None and (not isinstance(seconds, (int, float)) or float(seconds) <= 0.0):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: seconds must be positive number",
            )
        if not isinstance(hit_ratio, (int, float)) or not (0.0 < float(hit_ratio) <= 1.0):
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: hit_ratio must be in (0,1]",
            )
        if not isinstance(min_area, int) or min_area < 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: min_area must be non-negative integer",
            )
        if not isinstance(dilate, int) or dilate < 0:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message="invalid_request: dilate must be non-negative integer",
            )

        if seconds is not None:
            with self._state_lock:
                fps_for_mask = float(self._desired_fps or self._config.target_fps)
            frames = max(1, int(round(float(seconds) * fps_for_mask)))

        with self._state_lock:
            if self._state not in (STATE_IDLE, STATE_READY):
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: mask_start_not_allowed",
                )
            self._state = STATE_MASK_INIT

        self._log(f"mask init started frames={frames} threshold={threshold}")
        mask: np.ndarray | None = None
        mask_pixels = 0
        mask_ratio = 0.0
        warning: str | None = None
        try:
            pipeline = self._ensure_pipeline_started()
            pipeline.set_state_label(STATE_MASK_INIT)
            with self._state_lock:
                fps_for_timeout = float(self._desired_fps or self._config.target_fps or 1.0)
            estimated_capture_s = float(frames) / max(1.0, fps_for_timeout)
            mask_timeout_s = max(
                float(self._config.mask_init_timeout_s),
                estimated_capture_s * 4.0 + 2.0,
            )
            self._log(
                "mask init capture begin "
                f"frames={frames} timeout={mask_timeout_s:.1f}s hit_ratio={float(hit_ratio):.2f} "
                f"min_area={min_area} dilate={dilate}"
            )
            mask, mask_pixels = pipeline.build_mask(
                frames=frames,
                threshold=threshold,
                hit_ratio=float(hit_ratio),
                min_area=min_area,
                dilate=dilate,
                timeout_s=mask_timeout_s,
            )
            mask_ratio = mask_pixels / float(mask.size)
            self._log(
                "mask init capture finished "
                f"pixels={mask_pixels} ratio={mask_ratio:.3f}"
            )
            if mask_ratio > float(self._config.mask_max_ratio_warning):
                warning = f"mask_ratio_high ({mask_ratio:.3f})"
        except TimeoutError as exc:
            self._cancel_mask_state()
            if self._pipeline is not None:
                self._pipeline.set_mask(None)
                self._pipeline.set_state_label(STATE_IDLE)
            self._log(f"mask init timed out: {exc}")
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: mask_start_timeout: {exc}",
            )
        except BackendUnavailableError as exc:
            self._cancel_mask_state()
            if self._pipeline is not None:
                self._pipeline.set_mask(None)
                self._pipeline.set_state_label(STATE_IDLE)
            self._log(f"mask init backend unavailable: {exc}")
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_BACKEND_UNAVAILABLE,
                error_message=f"backend_unavailable: {exc}",
            )
        except Exception as exc:
            self._cancel_mask_state()
            if self._pipeline is not None:
                self._pipeline.set_mask(None)
                self._pipeline.set_state_label(STATE_IDLE)
            self._log(f"mask init failed: {exc}")
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: mask_start_failed: {exc}",
            )

        with self._state_lock:
            self._static_mask = mask
            self._mask_pixels = mask_pixels
            self._mask_ratio = mask_ratio
            self._mask_warning = warning
            self._state = STATE_READY
        if self._pipeline is not None:
            self._pipeline.set_mask(mask)
            self._pipeline.set_state_label(STATE_READY)
        self._log(
            "mask init completed "
            f"pixels={mask_pixels} ratio={mask_ratio:.3f}"
            + (f" warning={warning}" if warning else "")
        )

        result: dict[str, object] = {
            "mask_pixels": mask_pixels,
            "mask_ratio": mask_ratio,
            "mask_shape": list(mask.shape) if mask is not None else [],
            "mask_params": {
                "threshold": threshold,
                "frames": frames,
                "seconds": float(seconds) if seconds is not None else None,
                "hit_ratio": hit_ratio,
                "min_area": min_area,
                "dilate": dilate,
            },
        }
        if warning:
            result["mask_warning"] = warning
        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result=result,
        )

    def _handle_mask_stop(self, request_id: str, request_camera_id: str) -> dict[str, object]:
        with self._state_lock:
            if self._state not in (STATE_READY, STATE_IDLE):
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: mask_stop_not_allowed",
                )
            self._state = STATE_IDLE
            self._clear_mask()

        if self._pipeline is not None:
            self._pipeline.set_mask(None)
            self._pipeline.set_state_label(STATE_IDLE)

        return self._ok_response(
            request_id=request_id,
            request_camera_id=request_camera_id,
            result={"mask_cleared": True},
        )

    def _handle_start(
        self,
        request_id: str,
        camera_id: str,
        mode: str,
        params: dict[str, object],
    ) -> dict[str, object]:
        if mode not in ("capture", "pose_capture", "wand_metric_capture"):
            return self._error_response(
                request_id=request_id,
                request_camera_id=camera_id,
                error_code=ERROR_INVALID_REQUEST,
                error_message=f"invalid_request: unsupported_mode ({mode})",
            )

        mask_required = True

        with self._state_lock:
            prev_state = self._state
            if prev_state == STATE_RUNNING:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=camera_id,
                    error_code=ERROR_ALREADY_RUNNING,
                    error_message="already_running",
                )
            if prev_state == STATE_MASK_INIT:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: mask_initialization_in_progress",
                )
            if mask_required and prev_state != STATE_READY:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=camera_id,
                    error_code=ERROR_INVALID_REQUEST,
                    error_message="invalid_request: mask_required_for_mode",
                )
            self._state = STATE_RUNNING

        mask_to_apply = self._static_mask
        try:
            pipeline = self._ensure_pipeline_started()
            pipeline.set_mask(mask_to_apply)
            pipeline.set_state_label(STATE_RUNNING)
            pipeline.start_stream(mode)
            self._log(f"capture pipeline started mode={mode}")
        except BackendUnavailableError as exc:
            with self._state_lock:
                self._state = prev_state
            if self._pipeline is not None:
                self._pipeline.stop_stream()
                self._pipeline.set_state_label(prev_state)
            return self._error_response(
                request_id=request_id,
                request_camera_id=camera_id,
                error_code=ERROR_BACKEND_UNAVAILABLE,
                error_message=f"backend_unavailable: {exc}",
            )
        except Exception as exc:
            with self._state_lock:
                self._state = prev_state
            if self._pipeline is not None:
                self._pipeline.stop_stream()
                self._pipeline.set_state_label(prev_state)
            return self._error_response(
                request_id=request_id,
                request_camera_id=camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: udp_start_failed: {exc}",
            )

        result: dict[str, object] = {
            "mode": mode,
            "mask_active": mask_to_apply is not None,
            "mask_ratio": self._mask_ratio,
            "mask_warning": self._mask_warning,
        }
        return self._ok_response(
            request_id=request_id,
            request_camera_id=camera_id,
            result={k: v for k, v in result.items() if v is not None},
        )

    def _handle_stop(self, request_id: str, request_camera_id: str) -> dict[str, object]:
        with self._state_lock:
            if self._state != STATE_RUNNING:
                return self._error_response(
                    request_id=request_id,
                    request_camera_id=request_camera_id,
                    error_code=ERROR_NOT_RUNNING,
                    error_message="not_running",
                )

            self._state = STATE_IDLE

        try:
            pipeline = self._pipeline
            if pipeline is not None:
                self._record_blob_diagnostics(pipeline.get_last_detection_stats())
                self._record_timestamping_diagnostics(pipeline.get_last_timestamping_status())
                pipeline.stop_stream()
                self._log("capture pipeline stopped")
        except Exception as exc:
            return self._error_response(
                request_id=request_id,
                request_camera_id=request_camera_id,
                error_code=ERROR_INTERNAL,
                error_message=f"internal_error: udp_stop_failed: {exc}",
            )

        with self._state_lock:
            self._state = STATE_READY if self._static_mask is not None else STATE_IDLE
        if self._pipeline is not None:
            self._pipeline.set_state_label(self._state)

        return self._ok_response(request_id, request_camera_id)

    def _cancel_mask_state(self) -> None:
        with self._state_lock:
            self._state = STATE_IDLE
            self._clear_mask()

    def _clear_mask(self) -> None:
        self._static_mask = None
        self._mask_pixels = 0
        self._mask_ratio = 0.0
        self._mask_warning = None

    def _build_static_mask(
        self,
        backend: FrameBackend,
        frames: int,
        threshold: int,
        hit_ratio: float,
        min_area: int,
        dilate: int,
        deadline_monotonic: float | None = None,
    ) -> tuple[np.ndarray, int]:
        hit_counts: np.ndarray | None = None
        self._log(f"mask init frame loop begin frames={frames}")
        for frame_index in range(frames):
            if deadline_monotonic is not None and time.perf_counter() > deadline_monotonic:
                raise TimeoutError("mask_init_timed_out")
            self._log(f"mask init frame {frame_index + 1}/{frames} capture begin")
            frame = backend.capture_array()
            self._log(
                "mask init frame "
                f"{frame_index + 1}/{frames} capture ok shape={getattr(frame, 'shape', None)}"
            )
            gray = (
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if frame.ndim == 3
                else frame
            )
            if hit_counts is None:
                hit_counts = np.zeros(gray.shape, dtype=np.uint16)
                self._log(
                    "mask init accumulator allocated "
                    f"shape={gray.shape} dtype={hit_counts.dtype}"
                )
            hit_counts += (gray > threshold).astype(np.uint16)
            self._log(f"mask init frame {frame_index + 1}/{frames} accumulated")

        assert hit_counts is not None
        self._log("mask init postprocess begin")
        required = max(1, math.ceil(hit_ratio * frames))
        self._log(f"mask init postprocess required_hits={required}")
        mask_uint8 = (hit_counts >= required).astype(np.uint8) * 255

        if min_area > 0:
            contours_info = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
            self._log(f"mask init postprocess contours={len(contours)}")
            for contour in contours:
                if cv2.contourArea(contour) < min_area:
                    cv2.drawContours(mask_uint8, [contour], -1, 0, thickness=-1)

        if dilate > 0:
            kernel_size = max(1, dilate) * 2 + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            mask_uint8 = cv2.dilate(mask_uint8, kernel)
            self._log(f"mask init postprocess dilate kernel={kernel_size}")

        mask = mask_uint8.astype(bool)
        self._log(f"mask init postprocess complete pixels={int(np.count_nonzero(mask))}")
        return mask, int(np.count_nonzero(mask))

    def _apply_backend_settings(self, backend: FrameBackend) -> None:
        if self._desired_exposure_us is not None:
            backend.set_exposure_us(int(self._desired_exposure_us))
        if self._desired_gain is not None:
            backend.set_gain(float(self._desired_gain))
        if self._desired_fps is not None:
            backend.set_fps(int(self._desired_fps))
        if self._desired_focus is not None:
            backend.set_focus(float(self._desired_focus))

    def _record_blob_diagnostics(self, stats: dict[str, object]) -> None:
        with self._state_lock:
            self._last_blob_diagnostics = dict(stats)

    def _record_timestamping_diagnostics(self, stats: dict[str, object]) -> None:
        with self._state_lock:
            self._last_timestamping_diagnostics = {
                "active_source": str(stats.get("active_source", "capture_dequeue")),
                "sensor_timestamp_available": bool(stats.get("sensor_timestamp_available", False)),
            }

    def _preview_config_payload(self) -> dict[str, object]:
        with self._state_lock:
            config = self._preview_render_config
        return {
            "render_enabled": bool(config.render_enabled),
            "overlays": config.overlays.to_dict(),
            "charuco": config.charuco.to_dict(),
        }

    def _get_mjpeg_jpeg(self) -> bytes | None:
        with self._state_lock:
            render_config = self._preview_render_config
            pipeline = self._pipeline
        if not render_config.render_enabled:
            return self._mjpeg_render_off_jpeg
        if pipeline is None:
            return self._mjpeg_waiting_jpeg

        snapshot = pipeline.get_preview_snapshot()
        if snapshot is None:
            return self._mjpeg_waiting_jpeg
        # Overlay rendering is display-only. Detection paths consume raw frames upstream.
        canvas = _render_preview_canvas(
            frame=snapshot.frame,
            blobs=snapshot.blobs,
            mask=snapshot.mask,
            stats=snapshot.stats,
            camera_id=self._config.camera_id,
            extra_lines=list(snapshot.extra_lines),
            overlays=render_config.overlays,
            charuco_renderer=self._charuco_renderer,
            charuco_config=render_config.charuco,
        )
        encoded = _encode_preview_jpeg(canvas, quality=75)
        if encoded is None:
            return self._mjpeg_waiting_jpeg
        return encoded

    def _show_debug_preview(
        self,
        *,
        frame: np.ndarray,
        blobs: list[dict[str, float]],
        mask: np.ndarray | None,
        stats: dict[str, object],
        extra_lines: list[str] | None = None,
    ) -> None:
        preview = self._debug_preview
        if preview is None or not preview.enabled:
            return
        preview.show(
            frame=frame,
            blobs=blobs,
            mask=mask,
            stats=stats,
            camera_id=self._config.camera_id,
            extra_lines=extra_lines,
        )

    def _set_target_fps(self, fps: int) -> None:
        fps_value = int(fps)
        if fps_value <= 0:
            return
        with self._state_lock:
            self._config.target_fps = fps_value
            self._desired_fps = fps_value
            pipeline = self._pipeline
        if pipeline is not None:
            pipeline.set_backend_controls(fps=fps_value)

    def _set_exposure_us(self, value_us: int) -> None:
        value = int(value_us)
        with self._state_lock:
            self._desired_exposure_us = value
            pipeline = self._pipeline
        if pipeline is not None:
            pipeline.set_backend_controls(exposure_us=value)

    def _set_gain(self, value: float) -> None:
        gain_value = float(value)
        with self._state_lock:
            self._desired_gain = gain_value
            pipeline = self._pipeline
        if pipeline is not None:
            pipeline.set_backend_controls(gain=gain_value)

    def _set_focus(self, value: float) -> None:
        focus_value = float(value)
        with self._state_lock:
            self._desired_focus = focus_value
            pipeline = self._pipeline
        if pipeline is not None:
            pipeline.set_backend_controls(focus=focus_value)

    def _set_threshold(self, value: int) -> None:
        threshold_value = max(0, min(255, int(value)))
        with self._state_lock:
            self._desired_threshold = threshold_value
            pipeline = self._pipeline
            min_diameter_px = self._desired_blob_min_diameter_px
            max_diameter_px = self._desired_blob_max_diameter_px
            circularity_min = self._desired_circularity_min
        if pipeline is not None:
            pipeline.set_detection_settings(
                threshold=threshold_value,
                min_diameter_px=min_diameter_px,
                max_diameter_px=max_diameter_px,
                circularity_min=circularity_min,
            )

    def _set_blob_diameter(
        self,
        min_diameter_px: float | None,
        max_diameter_px: float | None,
    ) -> None:
        with self._state_lock:
            self._desired_blob_min_diameter_px = min_diameter_px
            self._desired_blob_max_diameter_px = max_diameter_px
            pipeline = self._pipeline
            threshold = self._desired_threshold
            circularity_min = self._desired_circularity_min
        if pipeline is not None:
            pipeline.set_detection_settings(
                threshold=threshold,
                min_diameter_px=min_diameter_px,
                max_diameter_px=max_diameter_px,
                circularity_min=circularity_min,
            )

    def _set_circularity_min(self, value: float) -> None:
        circularity_value = max(0.0, min(1.0, float(value)))
        with self._state_lock:
            self._desired_circularity_min = circularity_value
            pipeline = self._pipeline
            threshold = self._desired_threshold
            min_diameter_px = self._desired_blob_min_diameter_px
            max_diameter_px = self._desired_blob_max_diameter_px
        if pipeline is not None:
            pipeline.set_detection_settings(
                threshold=threshold,
                min_diameter_px=min_diameter_px,
                max_diameter_px=max_diameter_px,
                circularity_min=circularity_value,
            )

    def _handle_ping(self, request_id: str, request_camera_id: str) -> dict[str, object]:
        with self._state_lock:
            diagnostics = {
                "state": self._state,
                "exposure_us": self._desired_exposure_us,
                "gain": self._desired_gain,
                "fps": self._desired_fps,
                "focus": self._desired_focus,
                "threshold": self._desired_threshold,
                "blob_min_diameter_px": self._desired_blob_min_diameter_px,
                "blob_max_diameter_px": self._desired_blob_max_diameter_px,
                "circularity_min": self._desired_circularity_min,
                "mask_ratio": self._mask_ratio,
                "mask_pixels": self._mask_pixels,
                "mask_warning": self._mask_warning,
                "debug_preview_enabled": bool(self._debug_preview is not None and self._debug_preview.enabled),
            }
            pipeline = self._pipeline
            blob_diagnostics = dict(self._last_blob_diagnostics)
            timestamping = dict(self._last_timestamping_diagnostics)
        if pipeline is None:
            try:
                pipeline = self._ensure_pipeline_started()
            except BackendUnavailableError:
                pipeline = None
            except Exception:
                pipeline = None
        preview_payload = self._preview_config_payload()
        diagnostics["mjpeg_server_enabled"] = bool(self._config.mjpeg_port > 0)
        diagnostics["mjpeg_render_enabled"] = bool(preview_payload.get("render_enabled", False))
        diagnostics["preview_overlays"] = preview_payload.get("overlays", {})
        diagnostics["charuco_config"] = preview_payload.get("charuco", {})
        diagnostics["debug_preview_active"] = bool(
            (pipeline is not None and pipeline.debug_preview_active())
            or (self._debug_preview is not None and self._debug_preview.window_open)
        )
        diagnostics["blob_diagnostics"] = (
            pipeline.get_last_detection_stats() if pipeline is not None else blob_diagnostics
        )
        diagnostics["clock_sync"] = self._get_clock_sync_diagnostics()
        diagnostics["timestamping"] = (
            pipeline.get_last_timestamping_status() if pipeline is not None else timestamping
        )
        diagnostics["runtime"] = (
            pipeline.get_runtime_diagnostics() if pipeline is not None else {
                "capture_fps": 0.0,
                "processing_fps": 0.0,
                "preview_fps": 0.0,
                "processing_queue_depth": 0,
                "preview_queue_depth": 0,
                "frames_dropped_processing": 0,
                "frames_dropped_preview": 0,
                "capture_to_process_ms_p50": 0.0,
                "capture_to_process_ms_p90": 0.0,
                "capture_to_send_ms_p50": 0.0,
                "capture_to_send_ms_p90": 0.0,
                "send_fps": 0.0,
                "stream_active": False,
            }
        )
        diagnostics["supported_commands"] = sorted(MVP_SUPPORTED_COMMANDS)
        return self._ok_response(request_id, request_camera_id, result=diagnostics)

    def _get_clock_sync_diagnostics(self) -> dict[str, object]:
        import sys as _sys
        _cr_mod = _sys.modules.get("src.pi.service.capture_runtime", _capture_runtime_mod)
        now_us = _cr_mod._clock_realtime_us()
        with self._state_lock:
            cache_age_us = now_us - self._clock_sync_cache_checked_at_us
            cached = self._clock_sync_cache
        if self._clock_sync_cache_checked_at_us > 0 and cache_age_us < PTP_SANITY_CACHE_US:
            return {
                "status": str(cached.status),
                "offset_us": cached.offset_us,
                "source": str(cached.source),
                "role": str(cached.role),
                "timestamping_mode": str(cached.timestamping_mode),
            }
        snapshot = self._probe_clock_sync()
        with self._state_lock:
            self._clock_sync_cache_checked_at_us = now_us
            self._clock_sync_cache = snapshot
        return {
            "status": str(snapshot.status),
            "offset_us": snapshot.offset_us,
            "source": str(snapshot.source),
            "role": str(snapshot.role),
            "timestamping_mode": str(snapshot.timestamping_mode),
        }

    def _probe_clock_sync(self) -> ClockSyncSnapshot:
        role = cast(
            Literal["master", "slave", "unknown"],
            _read_linuxptp_setting(LINUXPTP_ROLE_PATH, {"master", "slave"}),
        )
        timestamping_mode = cast(
            Literal["software", "hardware", "unknown"],
            _read_linuxptp_setting(LINUXPTP_TIMESTAMPING_MODE_PATH, {"software", "hardware"}),
        )
        if shutil.which("pmc") is None:
            return ClockSyncSnapshot(
                status="unknown",
                offset_us=None,
                source="unavailable",
                role=role,
                timestamping_mode=timestamping_mode,
            )
        client_socket_path = f"/tmp/pmc.{os.getpid()}.{threading.get_ident()}"
        try:
            completed = subprocess.run(
                [
                    "pmc",
                    "-u",
                    "-b",
                    "0",
                    "-s",
                    PTP_RO_SOCKET_PATH,
                    "-i",
                    client_socket_path,
                    "GET TIME_STATUS_NP",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except (OSError, subprocess.SubprocessError, TimeoutError):
            return ClockSyncSnapshot(
                status="unknown",
                offset_us=None,
                source="unavailable",
                role=role,
                timestamping_mode=timestamping_mode,
            )
        finally:
            try:
                os.unlink(client_socket_path)
            except OSError:
                pass
        if completed.returncode != 0:
            return ClockSyncSnapshot(
                status="unknown",
                offset_us=None,
                source="unavailable",
                role=role,
                timestamping_mode=timestamping_mode,
            )
        return _parse_pmc_time_status(
            completed.stdout,
            role=role,
            timestamping_mode=timestamping_mode,
        )

    def _ensure_backend(self) -> FrameBackend:
        with self._state_lock:
            backend = self._backend

        if backend is not None:
            return backend

        return self._make_backend()

    def _take_preview_backend_for_mask(self) -> FrameBackend | None:
        with self._state_lock:
            preview_thread = self._preview_thread
            preview_backend = self._preview_backend
            if preview_thread is None or preview_backend is None or not preview_thread.is_alive():
                self._log("preview handoff unavailable")
                return None
            handoff_ready_event = self._preview_handoff_ready_event
            resume_event = self._preview_resume_event
            self._preview_handoff_requested = True
            self._preview_handoff_backend = None
            if handoff_ready_event is not None:
                handoff_ready_event.clear()
            if resume_event is not None:
                resume_event.clear()
        self._log("preview handoff requested")

        if handoff_ready_event is not None:
            handoff_ready_event.wait(timeout=PREVIEW_STOP_TIMEOUT_SECONDS)

        with self._state_lock:
            backend = self._preview_handoff_backend
            self._preview_handoff_backend = None
            if backend is not None:
                self._log("preview handoff acquired")
                return backend
            self._preview_handoff_requested = False
        self._log("preview handoff failed")
        return None

    def _make_backend(self) -> FrameBackend:
        with self._state_lock:
            backend_name = str(self._config.backend)
        if backend_name == "dummy":
            return DummyBackend()
        if backend_name == "picamera2":
            return Picamera2Backend()
        raise BackendUnavailableError(f"unknown_backend: {backend_name}")

    def _ok_response(
        self,
        request_id: str,
        request_camera_id: str,
        result: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        response_camera_id = (
            self._config.camera_id if request_camera_id == "broadcast" else request_camera_id
        )
        response: dict[str, object] = {
            "request_id": request_id,
            "camera_id": response_camera_id,
            "ack": True,
        }
        if result is not None:
            response["result"] = result
        return response

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
