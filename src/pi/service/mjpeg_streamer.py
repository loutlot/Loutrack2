#!/usr/bin/env python3
from __future__ import annotations

import sys
import threading
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer as _ThreadingHTTPServer

import os as _os, sys as _sys
_SERVICE_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _SERVICE_DIR not in _sys.path:
    _sys.path.insert(0, _SERVICE_DIR)
del _os, _sys


class MJPEGStreamer:
    """
    Serves a low-framerate MJPEG preview stream over HTTP for GUI-side intrinsics
    calibration.  Runs two daemon threads: one encodes frames, one serves HTTP.
    """

    def __init__(
        self,
        *,
        port: int,
        fps: float = 2.0,
        get_jpeg_fn: Callable[[], bytes | None],
    ) -> None:
        self._port = port
        self._fps = fps
        self._get_jpeg_fn = get_jpeg_fn
        self._latest_jpeg: bytes | None = None
        self._active_clients = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def start(self) -> None:
        t_encode = threading.Thread(target=self._encode_loop, daemon=True)
        t_serve = threading.Thread(target=self._serve_loop, daemon=True)
        t_encode.start()
        t_serve.start()

    def stop(self) -> None:
        self._stop.set()

    def _get_latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest_jpeg

    def active_client_count(self) -> int:
        with self._lock:
            return int(self._active_clients)

    def has_clients(self) -> bool:
        return self.active_client_count() > 0

    def _increment_active_clients(self) -> None:
        with self._lock:
            self._active_clients += 1

    def _decrement_active_clients(self) -> None:
        with self._lock:
            self._active_clients = max(0, self._active_clients - 1)

    def _encode_loop(self) -> None:
        while not self._stop.is_set():
            if not self.has_clients():
                self._stop.wait(1.0 / max(0.1, self._fps))
                continue
            try:
                jpeg = self._get_jpeg_fn()
                with self._lock:
                    self._latest_jpeg = bytes(jpeg) if jpeg is not None else None
            except Exception:
                pass
            self._stop.wait(1.0 / max(0.1, self._fps))

    def _serve_loop(self) -> None:
        streamer = self

        class _MJPEGHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path != "/mjpeg":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.end_headers()
                interval = 1.0 / max(0.1, streamer._fps)
                streamer._increment_active_clients()
                try:
                    while not streamer._stop.is_set():
                        jpeg = streamer._get_latest_jpeg()
                        if jpeg is not None:
                            header = (
                                b"--frame\r\n"
                                b"Content-Type: image/jpeg\r\n"
                                b"Content-Length: "
                                + str(len(jpeg)).encode()
                                + b"\r\n\r\n"
                            )
                            self.wfile.write(header + jpeg + b"\r\n")
                            self.wfile.flush()
                        streamer._stop.wait(interval)
                except Exception:
                    pass
                finally:
                    streamer._decrement_active_clients()

            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                pass

        try:
            server = _ThreadingHTTPServer(("0.0.0.0", self._port), _MJPEGHandler)
            server.timeout = 0.5
            while not self._stop.is_set():
                server.handle_request()
            server.server_close()
        except Exception as exc:
            print(f"[mjpeg] serve_loop failed: {exc}", file=sys.stderr, flush=True)
