"""
IntrinsicsCapture: MJPEG-based Charuco intrinsics calibration.

This module is kept as the alternate host-side intrinsics path. The production
GUI now prefers IntrinsicsHostSession, but tests and ad hoc workflows still use
this implementation directly.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CALIB_SRC_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class IntrinsicsConfig:
    camera_id: str
    mjpeg_url: str
    square_length_mm: float
    marker_length_mm: Optional[float] = None
    squares_x: int = 6
    squares_y: int = 8
    dictionary: str = "DICT_6X6_250"
    min_frames: int = 25
    target_frames: int = 50
    cooldown_s: float = 1.5
    spatial_threshold_px: float = 40.0
    output_dir: Path = field(default_factory=lambda: Path("calibration"))

    def __post_init__(self) -> None:
        if self.marker_length_mm is None:
            self.marker_length_mm = self.square_length_mm * 0.75


def _load_calibrate_module() -> Any:
    script_path = CALIB_SRC_ROOT / "camera-calibration" / "calibrate.py"
    spec = importlib.util.spec_from_file_location("_calibrate_intrinsics", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load calibrate module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["_calibrate_intrinsics"] = module
    spec.loader.exec_module(module)
    return module


class IntrinsicsCapture:
    """
    Streams MJPEG from a Pi, detects Charuco corners, applies 3-layer diversity
    filtering, and calibrates intrinsics via calibrate.py functions.
    """

    def __init__(self, config: IntrinsicsConfig) -> None:
        self._config = config
        self._lock = threading.Lock()
        self._phase: str = "idle"
        self._captured_corners: List[np.ndarray] = []
        self._captured_ids: List[np.ndarray] = []
        self._image_size: Optional[Tuple[int, int]] = None
        self._last_capture_time: float = 0.0
        self._last_captured_corners: Optional[np.ndarray] = None
        self._grid_coverage: np.ndarray = np.zeros((3, 3), dtype=int)
        self._latest_jpeg: Optional[bytes] = None
        self._calibration_result: Optional[Dict[str, Any]] = None
        self._last_error: Optional[str] = None
        self._rejected_cooldown: int = 0
        self._rejected_spatial: int = 0
        self._rejected_detection: int = 0
        self._stop_event = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        self._mjpeg_response: Any = None

        # Load calibrate.py module dynamically (picamera2 only imported inside run_live_capture)
        self._cal = _load_calibrate_module()

        # Setup Charuco board
        self._dictionary = self._cal.get_dictionary(config.dictionary)
        marker_len_mm = config.marker_length_mm if config.marker_length_mm is not None else config.square_length_mm * 0.75
        self._board = self._cal.create_charuco_board(
            squares_x=config.squares_x,
            squares_y=config.squares_y,
            square_length=config.square_length_mm / 1000.0,
            marker_length=marker_len_mm / 1000.0,
            dictionary=self._dictionary,
        )

    # ------------------------------------------------------------------
    # Public control API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start MJPEG polling and corner detection."""
        with self._lock:
            if self._phase == "capturing":
                return
            self._phase = "capturing"
            self._stop_event.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop(self) -> None:
        """Stop polling; captured frames are preserved."""
        with self._lock:
            if self._phase not in ("capturing",):
                return
            self._phase = "idle"
        self._stop_event.set()
        resp = self._mjpeg_response
        if resp is not None:
            try:
                resp.close()
            except Exception:
                pass
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=3.0)

    def clear(self) -> None:
        """Reset captured frames and grid coverage."""
        with self._lock:
            self._captured_corners = []
            self._captured_ids = []
            self._image_size = None
            self._last_capture_time = 0.0
            self._last_captured_corners = None
            self._grid_coverage = np.zeros((3, 3), dtype=int)
            self._rejected_cooldown = 0
            self._rejected_spatial = 0
            self._rejected_detection = 0
            self._calibration_result = None
            self._last_error = None

    def trigger_calibration(self) -> None:
        """Start background calibration from collected frames."""
        with self._lock:
            if len(self._captured_corners) < self._config.min_frames:
                raise ValueError(
                    f"Need at least {self._config.min_frames} frames, "
                    f"got {len(self._captured_corners)}"
                )
            corners = list(self._captured_corners)
            ids = list(self._captured_ids)
            image_size = self._image_size
            self._phase = "calibrating"
        if image_size is None:
            with self._lock:
                self._phase = "error"
                self._last_error = "no_image_size"
            return
        t = threading.Thread(
            target=self._run_calibration,
            args=(corners, ids, image_size),
            daemon=True,
        )
        t.start()

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "phase": self._phase,
                "camera_id": self._config.camera_id,
                "frames_captured": len(self._captured_corners),
                "frames_needed": self._config.min_frames,
                "frames_target": self._config.target_frames,
                "frames_rejected_cooldown": self._rejected_cooldown,
                "frames_rejected_spatial": self._rejected_spatial,
                "frames_rejected_detection": self._rejected_detection,
                "grid_coverage": self._grid_coverage.tolist(),
                "last_error": self._last_error,
                "calibration_result": self._calibration_result,
                "output_path": str(
                    self._config.output_dir
                    / f"calibration_intrinsics_v1_{self._config.camera_id}.json"
                ) if self._calibration_result else None,
            }

    def get_latest_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_jpeg

    # ------------------------------------------------------------------
    # Internal: MJPEG polling loop
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        response = None
        try:
            response = urllib.request.urlopen(self._config.mjpeg_url, timeout=10.0)
        except Exception as exc:
            with self._lock:
                self._last_error = f"connect_failed: {exc}"
                if self._phase == "capturing":
                    self._phase = "idle"
            return

        with self._lock:
            self._mjpeg_response = response

        # Parse boundary from Content-Type header
        content_type = response.headers.get("Content-Type", "")
        boundary = b""
        for part in content_type.split(";"):
            part = part.strip()
            if part.startswith("boundary="):
                boundary = part[len("boundary="):].encode()
                break

        try:
            while not self._stop_event.is_set():
                frame_bytes = self._read_mjpeg_frame(response, boundary)
                if frame_bytes is None:
                    break

                arr = np.frombuffer(frame_bytes, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is None:
                    continue

                corners, ids = self._cal.detect_charuco_corners(
                    img, self._board, self._dictionary
                )

                if corners is not None and ids is not None:
                    overlay = img.copy()
                    try:
                        cv2.aruco.drawDetectedCornersCharuco(overlay, corners, ids)
                    except Exception:
                        pass
                    ok, buf = cv2.imencode(
                        ".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 75]
                    )
                    if ok:
                        with self._lock:
                            self._latest_jpeg = buf.tobytes()
                    self._maybe_capture(img, corners, ids)
                else:
                    ok, buf = cv2.imencode(
                        ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60]
                    )
                    if ok:
                        with self._lock:
                            self._latest_jpeg = buf.tobytes()
                    with self._lock:
                        self._rejected_detection += 1

        except Exception as exc:
            with self._lock:
                if self._phase == "capturing":
                    self._last_error = f"stream_error: {exc}"
        finally:
            with self._lock:
                self._mjpeg_response = None
            try:
                response.close()
            except Exception:
                pass

    def _read_mjpeg_frame(self, response: Any, boundary: bytes) -> Optional[bytes]:
        """Read one JPEG chunk from the MJPEG multipart stream."""
        content_length = -1
        # Scan for Content-Length header, stop on blank line
        while True:
            try:
                line = response.readline()
            except Exception:
                return None
            if not line:
                return None
            stripped = line.strip()
            lower = stripped.lower()
            if lower.startswith(b"content-length:"):
                try:
                    content_length = int(stripped.split(b":", 1)[1].strip())
                except (ValueError, IndexError):
                    pass
            if not stripped:
                break

        if content_length > 0:
            try:
                data = response.read(content_length)
                _ = response.read(2)  # consume trailing \r\n
                return data
            except Exception:
                return None
        else:
            # Fallback: read until next boundary marker
            data = bytearray()
            while True:
                try:
                    line = response.readline()
                except Exception:
                    break
                if not line:
                    break
                if boundary and boundary in line:
                    break
                data.extend(line)
            result = bytes(data).strip()
            return result if result else None

    # ------------------------------------------------------------------
    # Internal: diversity filter & capture
    # ------------------------------------------------------------------

    def _maybe_capture(
        self,
        frame: np.ndarray,
        corners: np.ndarray,
        ids: np.ndarray,
    ) -> None:
        now = time.perf_counter()
        with self._lock:
            last_time = self._last_capture_time
            last_corners = self._last_captured_corners
            frames_captured = len(self._captured_corners)

        if frames_captured >= self._config.target_frames:
            return

        # Layer 1: time cooldown
        if now - last_time < self._config.cooldown_s:
            with self._lock:
                self._rejected_cooldown += 1
            return

        # Layer 2: spatial diversity
        if last_corners is not None:
            n = min(len(corners), len(last_corners))
            if n > 0:
                disp = float(
                    np.mean(
                        np.linalg.norm(
                            corners[:n].reshape(-1, 2)
                            - last_corners[:n].reshape(-1, 2),
                            axis=1,
                        )
                    )
                )
                if disp < self._config.spatial_threshold_px:
                    with self._lock:
                        self._rejected_spatial += 1
                    return

        # Layer 3: grid coverage tracking (3x3 cells)
        cx = min(2, int(np.mean(corners[:, 0, 0]) / frame.shape[1] * 3))
        cy = min(2, int(np.mean(corners[:, 0, 1]) / frame.shape[0] * 3))

        with self._lock:
            self._captured_corners.append(corners)
            self._captured_ids.append(ids)
            self._last_capture_time = now
            self._last_captured_corners = corners.copy()
            self._grid_coverage[cy, cx] += 1
            self._image_size = (frame.shape[1], frame.shape[0])

    # ------------------------------------------------------------------
    # Internal: calibration runner (background thread)
    # ------------------------------------------------------------------

    def _run_calibration(
        self,
        corners: List[np.ndarray],
        ids: List[np.ndarray],
        image_size: Tuple[int, int],
    ) -> None:
        try:
            rms, camera_matrix, dist_coeffs, rvecs, tvecs = (
                self._cal.calibrate_camera_charuco(corners, ids, self._board, image_size)
            )

            per_view_errors = self._cal.compute_per_view_errors(
                corners, ids, self._board, camera_matrix, dist_coeffs, rvecs, tvecs
            )

            total_points = sum(len(c) for c in corners)

            marker_len_mm = (
                self._config.marker_length_mm
                if self._config.marker_length_mm is not None
                else self._config.square_length_mm * 0.75
            )
            cal_config = self._cal.CalibrateConfig(
                camera=self._config.camera_id,
                output=None,
                square_length_mm=self._config.square_length_mm,
                marker_length_mm=marker_len_mm,
                squares_x=self._config.squares_x,
                squares_y=self._config.squares_y,
                dictionary=self._config.dictionary,
                input_dir=None,
                self_test=False,
                self_test_views=30,
                seed=0,
                min_frames=self._config.min_frames,
            )

            output = self._cal.build_output_json(
                camera_id=self._config.camera_id,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                rms_error=rms,
                image_size=image_size,
                config=cal_config,
                per_view_errors=per_view_errors,
                total_points=total_points,
                num_valid_frames=len(corners),
            )

            output_dir = Path(str(self._config.output_dir))
            output_dir.mkdir(parents=True, exist_ok=True)
            output_path = (
                output_dir
                / f"calibration_intrinsics_v1_{self._config.camera_id}.json"
            )
            output_path.write_text(
                json.dumps(output, indent=2) + "\n", encoding="utf-8"
            )

            with self._lock:
                self._calibration_result = output
                self._phase = "done"

        except Exception as exc:
            print(
                f"[intrinsics_capture] calibration failed: {exc}",
                file=sys.stderr,
                flush=True,
            )
            with self._lock:
                self._last_error = f"calibration_failed: {exc}"
                self._phase = "error"
