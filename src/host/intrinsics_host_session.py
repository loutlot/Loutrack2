"""
IntrinsicsHostSession: hybrid architecture for intrinsics calibration.

Pi detects ChArUco corners on raw frames and applies diversity filters.
Host polls corners/ids via TCP, then runs calibration locally.
This avoids JPEG compression artifacts that degrade cornerSubPix accuracy.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import threading
import time
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CALIB_SRC_ROOT = Path(__file__).resolve().parents[1]
REMOTE_INTRINSICS_FETCH_BATCH = 8


def _load_calibrate_module() -> Any:
    script_path = CALIB_SRC_ROOT / "camera-calibration" / "calibrate.py"
    spec = importlib.util.spec_from_file_location("_calibrate_intrinsics_host", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load calibrate module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["_calibrate_intrinsics_host"] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class IntrinsicsHostSessionConfig:
    camera_id: str
    mjpeg_url: str
    square_length_mm: float
    marker_length_mm: float
    squares_x: int
    squares_y: int
    dictionary: str = "DICT_6X6_250"
    min_frames: int = 25
    target_frames: int = 50
    poll_interval_s: float = 1.5
    output_dir: Path = field(default_factory=lambda: Path("calibration"))


class IntrinsicsHostSession:
    """
    Poll corners/ids from Pi over TCP and run calibration on the Host.

    Pi-side diversity filters (cooldown, spatial, grid) ensure only high-quality,
    varied frames are buffered. Because corner detection runs on raw Pi frames,
    JPEG compression artifacts do not affect calibration accuracy.
    """

    def __init__(
        self,
        config: IntrinsicsHostSessionConfig,
        broadcast_fn: Callable[..., Dict[str, Any]],
    ) -> None:
        self._config = config
        self._broadcast_fn = broadcast_fn
        self._lock = threading.Lock()
        self._phase: str = "idle"
        self._captured_corners: List[np.ndarray] = []
        self._captured_ids: List[np.ndarray] = []
        self._image_size: Optional[Tuple[int, int]] = None
        self._grid_coverage: np.ndarray = np.zeros((3, 3), dtype=int)
        self._calibration_result: Optional[Dict[str, Any]] = None
        self._last_error: Optional[str] = None
        self._rejected_cooldown: int = 0
        self._rejected_spatial: int = 0
        self._rejected_detection: int = 0
        self._stop_event = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        self._latest_jpeg: Optional[bytes] = None
        self._cal: Any = None
        self._board: Any = None
        self._dictionary: Any = None

    # ------------------------------------------------------------------
    # Public control API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Send intrinsics_start to Pi and begin polling for corners."""
        self._broadcast_fn(
            "intrinsics_start",
            square_length_mm=self._config.square_length_mm,
            marker_length_mm=self._config.marker_length_mm,
            squares_x=self._config.squares_x,
            squares_y=self._config.squares_y,
            min_frames=self._config.min_frames,
            cooldown_s=self._config.poll_interval_s,
        )
        with self._lock:
            self._phase = "capturing"
            self._stop_event.clear()
        self._poll_once()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop(self) -> None:
        """Send intrinsics_stop to Pi and stop polling; captured frames are preserved."""
        self._poll_once()
        self._stop_event.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=3.0)
        try:
            self._broadcast_fn("intrinsics_stop")
        except Exception:
            pass
        with self._lock:
            if self._phase == "capturing":
                self._phase = "idle"

    def clear(self) -> None:
        """Send intrinsics_clear to Pi and reset local buffer."""
        try:
            self._broadcast_fn("intrinsics_clear")
        except Exception:
            pass
        with self._lock:
            self._captured_corners = []
            self._captured_ids = []
            self._image_size = None
            self._grid_coverage = np.zeros((3, 3), dtype=int)
            self._calibration_result = None
            self._last_error = None
            self._rejected_cooldown = 0
            self._rejected_spatial = 0
            self._rejected_detection = 0

    def trigger_calibration(self) -> None:
        """Start background calibration from collected frames."""
        self.sync_remote_frames()
        with self._lock:
            local_count = len(self._captured_corners)
            if local_count < self._config.min_frames:
                remote_count = self._get_remote_frames_captured()
                raise ValueError(
                    f"Need at least {self._config.min_frames} frames, "
                    f"got {local_count}"
                    + (f" (remote reports {remote_count})" if remote_count is not None else "")
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
            jpeg = self._latest_jpeg
        if jpeg is not None:
            return jpeg
        mjpeg_url = self._config.mjpeg_url
        if not mjpeg_url:
            return None
        return self._fetch_single_mjpeg_frame(mjpeg_url)

    # ------------------------------------------------------------------
    # Internal: polling loop
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        while not self._stop_event.is_set():
            self._stop_event.wait(self._config.poll_interval_s)
            if self._stop_event.is_set():
                break
            self._poll_once()

    def sync_remote_frames(self) -> None:
        self._poll_once()

    def _get_remote_frames_captured(self) -> int | None:
        try:
            payload = self._broadcast_fn("intrinsics_status")
        except Exception:
            return None
        count = payload.get("frames_captured")
        if isinstance(count, (int, float)):
            return int(count)
        return None

    def _poll_once(self) -> None:
        try:
            payload = self._broadcast_fn(
                "intrinsics_get_corners",
                start_index=len(self._captured_corners),
                max_frames=REMOTE_INTRINSICS_FETCH_BATCH,
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            return

        self._apply_remote_payload(payload)
        total_count = int(payload.get("count", len(payload.get("frames", []))))
        while len(self._captured_corners) < total_count:
            try:
                payload = self._broadcast_fn(
                    "intrinsics_get_corners",
                    start_index=len(self._captured_corners),
                    max_frames=REMOTE_INTRINSICS_FETCH_BATCH,
                )
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                return
            returned_count = int(payload.get("returned_count", len(payload.get("frames", []))))
            self._apply_remote_payload(payload)
            if returned_count <= 0:
                break

    def _apply_remote_payload(self, payload: Dict[str, Any]) -> None:
        frames = payload.get("frames", [])
        image_size_raw = payload.get("image_size")
        last_error_raw = payload.get("last_error")
        phase_raw = payload.get("phase")

        with self._lock:
            for f in frames:
                c = np.array(f["corners"], dtype=np.float32)
                i = np.array(f["ids"], dtype=np.int32)
                self._captured_corners.append(c)
                self._captured_ids.append(i)
            if self._image_size is None and image_size_raw:
                self._image_size = (int(image_size_raw[0]), int(image_size_raw[1]))
            if isinstance(phase_raw, str) and phase_raw.strip():
                self._phase = phase_raw
            if isinstance(last_error_raw, str):
                self._last_error = last_error_raw
            elif last_error_raw is None:
                self._last_error = None
            self._rejected_cooldown = int(
                payload.get("frames_rejected_cooldown", self._rejected_cooldown)
            )
            self._rejected_spatial = int(
                payload.get("frames_rejected_spatial", self._rejected_spatial)
            )
            self._rejected_detection = int(
                payload.get("frames_rejected_detection", self._rejected_detection)
            )
            grid_raw = payload.get("grid_coverage")
            if grid_raw is not None:
                self._grid_coverage = np.array(grid_raw, dtype=int)

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
            cal = self._ensure_cal_module()
            board, dictionary = self._ensure_board(cal)

            rms, camera_matrix, dist_coeffs, rvecs, tvecs = cal.calibrate_camera_charuco(
                corners, ids, board, image_size
            )
            per_view_errors = cal.compute_per_view_errors(
                corners, ids, board, camera_matrix, dist_coeffs, rvecs, tvecs
            )
            total_points = sum(len(c) for c in corners)
            cal_config = cal.CalibrateConfig(
                camera=self._config.camera_id,
                output=None,
                square_length_mm=self._config.square_length_mm,
                marker_length_mm=self._config.marker_length_mm,
                squares_x=self._config.squares_x,
                squares_y=self._config.squares_y,
                dictionary=self._config.dictionary,
                input_dir=None,
                self_test=False,
                self_test_views=30,
                seed=0,
                min_frames=self._config.min_frames,
            )
            output = cal.build_output_json(
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
            output_path = output_dir / f"calibration_intrinsics_v1_{self._config.camera_id}.json"
            output_path.write_text(
                json.dumps(output, indent=2) + "\n", encoding="utf-8"
            )

            with self._lock:
                self._calibration_result = output
                self._phase = "done"
                self._last_error = None

        except Exception as exc:
            print(
                f"[intrinsics_host_session] calibration failed: {exc}",
                file=sys.stderr,
                flush=True,
            )
            with self._lock:
                self._last_error = f"calibration_failed: {exc}"
                self._phase = "error"

    def _ensure_cal_module(self) -> Any:
        if self._cal is not None:
            return self._cal
        self._cal = _load_calibrate_module()
        return self._cal

    def _ensure_board(self, cal: Any) -> tuple[Any, Any]:
        if self._board is not None:
            return self._board, self._dictionary
        dictionary = cal.get_dictionary(self._config.dictionary)
        board = cal.create_charuco_board(
            squares_x=self._config.squares_x,
            squares_y=self._config.squares_y,
            square_length=self._config.square_length_mm / 1000.0,
            marker_length=self._config.marker_length_mm / 1000.0,
            dictionary=dictionary,
        )
        self._board = board
        self._dictionary = dictionary
        return board, dictionary

    # ------------------------------------------------------------------
    # Internal: single MJPEG frame fetch for preview
    # ------------------------------------------------------------------

    def _fetch_single_mjpeg_frame(self, url: str) -> Optional[bytes]:
        try:
            with urllib.request.urlopen(url, timeout=1.5) as response:
                content_type = str(response.headers.get("Content-Type", ""))
                boundary = b""
                for part in content_type.split(";"):
                    token = part.strip()
                    if token.startswith("boundary="):
                        boundary = token.split("=", 1)[1].encode("utf-8")
                        break

                content_length = -1
                while True:
                    line = response.readline()
                    if not line:
                        return None
                    stripped = line.strip()
                    if stripped.lower().startswith(b"content-length:"):
                        try:
                            content_length = int(stripped.split(b":", 1)[1].strip())
                        except Exception:
                            content_length = -1
                    if stripped == b"":
                        break

                if content_length > 0:
                    data = response.read(content_length)
                    _ = response.read(2)
                    return bytes(data) if data else None

                data = bytearray()
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    line = response.readline()
                    if not line:
                        break
                    if boundary and boundary in line:
                        break
                    data.extend(line)
                frame = bytes(data).strip()
                return frame if frame else None
        except Exception:
            return None
