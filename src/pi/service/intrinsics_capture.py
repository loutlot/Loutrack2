#!/usr/bin/env python3
from __future__ import annotations

import importlib
import importlib.util
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast
import sys

import numpy as np

import os as _os, sys as _sys
_SERVICE_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _SERVICE_DIR not in _sys.path:
    _sys.path.insert(0, _SERVICE_DIR)
del _os, _sys

DEFAULT_CHARUCO_DICTIONARY = "DICT_6X6_250"
DEFAULT_CHARUCO_SQUARES_X = 6
DEFAULT_CHARUCO_SQUARES_Y = 8
DEFAULT_CHARUCO_SQUARE_LENGTH_MM = 30.0
DEFAULT_CHARUCO_MARKER_LENGTH_MM = 22.5
DEFAULT_INTRINSICS_TARGET_FRAMES = 50
DEFAULT_INTRINSICS_SPATIAL_THRESHOLD_PX = 40.0
CHARUCO_DETECTION_MIN_CORNERS = 6


def _intrinsics_calibrate_script_path() -> Path:
    return Path(__file__).resolve().parents[2] / "camera-calibration" / "calibrate.py"


def _load_intrinsics_calibrate_module() -> Any:
    script_path = _intrinsics_calibrate_script_path()
    spec = importlib.util.spec_from_file_location("_pi_intrinsics_calibrate", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed_to_load_calibrate_module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["_pi_intrinsics_calibrate"] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class _IntrinsicsCaptureConfig:
    camera_id: str
    square_length_mm: float
    marker_length_mm: float
    squares_x: int
    squares_y: int
    min_frames: int
    cooldown_s: float
    dictionary: str = DEFAULT_CHARUCO_DICTIONARY
    target_frames: int = DEFAULT_INTRINSICS_TARGET_FRAMES
    spatial_threshold_px: float = DEFAULT_INTRINSICS_SPATIAL_THRESHOLD_PX


class _IntrinsicsCaptureSession:
    def __init__(
        self,
        *,
        camera_id: str,
        log_fn: Callable[[str], None],
    ) -> None:
        self._camera_id = camera_id
        self._log = log_fn
        self._lock = threading.Lock()
        self._phase = "idle"
        self._config: _IntrinsicsCaptureConfig | None = None
        self._board: Any = None
        self._dictionary: Any = None
        self._cal_module: Any = None
        self._generation = 0
        self._captured_corners: list[np.ndarray] = []
        self._captured_ids: list[np.ndarray] = []
        self._image_size: tuple[int, int] | None = None
        self._last_capture_time = 0.0
        self._last_captured_corners: np.ndarray | None = None
        self._grid_coverage = np.zeros((3, 3), dtype=int)
        self._calibration_result: dict[str, object] | None = None
        self._last_error: str | None = None
        self._rejected_cooldown = 0
        self._rejected_spatial = 0
        self._rejected_detection = 0

    def start(self, config: _IntrinsicsCaptureConfig) -> None:
        marker_mm = float(config.marker_length_mm)
        square_mm = float(config.square_length_mm)
        if marker_mm <= 0.0 or square_mm <= 0.0:
            raise ValueError("invalid_request: intrinsics square/marker lengths must be > 0")
        if marker_mm >= square_mm:
            raise ValueError("invalid_request: intrinsics marker_length_mm must be smaller than square_length_mm")
        if int(config.squares_x) < 2 or int(config.squares_y) < 2:
            raise ValueError("invalid_request: intrinsics squares_x/squares_y must be >= 2")
        if int(config.min_frames) < 5:
            raise ValueError("invalid_request: intrinsics min_frames must be >= 5")
        if float(config.cooldown_s) <= 0.0:
            raise ValueError("invalid_request: intrinsics cooldown_s must be > 0")

        cal_module = self._ensure_cal_module()
        dictionary = cal_module.get_dictionary(config.dictionary)
        board = cal_module.create_charuco_board(
            squares_x=int(config.squares_x),
            squares_y=int(config.squares_y),
            square_length=float(config.square_length_mm) / 1000.0,
            marker_length=float(config.marker_length_mm) / 1000.0,
            dictionary=dictionary,
        )

        with self._lock:
            self._phase = "capturing"
            self._config = _IntrinsicsCaptureConfig(
                camera_id=str(config.camera_id),
                square_length_mm=float(config.square_length_mm),
                marker_length_mm=float(config.marker_length_mm),
                squares_x=int(config.squares_x),
                squares_y=int(config.squares_y),
                min_frames=int(config.min_frames),
                cooldown_s=float(config.cooldown_s),
                dictionary=str(config.dictionary),
                target_frames=int(config.target_frames),
                spatial_threshold_px=float(config.spatial_threshold_px),
            )
            self._board = board
            self._dictionary = dictionary
            self._generation += 1
            self._reset_capture_state_locked()

    def stop(self) -> None:
        with self._lock:
            if self._phase == "capturing":
                self._phase = "idle"

    def clear(self) -> None:
        with self._lock:
            self._reset_capture_state_locked()
            if self._phase in {"done", "error"}:
                self._phase = "idle"

    def report_runtime_error(self, message: str) -> None:
        with self._lock:
            self._last_error = str(message)

    def get_pending_corners(
        self,
        *,
        start_index: int = 0,
        max_frames: int | None = None,
    ) -> dict[str, object]:
        """Return buffered corner sets and image metadata; does not clear the buffer."""
        with self._lock:
            total_frames = len(self._captured_corners)
            safe_start = max(0, min(int(start_index), total_frames))
            safe_end = total_frames
            if max_frames is not None:
                safe_end = min(total_frames, safe_start + max(0, int(max_frames)))
            frames = [
                {
                    "corners": c.tolist(),
                    "ids": i.tolist(),
                }
                for c, i in zip(
                    self._captured_corners[safe_start:safe_end],
                    self._captured_ids[safe_start:safe_end],
                )
            ]
            image_size = list(self._image_size) if self._image_size else None
            return {
                "frames": frames,
                "count": total_frames,
                "start_index": safe_start,
                "returned_count": len(frames),
                "image_size": image_size,
                "phase": self._phase,
                "frames_captured": total_frames,
                "frames_rejected_cooldown": self._rejected_cooldown,
                "frames_rejected_spatial": self._rejected_spatial,
                "frames_rejected_detection": self._rejected_detection,
                "grid_coverage": self._grid_coverage.tolist(),
                "last_error": self._last_error,
            }

    def consume_frame(self, frame: np.ndarray) -> None:
        with self._lock:
            phase = self._phase
            config = self._config
            board = self._board
            dictionary = self._dictionary
            generation = self._generation

        if phase != "capturing" or config is None or board is None or dictionary is None:
            return

        cal_module = self._ensure_cal_module()
        corners, ids = cal_module.detect_charuco_corners(frame, board, dictionary)

        with self._lock:
            if generation != self._generation or self._phase != "capturing":
                return
            if len(self._captured_corners) >= int(config.target_frames):
                return
            if corners is None or ids is None or len(corners) < CHARUCO_DETECTION_MIN_CORNERS:
                self._rejected_detection += 1
                return

            now = time.perf_counter()
            if now - self._last_capture_time < float(config.cooldown_s):
                self._rejected_cooldown += 1
                return

            if self._last_captured_corners is not None:
                n = min(len(corners), len(self._last_captured_corners))
                if n > 0:
                    disp = float(
                        np.mean(
                            np.linalg.norm(
                                corners[:n].reshape(-1, 2) - self._last_captured_corners[:n].reshape(-1, 2),
                                axis=1,
                            )
                        )
                    )
                    if disp < float(config.spatial_threshold_px):
                        self._rejected_spatial += 1
                        return

            cx = min(2, int(np.mean(corners[:, 0, 0]) / max(1, frame.shape[1]) * 3.0))
            cy = min(2, int(np.mean(corners[:, 0, 1]) / max(1, frame.shape[0]) * 3.0))
            self._captured_corners.append(corners.copy())
            self._captured_ids.append(ids.copy())
            self._last_capture_time = now
            self._last_captured_corners = corners.copy()
            self._grid_coverage[cy, cx] += 1
            self._image_size = (int(frame.shape[1]), int(frame.shape[0]))

    def status_payload(self) -> dict[str, object]:
        with self._lock:
            config = self._config
            return {
                "phase": self._phase,
                "camera_id": self._camera_id,
                "frames_captured": len(self._captured_corners),
                "frames_needed": int(config.min_frames) if config is not None else 25,
                "frames_target": int(config.target_frames) if config is not None else DEFAULT_INTRINSICS_TARGET_FRAMES,
                "frames_rejected_cooldown": int(self._rejected_cooldown),
                "frames_rejected_spatial": int(self._rejected_spatial),
                "frames_rejected_detection": int(self._rejected_detection),
                "grid_coverage": self._grid_coverage.tolist(),
                "last_error": self._last_error,
                "calibration_result": self._calibration_result,
            }

    def is_active(self) -> bool:
        with self._lock:
            return self._phase == "capturing"

    def _reset_capture_state_locked(self) -> None:
        self._captured_corners = []
        self._captured_ids = []
        self._image_size = None
        self._last_capture_time = 0.0
        self._last_captured_corners = None
        self._grid_coverage = np.zeros((3, 3), dtype=int)
        self._calibration_result = None
        self._last_error = None
        self._rejected_cooldown = 0
        self._rejected_spatial = 0
        self._rejected_detection = 0

    def _ensure_cal_module(self) -> Any:
        with self._lock:
            module = self._cal_module
        if module is not None:
            return module
        module = _load_intrinsics_calibrate_module()
        with self._lock:
            if self._cal_module is None:
                self._cal_module = module
            return self._cal_module
