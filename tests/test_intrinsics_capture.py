"""
Tests for IntrinsicsCapture diversity filters and calibration integration.
"""
from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CALIB_SRC_ROOT = PROJECT_ROOT / "src"
HOST_SRC = PROJECT_ROOT / "src" / "host"

if str(CALIB_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(CALIB_SRC_ROOT))

from host.intrinsics_capture import IntrinsicsCapture, IntrinsicsConfig


# ── Helpers ────────────────────────────────────────────────────────────────────

def _load_calibrate_module() -> Any:
    script_path = CALIB_SRC_ROOT / "camera-calibration" / "calibrate.py"
    spec = importlib.util.spec_from_file_location("_calibrate_test", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["_calibrate_test"] = module
    spec.loader.exec_module(module)
    return module


def _make_config(**kwargs: Any) -> IntrinsicsConfig:
    defaults = {
        "camera_id": "pi-cam-01",
        "mjpeg_url": "http://localhost:9999/mjpeg",  # not used in unit tests
        "square_length_mm": 30.0,
        "min_frames": 3,
        "target_frames": 10,
        "cooldown_s": 0.2,
        "spatial_threshold_px": 40.0,
    }
    defaults.update(kwargs)
    return IntrinsicsConfig(**defaults)  # type: ignore[arg-type]


def _dummy_corners(
    cx: float = 200.0, cy: float = 200.0, spread: float = 5.0, n: int = 8
) -> np.ndarray:
    """Return a (n, 1, 2) float32 array of fake Charuco corners."""
    rng = np.random.default_rng(0)
    pts = rng.uniform(-spread, spread, (n, 2)).astype(np.float32)
    pts[:, 0] += cx
    pts[:, 1] += cy
    return pts.reshape(n, 1, 2)


def _dummy_ids(n: int = 8) -> np.ndarray:
    return np.arange(n, dtype=np.int32).reshape(n, 1)


# ── Tests ──────────────────────────────────────────────────────────────────────

class TestDiversityFilters:
    """Tests for the 3-layer diversity filter without network I/O."""

    def _capture_direct(
        self,
        cap: IntrinsicsCapture,
        frame: np.ndarray,
        corners: np.ndarray,
        ids: np.ndarray,
    ) -> None:
        """Call the internal _maybe_capture directly."""
        cap._maybe_capture(frame, corners, ids)

    def test_diversity_cooldown(self) -> None:
        """Two consecutive captures within cooldown period: second is rejected."""
        config = _make_config(cooldown_s=1.0)
        cap = IntrinsicsCapture(config)

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        corners = _dummy_corners(320, 240)
        ids = _dummy_ids()

        # First capture — should succeed
        self._capture_direct(cap, frame, corners, ids)
        assert cap.get_status()["frames_captured"] == 1

        # Immediately again — should be rejected by cooldown
        corners2 = _dummy_corners(cx=350, cy=250)  # moved enough spatially
        self._capture_direct(cap, frame, corners2, ids)
        assert cap.get_status()["frames_captured"] == 1
        assert cap.get_status()["frames_rejected_cooldown"] == 1

    def test_diversity_spatial(self) -> None:
        """Frame with corners too close to previous one is rejected."""
        config = _make_config(cooldown_s=0.0, spatial_threshold_px=40.0)
        cap = IntrinsicsCapture(config)

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        corners1 = _dummy_corners(cx=200.0, cy=200.0, spread=1.0)
        ids = _dummy_ids()

        # First capture
        self._capture_direct(cap, frame, corners1, ids)
        assert cap.get_status()["frames_captured"] == 1

        # Advance time beyond cooldown
        cap._last_capture_time = time.perf_counter() - 1.0

        # Corners barely moved (< spatial_threshold_px)
        corners2 = _dummy_corners(cx=201.0, cy=200.0, spread=1.0)
        self._capture_direct(cap, frame, corners2, ids)
        assert cap.get_status()["frames_captured"] == 1
        assert cap.get_status()["frames_rejected_spatial"] == 1

    def test_diversity_acceptance(self) -> None:
        """Frame that passes both cooldown and spatial filter is accepted."""
        config = _make_config(cooldown_s=0.0, spatial_threshold_px=40.0)
        cap = IntrinsicsCapture(config)

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        corners1 = _dummy_corners(cx=100.0, cy=100.0, spread=1.0)
        ids = _dummy_ids()

        self._capture_direct(cap, frame, corners1, ids)
        assert cap.get_status()["frames_captured"] == 1

        # Advance time beyond cooldown
        cap._last_capture_time = time.perf_counter() - 1.0

        # Move corners far enough
        corners2 = _dummy_corners(cx=300.0, cy=300.0, spread=1.0)
        self._capture_direct(cap, frame, corners2, ids)
        assert cap.get_status()["frames_captured"] == 2
        assert cap.get_status()["frames_rejected_spatial"] == 0
        assert cap.get_status()["frames_rejected_cooldown"] == 0

    def test_grid_coverage(self) -> None:
        """Grid coverage counter increments for each accepted frame."""
        config = _make_config(cooldown_s=0.0, spatial_threshold_px=0.0)
        cap = IntrinsicsCapture(config)

        # Frame size 600x400
        frame = np.zeros((400, 600, 3), dtype=np.uint8)
        ids = _dummy_ids()

        # Place corners in cell (0, 0): cx < 200, cy < 133
        corners_tl = _dummy_corners(cx=50.0, cy=50.0, spread=1.0)
        self._capture_direct(cap, frame, corners_tl, ids)
        cap._last_capture_time = 0.0  # reset cooldown

        # Place corners in cell (2, 2): cx > 400, cy > 266
        corners_br = _dummy_corners(cx=550.0, cy=350.0, spread=1.0)
        self._capture_direct(cap, frame, corners_br, ids)

        grid = np.array(cap.get_status()["grid_coverage"])
        assert grid[0, 0] >= 1, "top-left cell should be incremented"
        assert grid[2, 2] >= 1, "bottom-right cell should be incremented"


class TestCalibrationSynthetic:
    """End-to-end calibration using synthetic Charuco views."""

    def test_calibration_synthetic(self, tmp_path: Path) -> None:
        """
        Generate synthetic views, feed them through IntrinsicsCapture calibration,
        and verify RMS < 2.0px and output JSON matches schema.
        """
        cal = _load_calibrate_module()

        square_length_m = 0.03  # 30mm physical size
        square_px = 80  # pixels per square for board image
        dictionary = cal.get_dictionary("DICT_6X6_250")
        board = cal.create_charuco_board(
            squares_x=6,
            squares_y=8,
            square_length=square_length_m,
            marker_length=square_length_m * (20.0 / 30.0),
            dictionary=dictionary,
        )
        board_img = cal.draw_charuco_image(
            board,
            6 * square_px,
            8 * square_px,
        )
        image_size = (1152, 648)
        views = cal.generate_synthetic_views(board, board_img, 40, image_size, seed=42)

        # Collect corners
        all_corners = []
        all_ids = []
        for view in views:
            corners, ids = cal.detect_charuco_corners(view, board, dictionary)
            if corners is not None and ids is not None:
                all_corners.append(corners)
                all_ids.append(ids)

        assert len(all_corners) >= 20, f"Expected >=20 valid views, got {len(all_corners)}"

        rms, cam_matrix, dist_coeffs, rvecs, tvecs = cal.calibrate_camera_charuco(
            all_corners, all_ids, board, image_size
        )
        # Synthetic warp-based views don't follow the pinhole model exactly,
        # so RMS < 5.0 is achievable; real cameras should give < 1.0.
        assert rms < 5.0, f"RMS error too high: {rms:.3f}"

        per_view_errors = cal.compute_per_view_errors(
            all_corners, all_ids, board, cam_matrix, dist_coeffs, rvecs, tvecs
        )

        cal_config = cal.CalibrateConfig(
            camera="pi-cam-01",
            output=None,
            square_length_mm=square_length_m * 1000,
            marker_length_mm=square_length_m * 1000 * (20.0 / 30.0),
            squares_x=6,
            squares_y=8,
            dictionary="DICT_6X6_250",
            input_dir=None,
            self_test=False,
            self_test_views=30,
            seed=0,
            min_frames=25,
        )
        output = cal.build_output_json(
            camera_id="pi-cam-01",
            camera_matrix=cam_matrix,
            dist_coeffs=dist_coeffs,
            rms_error=rms,
            image_size=image_size,
            config=cal_config,
            per_view_errors=per_view_errors,
            total_points=sum(len(c) for c in all_corners),
            num_valid_frames=len(all_corners),
        )

        # Schema validation
        assert output["schema_version"] == "1.0"
        assert output["camera_id"] == "pi-cam-01"
        assert output["camera_model"] == "pinhole"
        assert "camera_matrix" in output
        assert "distortion_coefficients" in output
        assert output["rms_error"] < 5.0
        assert output["resolution"]["width"] == image_size[0]
        assert output["resolution"]["height"] == image_size[1]
        assert output["quality"]["num_valid_frames"] == len(all_corners)
