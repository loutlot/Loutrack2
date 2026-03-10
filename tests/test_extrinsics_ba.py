from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path
import sys

import cv2
import numpy as np


def _load_module(name: str, relative: str):
    path = Path(__file__).resolve().parents[1] / relative
    src_root = str(Path(__file__).resolve().parents[1] / "src")
    calib_root = str(Path(__file__).resolve().parents[1] / "src" / "camera-calibration")
    if src_root not in sys.path:
        sys.path.insert(0, src_root)
    if calib_root not in sys.path:
        sys.path.insert(0, calib_root)
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {relative}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


ba_mod = _load_module("extrinsics_ba", "src/camera-calibration/extrinsics_ba.py")
samples_mod = _load_module("extrinsics_samples", "src/camera-calibration/extrinsics_samples.py")


@dataclass
class _Camera:
    intrinsic_matrix: np.ndarray
    distortion_coeffs: np.ndarray


def test_pose_bundle_adjustment_runs_on_synthetic_data() -> None:
    k = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    cam_params = {
        "cam-a": _Camera(k, np.zeros(5, dtype=np.float64)),
        "cam-b": _Camera(k, np.zeros(5, dtype=np.float64)),
    }
    rot_b, _ = cv2.Rodrigues(np.array([0.0, 0.15, 0.0], dtype=np.float64))
    t_b = np.array([0.3, 0.0, 0.0], dtype=np.float64)

    samples = []
    for idx in range(8):
        point = np.array([-0.1 + idx * 0.03, 0.02 * idx, 2.0], dtype=np.float64)
        p1, _ = cv2.projectPoints(point.reshape(1, 3), np.zeros((3, 1)), np.zeros((3,)), k, np.zeros(5))
        rvec_b, _ = cv2.Rodrigues(rot_b)
        p2, _ = cv2.projectPoints(point.reshape(1, 3), rvec_b, t_b, k, np.zeros(5))
        samples.append(
            samples_mod.PoseCaptureSample(
                sample_id=idx,
                timestamps={"cam-a": 1000 + idx, "cam-b": 1000 + idx},
                image_points_by_camera={
                    "cam-a": p1.reshape(2),
                    "cam-b": p2.reshape(2),
                },
                quality={"visible_camera_count": 2.0, "span_us": 0.0, "mean_observation_quality": 1.0},
            )
        )

    result = ba_mod.run_pose_bundle_adjustment(
        camera_params=cam_params,
        reference_camera_id="cam-a",
        samples=samples,
        camera_poses_init={
            "cam-a": (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)),
            "cam-b": (rot_b, t_b),
        },
    )
    assert result.iterations > 0
    assert "cam-b" in result.camera_poses
