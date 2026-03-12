from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path
import sys

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


init_mod = _load_module("extrinsics_initializer", "src/camera-calibration/extrinsics_initializer.py")
samples_mod = _load_module("extrinsics_samples", "src/camera-calibration/extrinsics_samples.py")


@dataclass
class _Camera:
    intrinsic_matrix: np.ndarray
    distortion_coeffs: np.ndarray


def test_estimate_pairwise_initialization_uses_essential_inliers(monkeypatch) -> None:
    k = np.eye(3, dtype=np.float64)
    camera_params = {
        "cam-a": _Camera(k, np.zeros(5, dtype=np.float64)),
        "cam-b": _Camera(k, np.zeros(5, dtype=np.float64)),
    }
    samples = [
        samples_mod.PoseCaptureSample(
            sample_id=idx,
            timestamps={"cam-a": idx, "cam-b": idx},
            image_points_by_camera={
                "cam-a": np.array([0.1 * idx, 0.0], dtype=np.float64),
                "cam-b": np.array([0.1 * idx, 0.0], dtype=np.float64),
            },
            quality={"visible_camera_count": 2.0, "span_us": 0.0, "mean_observation_quality": 1.0, "parallax_proxy": 0.2},
        )
        for idx in range(8)
    ]

    def _find_essential(*args, **kwargs):
        return np.eye(3, dtype=np.float64), np.array([[1], [1], [1], [1], [0], [0], [0], [0]], dtype=np.uint8)

    captured = {}

    def _recover_pose(_essential, points_a, points_b):
        captured["count"] = len(points_a)
        return (
            int(len(points_a)),
            np.eye(3, dtype=np.float64),
            np.array([[1.0], [0.0], [0.0]], dtype=np.float64),
            np.ones((len(points_a), 1), dtype=np.uint8),
        )

    monkeypatch.setattr(init_mod.cv2, "findEssentialMat", _find_essential)
    monkeypatch.setattr(init_mod.cv2, "recoverPose", _recover_pose)
    rows = init_mod.estimate_pairwise_initialization(camera_params, samples, min_pairs=4)
    assert len(rows) == 0 or captured["count"] == 4


def test_build_initial_camera_poses_reports_disconnected_camera() -> None:
    pair = init_mod.PairInitialization(
        camera_a="cam-a",
        camera_b="cam-b",
        rotation_ab=np.eye(3, dtype=np.float64),
        translation_ab=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        inlier_ratio=0.9,
        usable_points=20,
        triangulation_angle_deg_p50=2.0,
    )
    poses, edges, excluded = init_mod.build_initial_camera_poses("cam-a", ["cam-a", "cam-b", "cam-c"], [pair])
    assert "cam-a" in poses and "cam-b" in poses
    assert len(edges) == 1
    assert excluded["cam-c"] == "no_connected_pairwise_path_from_reference"
