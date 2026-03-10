from __future__ import annotations

import importlib.util
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


def test_build_initial_camera_poses_from_pair_graph() -> None:
    pair = init_mod.PairInitialization(
        camera_a="cam-a",
        camera_b="cam-b",
        rotation_ab=np.eye(3, dtype=np.float64),
        translation_ab=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        inlier_ratio=0.9,
        usable_points=20,
        angle_score=0.8,
    )
    poses, edges = init_mod.build_initial_camera_poses("cam-a", ["cam-a", "cam-b"], [pair])
    assert "cam-a" in poses and "cam-b" in poses
    assert np.allclose(poses["cam-a"][0], np.eye(3))
    assert len(edges) == 1
