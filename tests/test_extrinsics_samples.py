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


capture_mod = _load_module("extrinsics_capture", "src/camera-calibration/extrinsics_capture.py")
samples_mod = _load_module("extrinsics_samples", "src/camera-calibration/extrinsics_samples.py")


def _obs(camera_id: str, timestamp: int, x: float, quality: float = 1.0) -> object:
    return capture_mod.PoseCaptureObservation(
        camera_id,
        timestamp,
        timestamp,
        np.array([x, 10.0], dtype=np.float64),
        10.0,
        1,
        quality,
    )


def test_build_pose_multiview_samples_prefers_min_global_span() -> None:
    obs = {
        "cam-a": [_obs("cam-a", 1000, 10.0)],
        "cam-b": [_obs("cam-b", 950, 12.0), _obs("cam-b", 1005, 13.0)],
        "cam-c": [_obs("cam-c", 1002, 8.0)],
    }
    rows = samples_mod.build_pose_multiview_samples(obs, "cam-a", pair_window_us=100)
    assert len(rows) == 1
    assert rows[0].timestamps["cam-b"] == 1005
    assert rows[0].timestamps["cam-c"] == 1002


def test_build_pose_multiview_samples_does_not_drift_after_bad_match() -> None:
    obs = {
        "cam-a": [_obs("cam-a", 1000, 10.0), _obs("cam-a", 2000, 11.0)],
        "cam-b": [_obs("cam-b", 1005, 12.0), _obs("cam-b", 1055, 30.0), _obs("cam-b", 2005, 13.0)],
    }
    rows = samples_mod.build_pose_multiview_samples(obs, "cam-a", pair_window_us=100)
    assert len(rows) == 2
    assert rows[0].timestamps["cam-b"] == 1005
    assert rows[1].timestamps["cam-b"] == 2005


def test_build_pose_multiview_samples_keeps_low_quality_high_information_sample() -> None:
    obs = {
        "cam-a": [_obs("cam-a", 1000, 10.0, quality=0.08)],
        "cam-b": [_obs("cam-b", 1002, 1000.0, quality=0.08)],
    }
    rows = samples_mod.build_pose_multiview_samples(obs, "cam-a", pair_window_us=20)
    assert len(rows) == 1
    assert rows[0].quality["mean_observation_quality"] < 0.1
    assert rows[0].quality["parallax_proxy"] > 0.0


def test_build_pose_multiview_samples_uses_soft_suppression() -> None:
    obs = {
        "cam-a": [_obs("cam-a", 1000, 10.0), _obs("cam-a", 1020, 11.0)],
        "cam-b": [_obs("cam-b", 1004, 15.0), _obs("cam-b", 1024, 16.0)],
    }
    rows = samples_mod.build_pose_multiview_samples(obs, "cam-a", pair_window_us=40, reuse_suppression_us=50)
    assert len(rows) == 2
    assert rows[0].timestamps["cam-b"] == 1004
    assert rows[1].timestamps["cam-b"] == 1024
