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


def test_build_pose_multiview_samples_monotonic() -> None:
    obs = {
        "cam-a": [
            capture_mod.PoseCaptureObservation("cam-a", 1000, 0, np.array([10.0, 10.0]), 10.0, 1, 1.0),
            capture_mod.PoseCaptureObservation("cam-a", 2000, 1, np.array([12.0, 10.0]), 10.0, 1, 1.0),
        ],
        "cam-b": [
            capture_mod.PoseCaptureObservation("cam-b", 1010, 0, np.array([8.0, 10.0]), 9.0, 1, 1.0),
            capture_mod.PoseCaptureObservation("cam-b", 2010, 1, np.array([9.0, 11.0]), 9.0, 1, 1.0),
        ],
    }
    rows = samples_mod.build_pose_multiview_samples(obs, "cam-a", pair_window_us=100)
    assert len(rows) == 2
    assert rows[0].sample_id == 0
    assert rows[1].sample_id == 1
    assert rows[0].timestamps["cam-b"] == 1010
