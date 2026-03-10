from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


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


mod = _load_module("extrinsics_capture", "src/camera-calibration/extrinsics_capture.py")


def test_pose_capture_reader_selects_single_best_blob(tmp_path: Path) -> None:
    log = tmp_path / "pose.jsonl"
    log.write_text(
        "\n".join(
            [
                json.dumps({"_type": "header", "schema_version": "1.0"}),
                json.dumps(
                    {
                        "_type": "frame",
                        "data": {
                            "camera_id": "pi-cam-01",
                            "timestamp": 100,
                            "frame_index": 1,
                            "blobs": [
                                {"x": 10.0, "y": 20.0, "area": 5.0},
                                {"x": 30.0, "y": 40.0, "area": 9.0},
                            ],
                        },
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )
    rows = mod.load_pose_capture_observations(log)
    assert "pi-cam-01" in rows
    assert rows["pi-cam-01"][0].blob_count == 2
    assert rows["pi-cam-01"][0].image_point.tolist() == [30.0, 40.0]
