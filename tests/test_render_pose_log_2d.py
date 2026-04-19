from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path


def _load_module():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    module_path = Path(__file__).resolve().parents[1] / "src" / "host" / "auxiliary" / "render_pose_log_2d.py"
    spec = importlib.util.spec_from_file_location("render_pose_log_2d", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to load render_pose_log_2d")
    module = importlib.util.module_from_spec(spec)
    sys.modules["render_pose_log_2d"] = module
    spec.loader.exec_module(module)
    return module


def test_render_pose_log_videos_emits_per_camera_mp4(tmp_path: Path) -> None:
    mod = _load_module()
    log_path = tmp_path / "pose.jsonl"
    entries = [
        {"_type": "header", "schema_version": "1.0"},
        {"_type": "frame", "data": {"camera_id": "pi-cam-01", "timestamp": 1000, "frame_index": 1, "blobs": [{"x": 100.0, "y": 120.0, "area": 30.0}]}},
        {"_type": "frame", "data": {"camera_id": "pi-cam-02", "timestamp": 1005, "frame_index": 1, "blobs": [{"x": 220.0, "y": 240.0, "area": 20.0}]}},
        {"_type": "frame", "data": {"camera_id": "pi-cam-01", "timestamp": 1010, "frame_index": 2, "blobs": [{"x": 110.0, "y": 128.0, "area": 28.0}]}},
        {"_type": "frame", "data": {"camera_id": "pi-cam-02", "timestamp": 1015, "frame_index": 2, "blobs": [{"x": 228.0, "y": 248.0, "area": 18.0}]}},
    ]
    log_path.write_text("\n".join(json.dumps(item) for item in entries), encoding="utf-8")

    outputs = mod.render_pose_log_videos(log_path=log_path, output_dir=tmp_path / "videos", fps=10, trail=3)

    assert set(outputs) == {"pi-cam-01", "pi-cam-02"}
    for output in outputs.values():
        path = Path(output)
        assert path.exists()
        assert path.stat().st_size > 0
