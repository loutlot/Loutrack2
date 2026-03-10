from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Dict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.wand_gui import HTML_PAGE, PROJECT_ROOT, WandGuiState, _resolve_static_asset
from host.wand_session import CameraTarget


class _FakeReceiver:
    def __init__(self):
        self._frame_callback = None

    def set_frame_callback(self, callback):
        self._frame_callback = callback

    def emit_frame(self, frame):
        if self._frame_callback:
            self._frame_callback(frame)

    @property
    def stats(self):
        return {"frames_received": 0, "cameras_discovered": 2}


class _FakeSession:
    def __init__(self):
        self.targets = [
            CameraTarget(camera_id="pi-cam-01", ip="192.168.1.101"),
            CameraTarget(camera_id="pi-cam-02", ip="192.168.1.102"),
        ]

    def discover_targets(self, camera_ids=None):
        if not camera_ids:
            return list(self.targets)
        wanted = set(camera_ids)
        return [target for target in self.targets if target.camera_id in wanted]

    def _broadcast(self, targets, fn_name, **kwargs):
        return {
            target.camera_id: {
                "ack": True,
                "result": {
                    "state": "IDLE",
                    "blob_diagnostics": {
                        "last_blob_count": 1,
                        "rejected_by_diameter": 0,
                        "rejected_by_circularity": 0,
                    },
                },
            }
            for target in targets
        }


class _FakeTrackingRuntime:
    def __init__(self):
        self.running = False
        self.started_with = None

    def start(self, calibration_path: str, patterns):
        self.running = True
        self.started_with = (calibration_path, list(patterns))
        return {"running": True, "calibration_loaded": True}

    def stop(self):
        self.running = False
        return {"frames_processed": 5}

    def status(self) -> Dict[str, Any]:
        return {"running": self.running, "calibration_loaded": self.running}

    def scene_snapshot(self) -> Dict[str, Any]:
        return {
            "tracking": {"running": self.running, "frames_processed": 0, "poses_estimated": 0},
            "cameras": [],
            "rigid_bodies": [],
            "raw_points": [],
            "timestamp_us": 123,
        }


def test_gui_pose_and_wand_metric_flow(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    receiver = _FakeReceiver()
    state = WandGuiState(
        session=_FakeSession(),
        receiver=receiver,
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.capture_log_dir = tmp_path / "logs"
    state.capture_log_dir.mkdir(parents=True, exist_ok=True)

    start_pose = state.run_command({"command": "start_pose_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in start_pose
    assert "Start Pose Capture" in HTML_PAGE

    class _Frame:
        def to_dict(self):
            return {
                "camera_id": "pi-cam-01",
                "timestamp": 1000,
                "frame_index": 1,
                "blobs": [{"x": 100.0, "y": 200.0, "area": 30.0}],
            }

    receiver.emit_frame(_Frame())
    stop_pose = state.run_command({"command": "stop_pose_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in stop_pose

    start_metric = state.run_command({"command": "start_wand_metric_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in start_metric
    receiver.emit_frame(_Frame())
    stop_metric = state.run_command({"command": "stop_wand_metric_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in stop_metric

    snapshot = state.get_state()
    assert snapshot["workflow"]["pose_capture_log_path"]
    assert snapshot["workflow"]["wand_metric_log_path"]


def test_generate_extrinsics_and_apply_wand_scale_floor_use_split_paths(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    pose_log.write_text("{}", encoding="utf-8")
    wand_metric_log = tmp_path / "extrinsics_wand_metric.jsonl"
    wand_metric_log.write_text("{}", encoding="utf-8")
    generated_called: Dict[str, Any] = {}
    applied_called: Dict[str, Any] = {}

    def _solve_similarity(**kwargs):
        generated_called.update(kwargs)
        output_path = Path(str(kwargs["output_path"]))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(
                {
                    "reference_camera_id": "pi-cam-01",
                    "session_meta": {"scale_source": "none", "floor_source": "none"},
                    "cameras": [],
                }
            ),
            encoding="utf-8",
        )
        return {"reference_camera_id": "pi-cam-01", "cameras": []}

    def _apply_metric(**kwargs):
        applied_called.update(kwargs)
        output_path = Path(str(kwargs["output_path"]))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(
                {
                    "reference_camera_id": "pi-cam-01",
                    "session_meta": {"scale_source": "wand_floor_metric", "floor_source": "wand_floor_metric"},
                    "cameras": [],
                }
            ),
            encoding="utf-8",
        )
        return {
            "reference_camera_id": "pi-cam-01",
            "session_meta": {"scale_source": "wand_floor_metric", "floor_source": "wand_floor_metric"},
            "cameras": [],
        }

    state._generate_extrinsics_solver = _solve_similarity  # type: ignore[attr-defined]
    state._apply_wand_scale_floor_solver = _apply_metric  # type: ignore[attr-defined]
    generated = state.generate_extrinsics(
        {
            "intrinsics_path": "calibration",
            "pose_log_path": str(pose_log),
            "output_path": "calibration/calibration_extrinsics_v1.json",
        }
    )
    assert generated["generate_extrinsics"]["ok"] is True
    assert "pose_log_path" in generated_called
    assert "wand_metric_log_path" not in generated_called

    applied = state.apply_wand_scale_floor(
        {
            "intrinsics_path": "calibration",
            "wand_metric_log_path": str(wand_metric_log),
            "output_path": "calibration/calibration_extrinsics_v1.json",
        }
    )
    assert applied["apply_wand_scale_floor"]["ok"] is True
    assert "extrinsics_path" in applied_called
    assert "wand_metric_log_path" in applied_called


def test_resolve_static_asset_prefers_repo_root_static(tmp_path: Path, monkeypatch) -> None:
    repo_static = tmp_path / "static"
    module_static = tmp_path / "src" / "static"
    repo_static.mkdir(parents=True)
    module_static.mkdir(parents=True)
    repo_asset = repo_static / "vendor" / "three.module.min.js"
    module_asset = module_static / "vendor" / "three.module.min.js"
    repo_asset.parent.mkdir(parents=True)
    module_asset.parent.mkdir(parents=True)
    repo_asset.write_text("repo-root", encoding="utf-8")
    module_asset.write_text("module-src", encoding="utf-8")
    monkeypatch.setattr("host.wand_gui.STATIC_DIR_CANDIDATES", (repo_static, module_static))
    resolved = _resolve_static_asset("vendor/three.module.min.js")
    assert resolved == repo_asset


def test_start_tracking_uses_calibration_dir(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    runtime = _FakeTrackingRuntime()
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=runtime,
    )
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "calibration_extrinsics_v1.json"
    extrinsics_path.write_text('{"cameras":[]}', encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    state.start_tracking({"patterns": ["waist"]})
    assert runtime.started_with == (str(calibration_dir), ["waist"])
    assert PROJECT_ROOT is not None
