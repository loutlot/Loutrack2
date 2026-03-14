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


def test_gui_pose_and_floor_capture_paths(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    monkeypatch.setattr("host.wand_gui.DEFAULT_WAND_METRIC_LOG_PATH", tmp_path / "extrinsics_wand_metric.jsonl")
    receiver = _FakeReceiver()
    state = WandGuiState(
        session=_FakeSession(),
        receiver=receiver,
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.capture_log_dir = tmp_path / "logs"
    state.capture_log_dir.mkdir(parents=True, exist_ok=True)

    assert "Capture Floor / Metric" in HTML_PAGE
    assert "wandMetricLogPath" in HTML_PAGE

    start_pose = state.run_command({"command": "start_pose_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in start_pose

    class _Frame:
        def to_dict(self):
            return {
                "camera_id": "pi-cam-01",
                "timestamp": 1000,
                "frame_index": 1,
                "blobs": [{"x": 100.0, "y": 200.0, "area": 30.0}],
                "blob_count": 1,
            }

    receiver.emit_frame(_Frame())
    stop_pose = state.run_command({"command": "stop_pose_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in stop_pose

    scheduled = {}
    monkeypatch.setattr(
        state,
        "_schedule_auto_stop",
        lambda camera_ids, duration_s, capture_kind: scheduled.update(
            {"camera_ids": list(camera_ids), "duration_s": duration_s, "capture_kind": capture_kind}
        ),
    )
    start_metric = state.run_command(
        {
            "command": "start_wand_metric_capture",
            "camera_ids": ["pi-cam-01"],
            "duration_s": 2.5,
        }
    )
    assert start_metric["duration_s"] == 2.5
    assert scheduled == {
        "camera_ids": ["pi-cam-01"],
        "duration_s": 2.5,
        "capture_kind": "wand_metric_capture",
    }
    receiver.emit_frame(_Frame())
    stop_metric = state.run_command({"command": "stop_wand_metric_capture", "camera_ids": ["pi-cam-01"]})
    assert "capture_log" in stop_metric

    snapshot = state.get_state()
    assert snapshot["workflow"]["pose_capture_log_path"]
    assert snapshot["workflow"]["wand_metric_log_path"]
    assert snapshot["workflow"]["wand_metric_complete"] is True


def test_generate_extrinsics_summary_and_tracking_use_v2_path(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    monkeypatch.setattr("host.wand_gui.DEFAULT_WAND_METRIC_LOG_PATH", tmp_path / "extrinsics_wand_metric.jsonl")
    runtime = _FakeTrackingRuntime()
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=runtime,
    )
    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    pose_log.write_text("{}", encoding="utf-8")
    wand_log = tmp_path / "extrinsics_wand_metric.jsonl"
    wand_log.write_text("{}", encoding="utf-8")
    output_path = tmp_path / "extrinsics_pose_v2.json"
    called: Dict[str, Any] = {}

    def _solve_pose(**kwargs):
        called.update(kwargs)
        payload = {
            "schema_version": "2.0",
            "method": "reference_pose_capture",
            "camera_order": ["pi-cam-01", "pi-cam-02"],
            "pose": {
                "frame": "similarity_camera",
                "camera_poses": [
                    {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0, 0, 0], "focal_scale": 1.0, "median_reproj_error_px": 0.5},
                    {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.1, 0.0, 0.0], "focal_scale": 1.0, "median_reproj_error_px": 0.7},
                ],
                "solve_summary": {
                    "captured_rows": 40,
                    "usable_rows": 32,
                    "complete_rows": 28,
                    "pair_overlaps": {"pi-cam-01|pi-cam-02": 32},
                    "median_reproj_error_px": 1.2,
                    "p90_reproj_error_px": 2.4,
                    "matched_delta_us_p50": 600,
                    "matched_delta_us_p90": 900,
                    "matched_delta_us_max": 1200,
                },
            },
            "wand_metric_log_path": str(wand_log),
            "metric": {"status": "resolved", "scale_m_per_unit": 1.0, "source": "wand_floor_metric"},
            "world": {"status": "resolved", "frame": "world", "to_world_matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], "floor_plane": {"normal": [0,0,1], "offset": 0.0, "axis": "Z"}, "source": "wand_floor_metric"},
        }
        Path(kwargs["output_path"]).write_text(json.dumps(payload), encoding="utf-8")
        return payload

    state._generate_extrinsics_solver = _solve_pose  # type: ignore[attr-defined]
    generated = state.generate_extrinsics(
        {
            "intrinsics_path": "calibration",
            "pose_log_path": str(pose_log),
            "wand_metric_log_path": str(wand_log),
            "output_path": str(output_path),
        }
    )
    assert generated["generate_extrinsics"]["ok"] is True
    assert generated["generate_extrinsics"]["quality"]["median_reproj_error_px"] == 1.2
    assert generated["generate_extrinsics"]["quality"]["matched_delta_us_p90"] == 900
    assert generated["generate_extrinsics"]["metric_status"] == "resolved"
    assert generated["generate_extrinsics"]["world_status"] == "resolved"
    assert called["wand_metric_log_path"] == str(wand_log)

    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text(output_path.read_text(encoding="utf-8"), encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    state.start_tracking({"patterns": ["waist"]})
    assert runtime.started_with == (str(calibration_dir), ["waist"])
    assert PROJECT_ROOT is not None


def test_generate_extrinsics_returns_diagnostic_instead_of_raising_for_missing_pose_rows(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    pose_log.write_text(
        "\n".join(
            (
                json.dumps({"camera_id": "pi-cam-01", "blob_count": 1, "blobs": [{"x": 10.0, "y": 20.0}]}),
                json.dumps({"camera_id": "pi-cam-02", "blob_count": 2, "blobs": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]}),
            )
        ),
        encoding="utf-8",
    )

    def _raise_solver(**_: Any) -> Dict[str, Any]:
        raise ValueError("At least two cameras with intrinsics and single-blob pose_capture observations are required")

    state._generate_extrinsics_solver = _raise_solver  # type: ignore[attr-defined]
    result = state.generate_extrinsics({"pose_log_path": str(pose_log)})

    summary = result["generate_extrinsics"]
    assert summary["ok"] is False
    assert "At least two cameras" in summary["error"]
    assert summary["pose_log_summary"]["rows_by_camera"] == {"pi-cam-01": 1, "pi-cam-02": 1}
    assert summary["pose_log_summary"]["single_blob_rows_by_camera"] == {"pi-cam-01": 1}
    assert summary["pose_log_summary"]["usable_camera_count"] == 1


def test_generate_extrinsics_returns_diagnostic_for_missing_pose_log(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )

    result = state.generate_extrinsics({"pose_log_path": str(tmp_path / "missing_pose.jsonl")})

    summary = result["generate_extrinsics"]
    assert summary["ok"] is False
    assert "log_path does not exist" in summary["error"]
    assert summary["pose_log_summary"]["usable_camera_count"] == 0


def test_workflow_marks_existing_pose_and_metric_logs_complete(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    monkeypatch.setattr("host.wand_gui.DEFAULT_WAND_METRIC_LOG_PATH", tmp_path / "extrinsics_wand_metric.jsonl")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.pose_capture_log_path = tmp_path / "extrinsics_pose_capture.jsonl"
    state.pose_capture_log_path.write_text("{}", encoding="utf-8")
    state.wand_metric_log_path = tmp_path / "extrinsics_wand_metric.jsonl"
    state.wand_metric_log_path.write_text("{}", encoding="utf-8")

    snapshot = state.get_state()

    assert snapshot["workflow"]["pose_capture_exists"] is True
    assert snapshot["workflow"]["pose_capture_complete"] is True
    assert snapshot["workflow"]["wand_metric_exists"] is True
    assert snapshot["workflow"]["wand_metric_complete"] is True


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
