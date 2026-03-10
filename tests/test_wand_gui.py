from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from pathlib import Path
from typing import Any, Dict

from host.wand_gui import HTML_PAGE, WandGuiState, _resolve_static_asset
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
        self.calls = []

    def discover_targets(self, camera_ids=None):
        if not camera_ids:
            return list(self.targets)
        wanted = set(camera_ids)
        return [target for target in self.targets if target.camera_id in wanted]

    def _is_healthy(self, target):
        return target.camera_id == "pi-cam-01"

    def _broadcast(self, targets, fn_name, **kwargs):
        self.calls.append((fn_name, [target.camera_id for target in targets], kwargs))
        return {
            target.camera_id: {
                "ack": True,
                "result": {
                    "state": "IDLE",
                    "blob_diagnostics": {
                        "last_blob_count": 3,
                        "rejected_by_diameter": 1,
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
        self.stopped = False

    def start(self, calibration_path: str, patterns):
        self.running = True
        self.started_with = (calibration_path, list(patterns))
        return {
            "running": True,
            "calibration_loaded": True,
            "frames_processed": 5,
            "poses_estimated": 3,
            "receiver": {"frames_received": 12},
            "sync": {"coverage_5000us": 1.0},
            "calibration_path": calibration_path,
            "patterns": list(patterns),
            "uptime_seconds": 0.1,
            "last_stop_summary": {},
        }

    def stop(self):
        self.running = False
        self.stopped = True
        return {
            "frames_processed": 5,
            "poses_estimated": 3,
            "duration_seconds": 1.2,
            "log_metadata": {"log_file": "logs/tracking_gui.jsonl"},
        }

    def status(self) -> Dict[str, Any]:
        return {
            "running": self.running,
            "calibration_loaded": self.running,
            "frames_processed": 5 if self.running else 0,
            "poses_estimated": 3 if self.running else 0,
            "receiver": {"frames_received": 12 if self.running else 0},
            "sync": {"coverage_5000us": 1.0 if self.running else 0.0},
            "calibration_path": self.started_with[0] if self.started_with else None,
            "patterns": self.started_with[1] if self.started_with else ["waist"],
            "uptime_seconds": 0.1 if self.running else 0.0,
            "last_stop_summary": {},
        }

    def scene_snapshot(self) -> Dict[str, Any]:
        return {
            "tracking": {
                "running": self.running,
                "frames_processed": 5 if self.running else 0,
                "poses_estimated": 3 if self.running else 0,
            },
            "cameras": [],
            "rigid_bodies": [],
            "raw_points": [],
            "timestamp_us": 123,
        }


def test_gui_state_apply_and_command(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )

    snapshot = state.get_state()
    assert len(snapshot["cameras"]) == 2
    assert snapshot["workflow"]["active_segment"] == "mask"
    assert "Blob Detection Adjustment" in HTML_PAGE
    assert "Mask Adjustment" in HTML_PAGE
    assert "Wand Capture" in HTML_PAGE
    assert "Extrinsics Generation" in HTML_PAGE
    assert "Calibration" in HTML_PAGE
    assert "Tracking" in HTML_PAGE

    applied = state.apply_config(
        {
            "camera_ids": ["pi-cam-01"],
            "exposure_us": 1400,
            "gain": 5.0,
            "fps": 75,
            "focus": 5.215,
            "threshold": 210,
            "circularity_min": 0.4,
            "blob_min_diameter_px": 2.0,
            "blob_max_diameter_px": 20.0,
            "mask_threshold": 205,
            "mask_seconds": 0.6,
        }
    )
    assert set(applied.keys()) == {
        "set_exposure",
        "set_gain",
        "set_fps",
        "set_focus",
        "set_threshold",
        "set_blob_diameter",
        "set_circularity_min",
    }
    persisted = (tmp_path / "wand_gui_settings.json").read_text(encoding="utf-8")
    assert '"exposure_us": 1400' in persisted
    assert '"threshold": 210' in persisted

    result = state.run_command({"command": "ping", "camera_ids": ["pi-cam-01"]})
    assert "ping" in result

    mask_cleared = state.run_command({"command": "mask_stop", "camera_ids": ["pi-cam-01"]})
    assert "mask_stop" in mask_cleared

    state._extrinsics_solver = lambda **kwargs: {  # type: ignore[attr-defined]
        "reference_camera_id": "pi-cam-01",
        "cameras": [{"camera_id": "pi-cam-01"}, {"camera_id": "pi-cam-02"}],
    }
    log_path = tmp_path / "wand_capture.jsonl"
    log_path.write_text("{}", encoding="utf-8")
    generated = state.generate_extrinsics(
        {
            "intrinsics_path": "calibration",
            "log_path": str(log_path),
            "output_path": "calibration/calibration_extrinsics_v1.json",
        }
    )
    assert generated["generate_extrinsics"]["ok"] is True
    assert generated["generate_extrinsics"]["camera_count"] == 2
    assert generated["generate_extrinsics"]["quality"]["pair_count_total"] >= 0

    snapshot = state.get_state()
    assert snapshot["workflow"]["extrinsics_ready"] is True
    assert snapshot["workflow"]["latest_extrinsics_path"] == "calibration/calibration_extrinsics_v1.json"

    tracking_start = state.start_tracking({"patterns": ["waist"]})
    assert tracking_start["ok"] is True
    assert tracking_start["running"] is True
    assert state.tracking_runtime.started_with == ("calibration", ["waist"])

    tracking_status = state.get_tracking_status()
    assert tracking_status["start_allowed"] is True
    assert tracking_status["running"] is True

    tracking_scene = state.get_tracking_scene()
    assert "tracking" in tracking_scene

    tracking_stop = state.stop_tracking()
    assert tracking_stop["ok"] is True
    assert tracking_stop["running"] is False
    assert tracking_stop["summary"]["frames_processed"] == 5
    assert "summary" not in tracking_stop["summary"]


def test_gui_html_normalizes_blob_range() -> None:
    assert "function normalizedBlobRange()" in HTML_PAGE
    assert "maxValue = minValue;" in HTML_PAGE


def test_generate_extrinsics_rejects_missing_log_path(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state._extrinsics_solver = lambda **kwargs: {  # type: ignore[attr-defined]
        "reference_camera_id": "pi-cam-01",
        "cameras": [{"camera_id": "pi-cam-01"}],
    }

    try:
        state.generate_extrinsics({"log_path": str(tmp_path / "missing.jsonl")})
    except ValueError as exc:
        assert "log_path does not exist" in str(exc)
    else:
        raise AssertionError("generate_extrinsics should reject missing log path")


def test_start_stop_writes_wand_capture_log(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    receiver = _FakeReceiver()
    state = WandGuiState(
        session=_FakeSession(),
        receiver=receiver,
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.capture_log_dir = tmp_path / "logs"
    state.capture_log_path = state.capture_log_dir / "wand_capture.jsonl"
    state.capture_log_dir.mkdir(parents=True, exist_ok=True)
    state.capture_log_path.write_text("stale", encoding="utf-8")

    start_result = state.run_command({"command": "start", "camera_ids": ["pi-cam-01"]})
    log_path = Path(start_result["capture_log"]["path"])
    assert log_path.exists()
    assert "stale" not in log_path.read_text(encoding="utf-8")

    class _Frame:
        def to_dict(self):
            return {
                "camera_id": "pi-cam-01",
                "timestamp": 1000,
                "frame_index": 1,
                "blobs": [{"x": 1.0, "y": 2.0, "area": 3.0}],
            }

    receiver.emit_frame(_Frame())
    stop_result = state.run_command({"command": "stop", "camera_ids": ["pi-cam-01"]})
    assert stop_result["capture_log"]["log_file"] == str(log_path)
    workflow = state.get_state()["workflow"]
    assert workflow["wand_capture_complete"] is True

    lines = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    assert any(item.get("_type") == "frame" for item in lines)
    assert any(
        item.get("_type") == "frame"
        and item.get("data", {}).get("camera_id") == "pi-cam-01"
        for item in lines
    )


def test_gui_loads_persisted_settings(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    path = tmp_path / "wand_gui_settings.json"
    path.write_text(
        '{"exposure_us": 15000, "gain": 9.0, "fps": 56, "threshold": 215, "mask_threshold": 207, "mask_seconds": 0.8}',
        encoding="utf-8",
    )
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=path,
        tracking_runtime=_FakeTrackingRuntime(),
    )
    snapshot = state.get_state()
    assert snapshot["config"]["exposure_us"] == 15000
    assert snapshot["config"]["gain"] == 9.0
    assert snapshot["config"]["threshold"] == 215


def test_tracking_empty_state_and_start_rejection(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "missing_extrinsics.json")
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    status = state.get_tracking_status()
    scene = state.get_tracking_scene()
    assert status["start_allowed"] is False
    assert status["empty_state"] == "Generate extrinsics first"
    assert scene["empty_state"] == "Generate extrinsics first"
    assert "Waiting for scene data" in HTML_PAGE

    try:
        state.start_tracking({"patterns": ["waist"]})
    except ValueError as exc:
        assert "generate extrinsics first" in str(exc)
    else:
        raise AssertionError("start_tracking should reject when extrinsics is missing")


def test_start_tracking_resolves_extrinsics_file_to_calibration_directory(tmp_path: Path, monkeypatch) -> None:
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


def test_state_restores_existing_default_extrinsics_on_startup(tmp_path: Path, monkeypatch) -> None:
    default_extrinsics = tmp_path / "calibration_extrinsics_v1.json"
    default_extrinsics.write_text(
        '{"cameras":[{"camera_id":"pi-cam-01","quality":{"pair_count":12,"inlier_ratio":0.9,"median_reproj_error_px":2.5}}]}',
        encoding="utf-8",
    )
    monkeypatch.setattr("host.wand_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", default_extrinsics)

    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )

    tracking_status = state.get_tracking_status()
    assert tracking_status["start_allowed"] is True
    assert tracking_status["latest_extrinsics_path"] == str(default_extrinsics)
    assert tracking_status["latest_extrinsics_quality"]["pair_count_total"] == 12


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

    monkeypatch.setattr(
        "host.wand_gui.STATIC_DIR_CANDIDATES",
        (repo_static, module_static),
    )

    resolved = _resolve_static_asset("vendor/three.module.min.js")

    assert resolved == repo_asset


def test_resolve_static_asset_rejects_path_traversal(tmp_path: Path, monkeypatch) -> None:
    repo_static = tmp_path / "static"
    repo_static.mkdir()
    outside = tmp_path / "static-evil"
    outside.mkdir()
    (outside / "secret.js").write_text("nope", encoding="utf-8")

    monkeypatch.setattr("host.wand_gui.STATIC_DIR_CANDIDATES", (repo_static,))

    assert _resolve_static_asset("../static-evil/secret.js") is None
