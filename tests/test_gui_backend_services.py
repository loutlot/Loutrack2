from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.gui_settings_store import GuiSettingsStore
from host.loutrack_gui import DEFAULT_EXTRINSICS_OUTPUT_PATH, LoutrackGuiState, PROJECT_ROOT
from host.wand_session import CameraTarget


class _FakeReceiver:
    def __init__(self) -> None:
        self._frame_callback = None
        self.is_running = False
        self.start_count = 0
        self.stop_count = 0

    def set_frame_callback(self, callback) -> None:  # noqa: ANN001
        self._frame_callback = callback

    def start(self) -> None:
        self.is_running = True
        self.start_count += 1

    def stop(self) -> None:
        self.is_running = False
        self.stop_count += 1

    @property
    def stats(self) -> Dict[str, int]:
        return {"frames_received": 0, "cameras_discovered": 2}


class _FakeSession:
    def __init__(self) -> None:
        self.targets = [
            CameraTarget(camera_id="pi-cam-01", ip="192.168.1.101"),
            CameraTarget(camera_id="pi-cam-02", ip="192.168.1.102"),
        ]
        self.broadcast_history: list[Dict[str, Any]] = []
        self.supported_commands = [
            "ping",
            "start",
            "stop",
            "set_exposure",
            "set_gain",
            "set_fps",
            "set_focus",
            "set_threshold",
            "set_blob_diameter",
            "set_circularity_min",
            "mask_start",
            "mask_stop",
            "set_preview",
            "intrinsics_start",
            "intrinsics_stop",
            "intrinsics_clear",
            "intrinsics_get_corners",
            "intrinsics_status",
        ]
        self.intrinsics_status: Dict[str, Any] = {
            "phase": "idle",
            "camera_id": "pi-cam-01",
            "frames_captured": 0,
            "frames_needed": 25,
            "frames_target": 50,
            "frames_rejected_cooldown": 0,
            "frames_rejected_spatial": 0,
            "frames_rejected_detection": 0,
            "grid_coverage": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            "last_error": None,
        }

    def discover_targets(self, camera_ids=None):  # noqa: ANN001
        if not camera_ids:
            return list(self.targets)
        wanted = set(camera_ids)
        return [target for target in self.targets if target.camera_id in wanted]

    def _broadcast(self, targets, fn_name, **kwargs):  # noqa: ANN001
        self.broadcast_history.append(
            {
                "fn_name": fn_name,
                "camera_ids": [target.camera_id for target in targets],
                "kwargs": dict(kwargs),
            }
        )
        if fn_name == "ping":
            return {
                target.camera_id: {
                    "ack": True,
                    "result": {
                        "state": "IDLE",
                        "supported_commands": list(self.supported_commands),
                        "blob_diagnostics": {"last_blob_count": 1},
                    },
                }
                for target in targets
            }
        if fn_name == "intrinsics_start":
            self.intrinsics_status = {
                **self.intrinsics_status,
                "phase": "capturing",
                "frames_needed": int(kwargs.get("min_frames", 25)),
            }
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "intrinsics_stop":
            self.intrinsics_status = {**self.intrinsics_status, "phase": "idle"}
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "intrinsics_clear":
            self.intrinsics_status = {**self.intrinsics_status, "frames_captured": 0}
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "intrinsics_get_corners":
            return {
                target.camera_id: {
                    "ack": True,
                    "result": {
                        "frames": [],
                        "count": self.intrinsics_status["frames_captured"],
                        "image_size": None,
                        "phase": self.intrinsics_status["phase"],
                        "frames_captured": self.intrinsics_status["frames_captured"],
                        "frames_rejected_cooldown": self.intrinsics_status["frames_rejected_cooldown"],
                        "frames_rejected_spatial": self.intrinsics_status["frames_rejected_spatial"],
                        "frames_rejected_detection": self.intrinsics_status["frames_rejected_detection"],
                        "grid_coverage": self.intrinsics_status["grid_coverage"],
                        "last_error": self.intrinsics_status["last_error"],
                    },
                }
                for target in targets
            }
        if fn_name == "intrinsics_status":
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "start":
            return {target.camera_id: {"ack": True, "result": {"state": "RUNNING"}} for target in targets}
        return {target.camera_id: {"ack": True, "result": {"state": "IDLE"}} for target in targets}


class _AlreadyRunningSession(_FakeSession):
    def _broadcast(self, targets, fn_name, **kwargs):  # noqa: ANN001
        if fn_name == "start":
            self.broadcast_history.append(
                {
                    "fn_name": fn_name,
                    "camera_ids": [target.camera_id for target in targets],
                    "kwargs": dict(kwargs),
                }
            )
            return {
                target.camera_id: {
                    "ack": False,
                    "error_message": "already_running",
                }
                for target in targets
            }
        return super()._broadcast(targets, fn_name, **kwargs)


class _TrackingRuntime:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.running = False
        self.started_with = None

    def start(self, calibration_path: str, patterns):  # noqa: ANN001
        if self.fail:
            raise RuntimeError("tracking bind failed")
        self.running = True
        self.started_with = (calibration_path, list(patterns))
        return {"running": True, "calibration_loaded": True}

    def stop(self) -> Dict[str, Any]:
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


def _build_state(
    tmp_path: Path,
    *,
    session: _FakeSession | None = None,
    runtime: _TrackingRuntime | None = None,
    receiver: _FakeReceiver | None = None,
) -> LoutrackGuiState:
    return LoutrackGuiState(
        session=session or _FakeSession(),
        receiver=receiver or _FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=runtime or _TrackingRuntime(),
    )


def test_gui_settings_store_migrates_invalid_legacy_payload_to_backup(tmp_path: Path) -> None:
    settings_path = tmp_path / "loutrack_gui_settings.json"
    old_settings = tmp_path / "wand_gui_settings.json"
    old_settings.write_text("{invalid-json", encoding="utf-8")
    store = GuiSettingsStore(
        project_root=tmp_path,
        settings_path=settings_path,
        old_settings_path=old_settings,
        uses_default_settings_path=True,
        default_pose_log_path=Path("logs/extrinsics_pose_capture.jsonl"),
        default_wand_metric_log_path=Path("logs/extrinsics_wand_metric.jsonl"),
        default_extrinsics_output_path=Path("calibration/extrinsics_pose_v2.json"),
        default_wand_metric_duration_s=3.0,
    )

    store.migrate_once()
    bundle = store.load_bundle()

    assert settings_path.exists()
    assert (tmp_path / "wand_gui_settings.json.invalid.bak").exists()
    assert bundle["calibration"]["committed"]["exposure_us"] == 12000


def test_gui_tracking_service_tolerates_already_running_streams(tmp_path: Path) -> None:
    session = _AlreadyRunningSession()
    state = _build_state(tmp_path, session=session, runtime=_TrackingRuntime())
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path

    response = state._tracking_service.start_tracking({"patterns": ["waist"], "camera_ids": ["pi-cam-01"]})

    assert response["ok"] is True
    assert response["running"] is True
    assert response["pi_stream_start"]["pi-cam-01"]["error_message"] == "already_running"


def test_gui_tracking_service_rolls_back_receiver_when_runtime_start_fails(tmp_path: Path) -> None:
    receiver = _FakeReceiver()
    receiver.start()
    state = _build_state(tmp_path, receiver=receiver, runtime=_TrackingRuntime(fail=True))
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path

    with pytest.raises(RuntimeError, match="tracking bind failed"):
        state._tracking_service.start_tracking({"patterns": ["waist"]})

    assert receiver.is_running is True
    assert receiver.stop_count == 1
    assert receiver.start_count == 2


def test_gui_intrinsics_service_rejects_missing_capability(tmp_path: Path) -> None:
    session = _FakeSession()
    session.supported_commands = ["ping", "intrinsics_start", "intrinsics_stop", "intrinsics_clear", "intrinsics_status"]
    state = _build_state(tmp_path, session=session)
    state.apply_settings_draft({"intrinsics": {"camera_id": "pi-cam-01", "square_length_mm": 30}})

    with pytest.raises(ValueError, match="intrinsics capability missing on Pi"):
        state._intrinsics_service.start_intrinsics_capture({})


def test_gui_intrinsics_service_builds_session_and_merges_remote_status(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "square_length_mm": 60,
                "marker_length_mm": 45,
                "squares_x": 6,
                "squares_y": 8,
                "min_frames": 5,
                "cooldown_s": 1.5,
            }
        }
    )

    response = state._intrinsics_service.start_intrinsics_capture({})
    host_session = state._intrinsics_host_session
    assert response["ok"] is True
    assert host_session is not None

    host_session._phase = "calibrating"
    state.session.intrinsics_status = {
        **state.session.intrinsics_status,
        "phase": "capturing",
        "frames_captured": 12,
        "frames_rejected_cooldown": 7,
    }

    status = state._intrinsics_service.get_intrinsics_status()

    assert status["phase"] == "calibrating"
    assert status["frames_captured"] == 12
    assert status["frames_rejected_cooldown"] == 7
    host_session.stop()


def test_gui_capture_log_service_pose_capture_start_and_stop_updates_runtime_hints(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.capture_log_dir = tmp_path / "logs"
    targets = state.session.discover_targets(["pi-cam-01"])

    started = state._capture_log_service.start_capture("start_pose_capture", {}, targets)
    stopped = state._capture_log_service.stop_capture("stop_pose_capture", targets)
    settings = state.get_settings()

    assert "capture_log" in started
    assert "capture_log" in stopped
    assert settings["runtime_hints"]["pose_log_path"].endswith(".jsonl")


def test_gui_capture_log_service_auto_stops_wand_metric_capture(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.capture_log_dir = tmp_path / "logs"
    targets = state.session.discover_targets(["pi-cam-01"])

    result = state._capture_log_service.start_capture(
        "start_wand_metric_capture",
        {"duration_s": 0.02},
        targets,
    )
    assert result["duration_s"] == 0.02

    deadline = time.time() + 1.0
    while time.time() < deadline:
        if "stop_wand_metric_capture" in state.last_result:
            break
        time.sleep(0.02)

    settings = state.get_settings()
    assert "stop_wand_metric_capture" in state.last_result
    assert settings["runtime_hints"]["wand_metric_log_path"].endswith(".jsonl")


def test_gui_extrinsics_service_reports_missing_pose_log_and_solver_failure(tmp_path: Path) -> None:
    state = _build_state(tmp_path)

    missing = state._extrinsics_service.generate_extrinsics({"pose_log_path": str(tmp_path / "missing.jsonl")})
    assert missing["generate_extrinsics"]["ok"] is False
    assert "log_path does not exist" in missing["generate_extrinsics"]["error"]

    pose_log = tmp_path / "pose.jsonl"
    pose_log.write_text(json.dumps({"camera_id": "pi-cam-01", "blob_count": 1, "blobs": [{"x": 1, "y": 2}]}), encoding="utf-8")

    def _raise_solver(**_: Any) -> Dict[str, Any]:
        raise ValueError("At least two cameras with intrinsics and single-blob pose_capture observations are required")

    state._generate_extrinsics_solver = _raise_solver  # type: ignore[attr-defined]
    failed = state._extrinsics_service.generate_extrinsics({"pose_log_path": str(pose_log)})

    assert failed["generate_extrinsics"]["ok"] is False
    assert failed["generate_extrinsics"]["pose_log_summary"]["usable_camera_count"] == 1


def test_gui_extrinsics_service_returns_quality_summary_on_success(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    state = _build_state(tmp_path)
    pose_log = tmp_path / "pose.jsonl"
    pose_log.write_text("{}", encoding="utf-8")
    output_path = tmp_path / "extrinsics_pose_v2.json"

    def _solve_pose(**kwargs: Any) -> Dict[str, Any]:
        payload = {
            "camera_order": ["pi-cam-01"],
            "pose": {
                "camera_poses": [{"camera_id": "pi-cam-01"}],
                "solve_summary": {
                    "usable_rows": 12,
                    "complete_rows": 10,
                    "median_reproj_error_px": 0.8,
                    "matched_delta_us_p90": 700,
                },
            },
            "metric": {"status": "resolved"},
            "world": {"status": "resolved"},
        }
        Path(str(kwargs["output_path"])).write_text(json.dumps(payload), encoding="utf-8")
        return payload

    state._generate_extrinsics_solver = _solve_pose  # type: ignore[attr-defined]
    result = state._extrinsics_service.generate_extrinsics(
        {
            "pose_log_path": str(pose_log),
            "output_path": str(output_path),
        }
    )

    assert result["generate_extrinsics"]["ok"] is True
    assert result["generate_extrinsics"]["quality"]["median_reproj_error_px"] == 0.8
    assert state.latest_extrinsics_path == output_path
    assert PROJECT_ROOT is not None
    assert DEFAULT_EXTRINSICS_OUTPUT_PATH is not None
