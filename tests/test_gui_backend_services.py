from __future__ import annotations

import json
import os
import sys
import threading
import time
import http.client
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.gui_settings_store import GuiSettingsStore
from host.gui_calibration_config_service import GuiCalibrationConfigService
from host.gui_camera_status_store import GuiCameraStatusStore
from host.gui_command_service import GuiCommandService
from host.gui_state_presenter import GuiStatePresenter
from host.gui_workflow_presenter import GuiWorkflowPresenter
from host.loutrack_gui import (
    DEFAULT_EXTRINSICS_OUTPUT_PATH,
    LoutrackGuiHandler,
    LoutrackGuiState,
    PROJECT_ROOT,
)
from host.wand_session import CameraTarget, FIXED_FPS


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
            "set_threshold",
            "set_blob_diameter",
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
                        "mjpeg_render_enabled": True,
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


class _ConfigurablePingSession(_FakeSession):
    def __init__(
        self,
        *,
        ack_by_camera: Dict[str, bool] | None = None,
        state_by_camera: Dict[str, str] | None = None,
        mask_pixels_by_camera: Dict[str, float] | None = None,
    ) -> None:
        super().__init__()
        self.ack_by_camera = ack_by_camera or {}
        self.state_by_camera = state_by_camera or {}
        self.mask_pixels_by_camera = mask_pixels_by_camera or {}

    def _broadcast(self, targets, fn_name, **kwargs):  # noqa: ANN001
        if fn_name != "ping":
            return super()._broadcast(targets, fn_name, **kwargs)
        result = {}
        for target in targets:
            ack = self.ack_by_camera.get(target.camera_id, True)
            payload = {
                "state": self.state_by_camera.get(target.camera_id, "READY"),
                "supported_commands": list(self.supported_commands),
                "blob_diagnostics": {"last_blob_count": 1},
                "mask_pixels": self.mask_pixels_by_camera.get(target.camera_id, 12),
                "mjpeg_render_enabled": True,
            }
            result[target.camera_id] = {
                "ack": ack,
                "result": payload,
                "error_message": None if ack else "offline",
            }
        return result


class _TrackingRuntime:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.running = False
        self.started_with = None
        self.started_epipolar_threshold_px = None

    def start(  # noqa: ANN001
        self,
        calibration_path: str,
        patterns,
        epipolar_threshold_px=None,
    ):
        if self.fail:
            raise RuntimeError("tracking bind failed")
        self.running = True
        self.started_with = (calibration_path, list(patterns))
        self.started_epipolar_threshold_px = epipolar_threshold_px
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


class _FakeMjpegResponse:
    def __init__(self, chunks: list[bytes], *, content_type: str = "multipart/x-mixed-replace; boundary=frame") -> None:
        self._chunks = list(chunks)
        self.headers = {"Content-Type": content_type}

    def read(self, size: int = -1) -> bytes:
        if not self._chunks:
            return b""
        if size < 0:
            data = b"".join(self._chunks)
            self._chunks.clear()
            return data
        data = self._chunks[0][:size]
        self._chunks[0] = self._chunks[0][size:]
        if not self._chunks[0]:
            self._chunks.pop(0)
        return data

    def __enter__(self) -> "_FakeMjpegResponse":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # noqa: ANN001
        return None


class _BrokenPipeWriter:
    def write(self, _data: bytes) -> int:
        raise BrokenPipeError()

    def flush(self) -> None:
        return None


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
        default_wand_metric_duration_s=1.0,
    )

    store.migrate_once()
    bundle = store.load_bundle()

    assert settings_path.exists()
    assert (tmp_path / "wand_gui_settings.json.invalid.bak").exists()
    assert bundle["calibration"]["committed"]["exposure_us"] == 5000
    assert bundle["calibration"]["committed"]["threshold"] == 150
    assert bundle["calibration"]["committed"]["mask_threshold"] == 120
    assert "fps" not in bundle["calibration"]["draft"]
    assert "fps" not in bundle["calibration"]["committed"]


def test_gui_calibration_config_service_applies_config_and_dispatches_capture_settings(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    service = GuiCalibrationConfigService(state)

    result = service.apply_config(
        {
            "camera_ids": ["pi-cam-01"],
            "exposure_us": 8000,
            "gain": 10.5,
            "mask_threshold": 175,
            "mask_seconds": 0.9,
            "wand_metric_seconds": 4.5,
        }
    )
    settings = state.get_settings()

    assert result["set_exposure"]["pi-cam-01"]["ack"] is True
    assert result["set_gain"]["pi-cam-01"]["ack"] is True
    assert result["set_threshold"]["pi-cam-01"]["ack"] is True
    assert result["set_blob_diameter"]["pi-cam-01"]["ack"] is True
    assert state.config.exposure_us == 8000
    assert state.config.gain == 10.5
    assert state.config.fps == FIXED_FPS
    assert state.config.duration_s == 1.0
    assert state._mask_params()["threshold"] == 175
    assert state._mask_params()["seconds"] == 0.9
    assert settings["calibration"]["committed"]["exposure_us"] == 8000
    assert settings["calibration"]["committed"]["gain"] == 10.5
    assert settings["calibration"]["committed"]["wand_metric_seconds"] == 1.0
    assert "fps" not in settings["calibration"]["draft"]
    assert "fps" not in settings["calibration"]["committed"]
    assert [item["fn_name"] for item in state.session.broadcast_history] == [
        "set_exposure",
        "set_gain",
        "set_threshold",
        "set_blob_diameter",
    ]
    assert "fps" not in state.get_state()["config"]
    assert state.last_result == result


def test_gui_command_service_refresh_and_preview_preserve_response_shapes(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    service = GuiCommandService(state)

    refreshed = service.run_command({"command": "refresh"})
    preview = service.run_command(
        {
            "command": "set_preview",
            "camera_ids": ["pi-cam-01"],
            "render_enabled": False,
            "overlays": {"blob": True, "mask": False},
            "charuco": {"enabled": True},
        }
    )

    assert "cameras" in refreshed
    assert len(refreshed["cameras"]) == 2
    assert "set_preview" in preview
    preview_call = state.session.broadcast_history[-1]
    assert preview_call["fn_name"] == "set_preview"
    assert preview_call["camera_ids"] == ["pi-cam-01"]
    assert preview_call["kwargs"] == {
        "render_enabled": False,
        "overlays": {"blob": True, "mask": False},
        "charuco": {"enabled": True},
    }
    assert state.last_result == preview


def test_gui_command_service_dispatches_mask_commands_with_expected_validation(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    service = GuiCommandService(state)
    validation_calls: list[str] = []

    def _validate() -> Dict[str, Any]:
        validation_calls.append("called")
        return {}

    state._ensure_calibration_settings_valid = _validate  # type: ignore[method-assign]

    started = service.run_command({"command": "mask_start", "camera_ids": ["pi-cam-01"]})
    stopped = service.run_command({"command": "mask_stop", "camera_ids": ["pi-cam-01"]})

    assert list(started) == ["mask_start"]
    assert list(stopped) == ["mask_stop"]
    assert validation_calls == ["called"]
    assert state.session.broadcast_history[-2]["fn_name"] == "mask_start"
    assert state.session.broadcast_history[-2]["kwargs"] == state._mask_params()
    assert state.session.broadcast_history[-1]["fn_name"] == "mask_stop"
    assert state.session.broadcast_history[-1]["kwargs"] == {}


def test_gui_command_service_dispatches_pose_capture_through_capture_log_service(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.capture_log_dir = tmp_path / "logs"
    state._settings_store.default_wand_metric_duration_s = 0.02
    service = GuiCommandService(state)
    validation_calls: list[str] = []

    def _validate() -> Dict[str, Any]:
        validation_calls.append("called")
        return {}

    state._ensure_calibration_settings_valid = _validate  # type: ignore[method-assign]

    started = service.run_command({"command": "start_pose_capture", "camera_ids": ["pi-cam-01"]})
    stopped = service.run_command({"command": "stop_pose_capture", "camera_ids": ["pi-cam-01"]})
    settings = state.get_settings()

    assert validation_calls == ["called"]
    assert "capture_log" in started
    assert started["start_pose_capture"]["pi-cam-01"]["ack"] is True
    assert stopped["stop_pose_capture"]["pi-cam-01"]["ack"] is True
    assert state.session.broadcast_history[-2]["fn_name"] == "start"
    assert state.session.broadcast_history[-2]["kwargs"]["mode"] == "pose_capture"
    assert isinstance(state.session.broadcast_history[-2]["kwargs"]["start_at_us"], int)
    assert state.session.broadcast_history[-1]["fn_name"] == "stop"
    assert settings["runtime_hints"]["pose_log_path"].endswith(".jsonl")


def test_gui_command_service_dispatches_wand_metric_capture_and_auto_stop(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.capture_log_dir = tmp_path / "logs"
    state._settings_store.default_wand_metric_duration_s = 0.02
    service = GuiCommandService(state)
    validation_calls: list[str] = []

    def _validate() -> Dict[str, Any]:
        validation_calls.append("called")
        return {}

    state._ensure_calibration_settings_valid = _validate  # type: ignore[method-assign]

    started = service.run_command(
        {
            "command": "start_wand_metric_capture",
            "camera_ids": ["pi-cam-01"],
            "duration_s": 3.0,
        }
    )

    assert validation_calls == ["called"]
    assert started["duration_s"] == 0.02
    assert started["start_wand_metric_capture"]["pi-cam-01"]["ack"] is True
    assert state.session.broadcast_history[-1]["fn_name"] == "start"
    assert state.session.broadcast_history[-1]["kwargs"]["mode"] == "wand_metric_capture"
    assert isinstance(state.session.broadcast_history[-1]["kwargs"]["start_at_us"], int)

    deadline = time.time() + 1.0
    while time.time() < deadline:
        if "stop_wand_metric_capture" in state.last_result:
            break
        time.sleep(0.02)

    settings = state.get_settings()
    assert "stop_wand_metric_capture" in state.last_result
    assert settings["runtime_hints"]["wand_metric_log_path"].endswith(".jsonl")


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


def test_gui_tracking_service_forwards_epipolar_threshold(tmp_path: Path) -> None:
    runtime = _TrackingRuntime()
    state = _build_state(tmp_path, runtime=runtime)
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path

    response = state._tracking_service.start_tracking(
        {
            "patterns": ["waist"],
            "camera_ids": ["pi-cam-01"],
            "epipolar_threshold_px": 3.4,
        }
    )

    assert runtime.started_epipolar_threshold_px == 3.5
    assert response["epipolar_threshold_px"] == 3.5


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


def test_gui_tracking_status_reports_extrinsics_missing(tmp_path: Path) -> None:
    state = _build_state(tmp_path, session=_ConfigurablePingSession())
    state.latest_extrinsics_path = tmp_path / "missing_extrinsics.json"

    status = state.get_tracking_status(cameras=state.refresh_targets())

    assert status["start_allowed"] is False
    assert status["start_blockers"] == ["extrinsics_missing"]
    assert status["empty_state"] == "Generate extrinsics first"


def test_gui_tracking_status_reports_no_selected_cameras(tmp_path: Path) -> None:
    state = _build_state(tmp_path, session=_ConfigurablePingSession())
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    cameras = state.refresh_targets()
    for camera in cameras:
        camera["selected"] = False

    status = state.get_tracking_status(cameras=[])

    assert status["start_allowed"] is False
    assert status["start_blockers"] == ["no_cameras_selected"]


def test_gui_tracking_status_reports_unhealthy_camera_blocker(tmp_path: Path) -> None:
    session = _ConfigurablePingSession(ack_by_camera={"pi-cam-02": False})
    state = _build_state(tmp_path, session=session)
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    state.selected_camera_ids = ["pi-cam-01", "pi-cam-02"]

    status = state.get_tracking_status(cameras=state.refresh_targets())

    assert status["start_allowed"] is False
    assert status["start_blockers"] == ["camera_unhealthy"]


def test_gui_tracking_status_reports_mask_missing_blocker(tmp_path: Path) -> None:
    session = _ConfigurablePingSession(mask_pixels_by_camera={"pi-cam-02": 0})
    state = _build_state(tmp_path, session=session)
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    state.selected_camera_ids = ["pi-cam-01", "pi-cam-02"]

    status = state.get_tracking_status(cameras=state.refresh_targets())

    assert status["start_allowed"] is False
    assert status["start_blockers"] == ["mask_missing"]


def test_gui_tracking_status_allows_start_when_preconditions_are_met(tmp_path: Path) -> None:
    state = _build_state(tmp_path, session=_ConfigurablePingSession())
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    state.selected_camera_ids = ["pi-cam-01"]

    status = state.get_tracking_status(cameras=state.refresh_targets())

    assert status["start_allowed"] is True
    assert status["start_blockers"] == []
    assert status["stop_allowed"] is False


def test_tracking_stop_is_idempotent_when_idle(tmp_path: Path) -> None:
    state = _build_state(tmp_path)

    response = state.stop_tracking()

    assert response["ok"] is True
    assert response["running"] is False
    assert response["pi_stream_stop"] == {}


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
    assert isinstance(started["scheduled_start_at_us"], int)
    assert settings["runtime_hints"]["pose_log_path"].endswith(".jsonl")


def test_gui_capture_log_service_auto_stops_wand_metric_capture(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.capture_log_dir = tmp_path / "logs"
    state._settings_store.default_wand_metric_duration_s = 0.02
    targets = state.session.discover_targets(["pi-cam-01"])

    result = state._capture_log_service.start_capture(
        "start_wand_metric_capture",
        {"duration_s": 3.0},
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
            "world": {
                "status": "resolved",
                "frame": "world",
                "floor_plane": {"axis": "Z", "normal": [0.0, 0.0, 1.0]},
                "floor_normal_sign_source": "wand_face",
                "validation": {"floor_residual_mm": 1.2},
            },
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
    assert result["generate_extrinsics"]["status"] == "success"
    assert result["generate_extrinsics"]["pose_log_path"] == str(pose_log)
    assert result["generate_extrinsics"]["world"]["frame"] == "world"
    assert result["generate_extrinsics"]["world"]["up_axis"] == "Z"
    assert result["generate_extrinsics"]["world"]["floor_normal_sign_source"] == "wand_face"
    assert state.latest_extrinsics_path == output_path
    assert PROJECT_ROOT is not None
    assert DEFAULT_EXTRINSICS_OUTPUT_PATH is not None


def test_gui_camera_status_store_refresh_targets_matches_loutrack_gui_behavior(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.selected_camera_ids = ["pi-cam-01"]
    state.camera_status = {
        "pi-cam-03": {
            "camera_id": "pi-cam-03",
            "ip": "192.168.1.103",
            "healthy": False,
            "selected": False,
            "diagnostics": {"state": "IDLE"},
            "last_ack": None,
            "last_error": "stale",
        }
    }

    actual = GuiCameraStatusStore(state).refresh_targets()
    expected = [
        {
            "camera_id": "pi-cam-01",
            "ip": "192.168.1.101",
            "healthy": True,
            "selected": True,
            "diagnostics": {
                "state": "IDLE",
                "supported_commands": list(state.session.supported_commands),
                "blob_diagnostics": {"last_blob_count": 1},
                "mjpeg_render_enabled": True,
                "preview_render_requested": True,
                "preview_transport": "proxy",
                "preview_path": "/api/cameras/pi-cam-01/mjpeg",
                "preview_error": None,
                "preview_unreachable": False,
            },
            "last_ack": True,
            "last_error": None,
        },
        {
            "camera_id": "pi-cam-02",
            "ip": "192.168.1.102",
            "healthy": True,
            "selected": False,
            "diagnostics": {
                "state": "IDLE",
                "supported_commands": list(state.session.supported_commands),
                "blob_diagnostics": {"last_blob_count": 1},
                "mjpeg_render_enabled": True,
                "preview_render_requested": True,
                "preview_transport": "proxy",
                "preview_path": "/api/cameras/pi-cam-02/mjpeg",
                "preview_error": None,
                "preview_unreachable": False,
            },
            "last_ack": True,
            "last_error": None,
        },
        {
            "camera_id": "pi-cam-03",
            "ip": "192.168.1.103",
            "healthy": False,
            "selected": False,
            "diagnostics": {"state": "IDLE"},
            "last_ack": None,
            "last_error": "stale",
        },
    ]

    assert actual == expected
    assert state.camera_status == {item["camera_id"]: item for item in expected}


def test_gui_camera_status_store_update_camera_status_matches_loutrack_gui_behavior(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.camera_status = {"pi-cam-01": {"camera_id": "pi-cam-01", "ip": "192.168.1.101"}}

    result = {
        "set_exposure": {
            "pi-cam-01": {"ack": True, "result": {"state": "IDLE"}},
            "pi-cam-02": {"ack": False, "error_message": "offline"},
        },
        "set_gain": {
            "pi-cam-03": {"ack": False, "error": "network_error"},
        },
    }

    GuiCameraStatusStore(state).update_camera_status(result)

    assert state.camera_status == {
        "pi-cam-01": {
            "camera_id": "pi-cam-01",
            "ip": "192.168.1.101",
            "last_ack": True,
            "last_error": None,
        },
        "pi-cam-02": {
            "camera_id": "pi-cam-02",
            "ip": "unknown",
            "last_ack": False,
            "last_error": "offline",
        },
        "pi-cam-03": {
            "camera_id": "pi-cam-03",
            "ip": "unknown",
            "last_ack": False,
            "last_error": "network_error",
        },
    }


def test_camera_status_includes_proxy_preview_diagnostics(tmp_path: Path) -> None:
    state = _build_state(tmp_path, session=_ConfigurablePingSession())
    state.record_preview_proxy_result("pi-cam-02", "upstream_unreachable: timed out")

    cameras = state.refresh_targets()
    diagnostics_by_camera = {camera["camera_id"]: camera["diagnostics"] for camera in cameras}

    assert diagnostics_by_camera["pi-cam-01"]["preview_render_requested"] is True
    assert diagnostics_by_camera["pi-cam-01"]["preview_path"] == "/api/cameras/pi-cam-01/mjpeg"
    assert diagnostics_by_camera["pi-cam-01"]["preview_unreachable"] is False
    assert diagnostics_by_camera["pi-cam-02"]["preview_error"] == "upstream_unreachable: timed out"
    assert diagnostics_by_camera["pi-cam-02"]["preview_unreachable"] is True


def test_camera_mjpeg_proxy_relays_upstream_stream(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    state = _build_state(tmp_path)
    LoutrackGuiHandler.state = state

    def _fake_urlopen(request, timeout=0):  # noqa: ANN001
        assert getattr(request, "full_url", request) == "http://192.168.1.101:8555/mjpeg"
        assert timeout == LoutrackGuiHandler._MJPEG_PROXY_TIMEOUT_S
        return _FakeMjpegResponse([b"--frame\r\n", b"Content-Type: image/jpeg\r\n\r\n", b"jpeg-bytes"])

    monkeypatch.setattr("host.loutrack_gui.urllib.request.urlopen", _fake_urlopen)

    server = ThreadingHTTPServer(("127.0.0.1", 0), LoutrackGuiHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        connection = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=2.0)
        connection.request("GET", "/api/cameras/pi-cam-01/mjpeg")
        response = connection.getresponse()
        assert response.status == 200
        assert response.getheader("Content-Type") == "multipart/x-mixed-replace; boundary=frame"
        assert response.read() == b"--frame\r\nContent-Type: image/jpeg\r\n\r\njpeg-bytes"
        connection.close()
    finally:
        server.shutdown()
        thread.join(timeout=2.0)
        server.server_close()

    assert state.get_preview_proxy_error("pi-cam-01") is None


def test_camera_mjpeg_proxy_client_disconnect_does_not_mark_upstream_unreachable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = _build_state(tmp_path)
    state.record_preview_proxy_result("pi-cam-01", "upstream_unreachable: old")

    handler = object.__new__(LoutrackGuiHandler)
    handler.state = state
    handler.wfile = _BrokenPipeWriter()
    handler.send_response = lambda *_args, **_kwargs: None
    handler.send_header = lambda *_args, **_kwargs: None
    handler.end_headers = lambda: None

    def _fake_urlopen(request, timeout=0):  # noqa: ANN001
        assert getattr(request, "full_url", request) == "http://192.168.1.101:8555/mjpeg"
        assert timeout == LoutrackGuiHandler._MJPEG_PROXY_TIMEOUT_S
        return _FakeMjpegResponse([b"jpeg-bytes"])

    monkeypatch.setattr("host.loutrack_gui.urllib.request.urlopen", _fake_urlopen)

    handler._proxy_camera_mjpeg("pi-cam-01")

    assert state.get_preview_proxy_error("pi-cam-01") is None


def test_camera_mjpeg_proxy_returns_502_and_records_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    state = _build_state(tmp_path)
    LoutrackGuiHandler.state = state

    def _fake_urlopen(request, timeout=0):  # noqa: ANN001
        raise urllib.error.URLError("timed out")

    monkeypatch.setattr("host.loutrack_gui.urllib.request.urlopen", _fake_urlopen)

    server = ThreadingHTTPServer(("127.0.0.1", 0), LoutrackGuiHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        connection = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=2.0)
        connection.request("GET", "/api/cameras/pi-cam-01/mjpeg")
        response = connection.getresponse()
        assert response.status == 502
        connection.close()
    finally:
        server.shutdown()
        thread.join(timeout=2.0)
        server.server_close()

    assert state.get_preview_proxy_error("pi-cam-01") == "upstream_unreachable: timed out"


@pytest.mark.parametrize(
    ("scenario", "camera_overrides"),
    [
        (
            "floor",
            [
                {
                    "camera_id": "pi-cam-01",
                    "selected": True,
                    "healthy": True,
                    "diagnostics": {
                        "state": "READY",
                        "mask_pixels": 12,
                        "debug_preview_enabled": True,
                        "debug_preview_active": False,
                        "blob_diagnostics": {"last_blob_count": 2},
                    },
                },
                {
                    "camera_id": "pi-cam-02",
                    "selected": True,
                    "healthy": False,
                    "diagnostics": {
                        "state": "READY",
                        "mask_pixels": 8,
                        "debug_preview_enabled": False,
                        "debug_preview_active": True,
                        "blob_diagnostics": {"last_blob_count": 0},
                    },
                },
            ],
        ),
        (
            "wand",
            [
                {
                    "camera_id": "pi-cam-01",
                    "selected": True,
                    "healthy": True,
                    "diagnostics": {
                        "state": "RUNNING",
                        "mask_pixels": 12,
                        "debug_preview_enabled": True,
                        "debug_preview_active": True,
                        "blob_diagnostics": {"last_blob_count": 2},
                    },
                },
                {
                    "camera_id": "pi-cam-02",
                    "selected": False,
                    "healthy": True,
                    "diagnostics": {
                        "state": "READY",
                        "mask_pixels": 8,
                        "debug_preview_enabled": False,
                        "debug_preview_active": False,
                        "blob_diagnostics": {"last_blob_count": 1},
                    },
                },
            ],
        ),
    ],
)
def test_gui_workflow_presenter_matches_loutrack_gui_summary(
    tmp_path: Path,
    scenario: str,
    camera_overrides: list[Dict[str, Any]],
) -> None:
    state = _build_state(tmp_path)
    state._active_capture_kind = "pose_capture"
    state._capture_completed = {"pose_capture": scenario == "floor", "wand_metric_capture": False}
    extrinsics_path = state.settings_path.parent / "extrinsics.json"
    extrinsics_path.write_text("{}", encoding="utf-8")
    state.latest_extrinsics_path = extrinsics_path
    state.latest_extrinsics_quality = {"median_reproj_error_px": 0.8}
    state.latest_extrinsics_result = None

    actual = GuiWorkflowPresenter(state).summarize(camera_overrides)

    assert actual == {
        "total_count": 2,
        "selected_count": 2 if scenario == "floor" else 1,
        "healthy_count": 1,
        "blob_ready_count": 1,
        "mask_ready_count": 2 if scenario == "floor" else 1,
        "running_count": 0 if scenario == "floor" else 1,
        "preview_enabled_count": 1,
        "preview_active_count": 1,
        "pose_capture_complete": scenario == "floor",
        "pose_capture_exists": False,
        "pose_capture_log_path": str(state.pose_capture_log_path),
        "wand_metric_complete": False,
        "wand_metric_exists": False,
        "wand_metric_log_path": str(state.wand_metric_log_path),
        "extrinsics_ready": True,
        "latest_extrinsics_path": str(extrinsics_path),
        "latest_extrinsics_quality": {"median_reproj_error_px": 0.8},
        "latest_extrinsics_result": None,
        "active_segment": scenario,
        "active_capture_kind": "pose_capture",
    }


def test_gui_state_presenter_matches_loutrack_gui_state_payload(tmp_path: Path) -> None:
    state = _build_state(tmp_path)
    state.last_result = {"tracking_start": {"ok": True, "running": True}}
    cameras = [{"camera_id": "pi-cam-01", "healthy": True, "selected": True, "diagnostics": {"state": "IDLE"}}]
    workflow = {"active_segment": "mask", "healthy_count": 1}
    bundle = {"validation": {"calibration": {}}, "runtime_hints": {"pose_log_path": "logs/pose.jsonl"}}
    tracking_status = {"running": True, "start_allowed": True}
    intrinsics_settings = {"camera_id": "pi-cam-01", "min_frames": 25}

    class _StubCameraStatusStore:
        def __init__(self, items: list[Dict[str, Any]]) -> None:
            self._items = items
            self.calls = 0

        def refresh_targets(self) -> list[Dict[str, Any]]:
            self.calls += 1
            return self._items

    class _StubWorkflowPresenter:
        def __init__(self, payload: Dict[str, Any]) -> None:
            self._payload = payload
            self.calls: list[list[Dict[str, Any]]] = []

        def summarize(self, items: list[Dict[str, Any]]) -> Dict[str, Any]:
            self.calls.append(items)
            return self._payload

    camera_store = _StubCameraStatusStore(cameras)
    workflow_presenter = _StubWorkflowPresenter(workflow)
    state._load_settings_bundle = lambda: bundle  # type: ignore[method-assign]
    state.get_tracking_status = lambda: tracking_status  # type: ignore[method-assign]
    state._load_intrinsics_settings = lambda: intrinsics_settings  # type: ignore[method-assign]

    actual = GuiStatePresenter(
        state,
        camera_status_store=camera_store,  # type: ignore[arg-type]
        workflow_presenter=workflow_presenter,  # type: ignore[arg-type]
    ).get_state()

    assert camera_store.calls == 1
    assert workflow_presenter.calls == [cameras]
    assert actual == {
        "config": state._config_payload(),
        "cameras": cameras,
        "workflow": workflow,
        "extrinsics_methods": state._generate_extrinsics_registry.to_payload(),
        "last_result": {"tracking_start": {"ok": True, "running": True}},
        "receiver": state.receiver.stats,
        "tracking": tracking_status,
        "intrinsics_settings": intrinsics_settings,
        "settings_meta": {
            "validation": {"calibration": {}},
            "runtime_hints": {"pose_log_path": "logs/pose.jsonl"},
        },
    }
