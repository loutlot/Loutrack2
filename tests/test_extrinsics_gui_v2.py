from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Dict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.intrinsics_host_session import IntrinsicsHostSession, IntrinsicsHostSessionConfig
from host.loutrack_gui import HTML_PAGE, PROJECT_ROOT, LoutrackGuiState, _resolve_static_asset
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
        self.last_broadcast: Dict[str, Any] | None = None
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

    def discover_targets(self, camera_ids=None):
        if not camera_ids:
            return list(self.targets)
        wanted = set(camera_ids)
        return [target for target in self.targets if target.camera_id in wanted]

    def _broadcast(self, targets, fn_name, **kwargs):
        self.last_broadcast = {
            "fn_name": fn_name,
            "camera_ids": [target.camera_id for target in targets],
            "kwargs": dict(kwargs),
        }
        self.broadcast_history.append(dict(self.last_broadcast))
        if fn_name == "intrinsics_start":
            self.intrinsics_status = {
                **self.intrinsics_status,
                "phase": "capturing",
                "camera_id": kwargs.get("camera_id", self.intrinsics_status["camera_id"]),
                "frames_needed": int(kwargs.get("min_frames", 25)),
                "last_error": None,
            }
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "intrinsics_stop":
            self.intrinsics_status = {**self.intrinsics_status, "phase": "idle"}
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "intrinsics_clear":
            self.intrinsics_status = {
                **self.intrinsics_status,
                "frames_captured": 0,
                "frames_rejected_cooldown": 0,
                "frames_rejected_spatial": 0,
                "frames_rejected_detection": 0,
                "grid_coverage": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                "last_error": None,
            }
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "intrinsics_get_corners":
            frames = kwargs.get("frames_override", [])
            start_index = int(kwargs.get("start_index", 0) or 0)
            max_frames = kwargs.get("max_frames")
            if isinstance(max_frames, int) and max_frames > 0:
                frames = frames[start_index:start_index + max_frames]
            else:
                frames = frames[start_index:]
            payload = {
                "frames": frames,
                "count": int(kwargs.get("count_override", self.intrinsics_status["frames_captured"])),
                "start_index": start_index,
                "returned_count": len(frames),
                "image_size": None,
                "phase": self.intrinsics_status["phase"],
                "frames_captured": self.intrinsics_status["frames_captured"],
                "frames_rejected_cooldown": self.intrinsics_status["frames_rejected_cooldown"],
                "frames_rejected_spatial": self.intrinsics_status["frames_rejected_spatial"],
                "frames_rejected_detection": self.intrinsics_status["frames_rejected_detection"],
                "grid_coverage": self.intrinsics_status["grid_coverage"],
                "last_error": self.intrinsics_status["last_error"],
            }
            return {target.camera_id: {"ack": True, "result": payload} for target in targets}
        if fn_name == "intrinsics_status":
            return {target.camera_id: {"ack": True, "result": dict(self.intrinsics_status)} for target in targets}
        if fn_name == "ping":
            return {
                target.camera_id: {
                    "ack": True,
                    "result": {
                        "state": "IDLE",
                        "supported_commands": list(self.supported_commands),
                        "blob_diagnostics": {
                            "last_blob_count": 1,
                            "rejected_by_diameter": 0,
                            "rejected_by_circularity": 0,
                        },
                    },
                }
                for target in targets
            }
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


def test_intrinsics_host_session_polls_status_immediately_on_start(tmp_path: Path) -> None:
    calls: list[str] = []

    def _broadcast(fn_name: str, **kwargs: Any) -> Dict[str, Any]:
        _ = kwargs
        calls.append(fn_name)
        if fn_name == "intrinsics_start":
            return {"phase": "capturing"}
        if fn_name == "intrinsics_get_corners":
            return {
                "frames": [],
                "count": 0,
                "image_size": None,
                "phase": "capturing",
                "frames_rejected_cooldown": 1,
                "frames_rejected_spatial": 2,
                "frames_rejected_detection": 3,
                "grid_coverage": [[0, 1, 0], [0, 0, 0], [0, 0, 0]],
                "last_error": None,
            }
        if fn_name == "intrinsics_stop":
            return {"phase": "idle"}
        raise AssertionError(f"unexpected command: {fn_name}")

    session = IntrinsicsHostSession(
        IntrinsicsHostSessionConfig(
            camera_id="pi-cam-01",
            mjpeg_url="",
            square_length_mm=30.0,
            marker_length_mm=22.5,
            squares_x=6,
            squares_y=8,
            min_frames=25,
            poll_interval_s=10.0,
            output_dir=tmp_path,
        ),
        _broadcast,
    )

    session.start()
    status = session.get_status()
    session.stop()

    assert calls[:2] == ["intrinsics_start", "intrinsics_get_corners"]
    assert status["phase"] == "capturing"
    assert status["frames_rejected_cooldown"] == 1
    assert status["frames_rejected_spatial"] == 2
    assert status["frames_rejected_detection"] == 3


def test_intrinsics_stop_returns_idle_status(tmp_path: Path) -> None:
    session = _FakeSession()
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "square_length_mm": 60,
                "marker_length_mm": 45,
                "squares_x": 6,
                "squares_y": 8,
                "min_frames": 25,
                "cooldown_s": 1.5,
            }
        }
    )

    _ = state.start_intrinsics_capture({})
    stopped = state.stop_intrinsics_capture()

    assert stopped["ok"] is True
    assert stopped["status"]["phase"] == "idle"


def test_intrinsics_clear_returns_reset_status(tmp_path: Path) -> None:
    session = _FakeSession()
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "square_length_mm": 60,
                "marker_length_mm": 45,
                "squares_x": 6,
                "squares_y": 8,
                "min_frames": 25,
                "cooldown_s": 1.5,
            }
        }
    )

    _ = state.start_intrinsics_capture({})
    session.intrinsics_status = {
        **session.intrinsics_status,
        "frames_captured": 9,
        "frames_rejected_cooldown": 10,
        "frames_rejected_spatial": 2,
        "frames_rejected_detection": 3,
    }

    cleared = state.clear_intrinsics_frames()

    assert cleared["ok"] is True
    assert cleared["status"]["frames_captured"] == 0
    assert cleared["status"]["frames_rejected_cooldown"] == 0
    assert cleared["status"]["frames_rejected_spatial"] == 0
    assert cleared["status"]["frames_rejected_detection"] == 0


def test_gui_pose_and_floor_capture_paths(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_WAND_METRIC_LOG_PATH", tmp_path / "extrinsics_wand_metric.jsonl")
    receiver = _FakeReceiver()
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=receiver,
        settings_path=tmp_path / "loutrack_gui_settings.json",
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
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_WAND_METRIC_LOG_PATH", tmp_path / "extrinsics_wand_metric.jsonl")
    runtime = _FakeTrackingRuntime()
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
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
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
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
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )

    result = state.generate_extrinsics({"pose_log_path": str(tmp_path / "missing_pose.jsonl")})

    summary = result["generate_extrinsics"]
    assert summary["ok"] is False
    assert "log_path does not exist" in summary["error"]
    assert summary["pose_log_summary"]["usable_camera_count"] == 0


def test_workflow_marks_existing_pose_and_metric_logs_complete(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_EXTRINSICS_OUTPUT_PATH", tmp_path / "extrinsics_pose_v2.json")
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_WAND_METRIC_LOG_PATH", tmp_path / "extrinsics_wand_metric.jsonl")
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
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


def test_settings_draft_updates_committed_and_returns_validation(tmp_path: Path) -> None:
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    settings = state.get_settings()
    assert settings["calibration"]["committed"]["exposure_us"] == 12000

    invalid = state.apply_settings_draft({"intrinsics": {"square_length_mm": 0}})
    assert "square_length_mm" in invalid["validation"]["intrinsics"]
    assert invalid["intrinsics"]["committed"]["square_length_mm"] == 30.0

    updated = state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-02",
                "mjpeg_url": "http://192.168.8.100:8555/mjpeg",
                "square_length_mm": 35.0,
                "squares_x": 7,
                "squares_y": 9,
            }
        }
    )
    assert updated["validation"]["intrinsics"] == {}
    assert updated["intrinsics"]["committed"]["camera_id"] == "pi-cam-02"
    assert updated["intrinsics"]["committed"]["square_length_mm"] == 35.0
    assert updated["intrinsics"]["committed"]["squares_x"] == 7
    assert updated["intrinsics"]["committed"]["squares_y"] == 9


def test_settings_manual_log_lock_and_reset_to_runtime_hint(tmp_path: Path) -> None:
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    locked = state.apply_settings_draft({"extrinsics": {"pose_log_path": "logs/manual_pose.jsonl"}})
    assert locked["extrinsics"]["locks"]["pose_log_path_manual"] is True
    assert locked["extrinsics"]["draft"]["pose_log_path"] == "logs/manual_pose.jsonl"

    state._update_runtime_hints(pose_log_path="logs/runtime_pose.jsonl")
    after_runtime = state.get_settings()
    assert after_runtime["runtime_hints"]["pose_log_path"] == "logs/runtime_pose.jsonl"
    assert after_runtime["extrinsics"]["draft"]["pose_log_path"] == "logs/manual_pose.jsonl"

    reset = state.apply_settings_draft({"reset_runtime": ["pose_log_path"]})
    assert reset["extrinsics"]["locks"]["pose_log_path_manual"] is False
    assert reset["extrinsics"]["draft"]["pose_log_path"] == "logs/runtime_pose.jsonl"


def test_get_state_no_longer_reloads_config_from_settings_file(tmp_path: Path) -> None:
    settings_path = tmp_path / "loutrack_gui_settings.json"
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=settings_path,
        tracking_runtime=_FakeTrackingRuntime(),
    )
    baseline = state.get_state()["config"]["exposure_us"]
    file_payload = json.loads(settings_path.read_text(encoding="utf-8"))
    file_payload["calibration"]["committed"]["exposure_us"] = 9999
    file_payload["calibration"]["draft"]["exposure_us"] = 9999
    settings_path.write_text(json.dumps(file_payload), encoding="utf-8")

    current = state.get_state()["config"]["exposure_us"]
    assert current == baseline


def test_set_preview_forwards_overlays_and_charuco(tmp_path: Path) -> None:
    session = _FakeSession()
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.run_command(
        {
            "command": "set_preview",
            "camera_ids": ["pi-cam-01"],
            "render_enabled": True,
            "overlays": {"blob": False, "mask": False, "text": True, "charuco": True},
            "charuco": {
                "dictionary": "DICT_6X6_250",
                "squares_x": 6,
                "squares_y": 8,
                "square_length_mm": 60.0,
            },
        }
    )
    assert session.last_broadcast is not None
    assert session.last_broadcast["fn_name"] == "set_preview"
    assert session.last_broadcast["camera_ids"] == ["pi-cam-01"]
    kwargs = session.last_broadcast["kwargs"]
    assert kwargs["render_enabled"] is True
    assert kwargs["overlays"] == {"blob": False, "mask": False, "text": True, "charuco": True}
    assert kwargs["charuco"]["square_length_mm"] == 60.0


def test_intrinsics_start_sends_start_to_pi_and_creates_session(tmp_path: Path) -> None:
    session = _FakeSession()
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "mjpeg_url": "http://192.168.1.101:8555/mjpeg",
                "square_length_mm": 60,
                "marker_length_mm": 45,
                "squares_x": 6,
                "squares_y": 8,
                "min_frames": 25,
                "cooldown_s": 1.5,
            }
        }
    )

    resp = state.start_intrinsics_capture({})

    assert resp["ok"] is True
    assert state._intrinsics_host_session is not None
    assert session.last_broadcast is not None
    start_call = next(item for item in session.broadcast_history if item["fn_name"] == "intrinsics_start")
    assert start_call["camera_ids"] == ["pi-cam-01"]
    kwargs = start_call["kwargs"]
    assert kwargs["square_length_mm"] == 60.0
    assert kwargs["marker_length_mm"] == 45.0
    assert kwargs["squares_x"] == 6
    assert kwargs["squares_y"] == 8
    assert kwargs["min_frames"] == 25
    assert kwargs["cooldown_s"] == 1.5
    state._intrinsics_host_session.stop()


def test_intrinsics_start_allows_empty_mjpeg_url(tmp_path: Path) -> None:
    session = _FakeSession()
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    updated = state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "mjpeg_url": "",
                "square_length_mm": 60,
                "squares_x": 6,
                "squares_y": 8,
                "min_frames": 25,
                "cooldown_s": 1.5,
            }
        }
    )
    assert "mjpeg_url" not in updated["validation"]["intrinsics"]

    resp = state.start_intrinsics_capture({})

    assert resp["ok"] is True
    assert session.last_broadcast is not None
    assert any(item["fn_name"] == "intrinsics_start" for item in session.broadcast_history)
    state._intrinsics_host_session.stop()


def test_intrinsics_start_rejects_pi_missing_get_corners_command(tmp_path: Path) -> None:
    session = _FakeSession()
    # Has intrinsics_start/stop/clear/status but not intrinsics_get_corners
    session.supported_commands = [
        "ping", "start", "stop", "set_preview",
        "intrinsics_start", "intrinsics_stop", "intrinsics_clear", "intrinsics_status",
    ]
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
    state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "mjpeg_url": "http://192.168.1.101:8555/mjpeg",
                "square_length_mm": 30,
            }
        }
    )

    try:
        state.start_intrinsics_capture({})
    except ValueError as exc:
        assert "intrinsics capability missing on Pi" in str(exc)
        assert "intrinsics_get_corners" in str(exc)
    else:
        raise AssertionError("expected ValueError when intrinsics_get_corners is missing")


def test_intrinsics_calibrate_triggers_host_session(tmp_path: Path, monkeypatch) -> None:
    session = _FakeSession()
    state = LoutrackGuiState(
        session=session,
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )

    calibrate_called: list = []

    class _FakeHostSession:
        def start(self) -> None:
            pass

        def stop(self) -> None:
            pass

        def clear(self) -> None:
            pass

        def trigger_calibration(self) -> None:
            calibrate_called.append(True)

        def get_status(self) -> Dict[str, Any]:
            return {
                "phase": "capturing",
                "camera_id": "pi-cam-01",
                "frames_captured": 0,
                "frames_needed": 25,
                "frames_target": 50,
                "frames_rejected_cooldown": 0,
                "frames_rejected_spatial": 0,
                "frames_rejected_detection": 0,
                "grid_coverage": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                "last_error": None,
                "calibration_result": None,
                "output_path": None,
            }

        def get_latest_jpeg(self) -> None:
            return None

    monkeypatch.setattr(
        "host.loutrack_gui.IntrinsicsHostSession",
        lambda config, fn: _FakeHostSession(),
    )

    state.apply_settings_draft(
        {
            "intrinsics": {
                "camera_id": "pi-cam-01",
                "mjpeg_url": "http://192.168.1.101:8555/mjpeg",
                "square_length_mm": 30,
            }
        }
    )
    _ = state.start_intrinsics_capture({})
    resp = state.trigger_intrinsics_calibration()

    assert resp["ok"] is True
    assert calibrate_called


def test_intrinsics_calibration_syncs_remote_frames_before_min_frame_check(
    tmp_path: Path,
    monkeypatch,
) -> None:
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
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
    _ = state.start_intrinsics_capture({})
    host_session = state._intrinsics_host_session
    assert host_session is not None

    remote_frame = {
        "corners": [[[10.0, 20.0]], [[30.0, 40.0]], [[50.0, 60.0]], [[70.0, 80.0]], [[90.0, 100.0]], [[110.0, 120.0]]],
        "ids": [[0], [1], [2], [3], [4], [5]],
    }
    original_broadcast = state.session._broadcast

    def _broadcast_with_frame(targets, fn_name, **kwargs):
        response = original_broadcast(targets, fn_name, **kwargs)
        if fn_name == "intrinsics_get_corners":
            return {
                target.camera_id: {
                    "ack": True,
                    "result": {
                        **response[target.camera_id]["result"],
                        "frames": [remote_frame, remote_frame, remote_frame, remote_frame, remote_frame][
                            int(kwargs.get("start_index", 0) or 0):
                        ][: int(kwargs.get("max_frames", 8) or 8)],
                        "count": 5,
                        "returned_count": len(
                            [remote_frame, remote_frame, remote_frame, remote_frame, remote_frame][
                                int(kwargs.get("start_index", 0) or 0):
                            ][: int(kwargs.get("max_frames", 8) or 8)]
                        ),
                        "image_size": [1280, 960],
                        "phase": "capturing",
                    },
                }
                for target in targets
            }
        return response

    state.session._broadcast = _broadcast_with_frame  # type: ignore[method-assign]

    called = {"count": 0}

    def _fake_run_calibration(corners, ids, image_size):
        called["count"] += 1
        assert len(corners) == 5
        assert len(ids) == 5
        assert image_size == (1280, 960)

    monkeypatch.setattr(host_session, "_run_calibration", _fake_run_calibration)

    resp = state.trigger_intrinsics_calibration()

    assert resp["ok"] is True
    assert called["count"] == 1


def test_intrinsics_calibration_after_stop_syncs_remote_frames(
    tmp_path: Path,
    monkeypatch,
) -> None:
    state = LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "loutrack_gui_settings.json",
        tracking_runtime=_FakeTrackingRuntime(),
    )
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
    _ = state.start_intrinsics_capture({})
    _ = state.stop_intrinsics_capture()
    host_session = state._intrinsics_host_session
    assert host_session is not None

    remote_frame = {
        "corners": [[[10.0, 20.0]], [[30.0, 40.0]], [[50.0, 60.0]], [[70.0, 80.0]], [[90.0, 100.0]], [[110.0, 120.0]]],
        "ids": [[0], [1], [2], [3], [4], [5]],
    }
    original_broadcast = state.session._broadcast

    def _broadcast_with_stopped_frames(targets, fn_name, **kwargs):
        response = original_broadcast(targets, fn_name, **kwargs)
        if fn_name == "intrinsics_get_corners":
            return {
                target.camera_id: {
                    "ack": True,
                    "result": {
                        **response[target.camera_id]["result"],
                        "frames": [remote_frame, remote_frame, remote_frame, remote_frame, remote_frame][
                            int(kwargs.get("start_index", 0) or 0):
                        ][: int(kwargs.get("max_frames", 8) or 8)],
                        "count": 5,
                        "returned_count": len(
                            [remote_frame, remote_frame, remote_frame, remote_frame, remote_frame][
                                int(kwargs.get("start_index", 0) or 0):
                            ][: int(kwargs.get("max_frames", 8) or 8)]
                        ),
                        "image_size": [1280, 960],
                        "phase": "idle",
                    },
                }
                for target in targets
            }
        return response

    state.session._broadcast = _broadcast_with_stopped_frames  # type: ignore[method-assign]

    called = {"count": 0}

    def _fake_run_calibration(corners, ids, image_size):
        called["count"] += 1
        assert len(corners) == 5
        assert len(ids) == 5
        assert image_size == (1280, 960)

    monkeypatch.setattr(host_session, "_run_calibration", _fake_run_calibration)

    resp = state.trigger_intrinsics_calibration()

    assert resp["ok"] is True
    assert called["count"] == 1


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
    monkeypatch.setattr("host.loutrack_gui.STATIC_DIR_CANDIDATES", (repo_static, module_static))
    resolved = _resolve_static_asset("vendor/three.module.min.js")
    assert resolved == repo_asset
