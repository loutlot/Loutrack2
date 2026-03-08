from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from pathlib import Path

from host.wand_gui import HTML_PAGE, WandGuiState
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


def test_gui_state_apply_and_command(tmp_path: Path) -> None:
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
    )

    snapshot = state.get_state()
    assert len(snapshot["cameras"]) == 2
    assert snapshot["workflow"]["active_segment"] == "mask"
    assert "Blob Detection Adjustment" in HTML_PAGE
    assert "Mask Adjustment" in HTML_PAGE
    assert "Wand Capture" in HTML_PAGE
    assert "Extrinsics Generation" in HTML_PAGE

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

    snapshot = state.get_state()
    assert snapshot["workflow"]["extrinsics_ready"] is True


def test_gui_html_normalizes_blob_range() -> None:
    assert "function normalizedBlobRange()" in HTML_PAGE
    assert "maxValue = minValue;" in HTML_PAGE


def test_generate_extrinsics_rejects_missing_log_path(tmp_path: Path) -> None:
    state = WandGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "wand_gui_settings.json",
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


def test_start_stop_writes_wand_capture_log(tmp_path: Path) -> None:
    receiver = _FakeReceiver()
    state = WandGuiState(
        session=_FakeSession(),
        receiver=receiver,
        settings_path=tmp_path / "wand_gui_settings.json",
    )
    state.capture_log_dir = tmp_path / "logs"

    start_result = state.run_command({"command": "start", "camera_ids": ["pi-cam-01"]})
    log_path = Path(start_result["capture_log"]["path"])
    assert log_path.exists()

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

    lines = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    assert any(item.get("_type") == "frame" for item in lines)
    assert any(
        item.get("_type") == "frame"
        and item.get("data", {}).get("camera_id") == "pi-cam-01"
        for item in lines
    )


def test_gui_loads_persisted_settings(tmp_path: Path) -> None:
    path = tmp_path / "wand_gui_settings.json"
    path.write_text(
        '{"exposure_us": 15000, "gain": 9.0, "fps": 56, "threshold": 215, "mask_threshold": 207, "mask_seconds": 0.8}',
        encoding="utf-8",
    )
    state = WandGuiState(session=_FakeSession(), receiver=_FakeReceiver(), settings_path=path)
    snapshot = state.get_state()
    assert snapshot["config"]["exposure_us"] == 15000
    assert snapshot["config"]["gain"] == 9.0
    assert snapshot["config"]["threshold"] == 215


if __name__ == "__main__":
    test_gui_state_apply_and_command()
    print("wand_gui tests passed")
