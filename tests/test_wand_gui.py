from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.wand_gui import HTML_PAGE, WandGuiState
from host.wand_session import CameraTarget


class _FakeReceiver:
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


def test_gui_state_apply_and_command() -> None:
    state = WandGuiState(session=_FakeSession(), receiver=_FakeReceiver())

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

    result = state.run_command({"command": "ping", "camera_ids": ["pi-cam-01"]})
    assert "ping" in result

    mask_cleared = state.run_command({"command": "mask_stop", "camera_ids": ["pi-cam-01"]})
    assert "mask_stop" in mask_cleared

    state._extrinsics_solver = lambda **kwargs: {  # type: ignore[attr-defined]
        "reference_camera_id": "pi-cam-01",
        "cameras": [{"camera_id": "pi-cam-01"}, {"camera_id": "pi-cam-02"}],
    }
    generated = state.generate_extrinsics(
        {
            "intrinsics_path": "calibration",
            "log_path": "logs/wand_capture.jsonl",
            "output_path": "calibration/calibration_extrinsics_v1.json",
        }
    )
    assert generated["generate_extrinsics"]["ok"] is True
    assert generated["generate_extrinsics"]["camera_count"] == 2

    snapshot = state.get_state()
    assert snapshot["workflow"]["extrinsics_ready"] is True


if __name__ == "__main__":
    test_gui_state_apply_and_command()
    print("wand_gui tests passed")
