from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.wand_session import WAND_POINTS_MM, SessionConfig, WandSession


class _FakeReceiver:
    def __init__(self, addresses):
        self._addresses = addresses

    def get_camera_addresses(self):
        return dict(self._addresses)


class _FakeControl:
    def __init__(self):
        self.calls = []

    def _resp(self, name, camera_id, timeout):
        self.calls.append((name, camera_id, timeout))
        return {"ack": True}

    def ping(self, ip, port, camera_id, timeout=2.0):
        return self._resp("ping", camera_id, timeout)

    def set_exposure(self, ip, port, camera_id, value, timeout=2.0):
        return self._resp("set_exposure", camera_id, timeout)

    def set_gain(self, ip, port, camera_id, value, timeout=2.0):
        return self._resp("set_gain", camera_id, timeout)

    def set_fps(self, ip, port, camera_id, value, timeout=2.0):
        return self._resp("set_fps", camera_id, timeout)

    def set_focus(self, ip, port, camera_id, value, timeout=2.0):
        return self._resp("set_focus", camera_id, timeout)

    def set_threshold(self, ip, port, camera_id, value, timeout=2.0):
        return self._resp("set_threshold", camera_id, timeout)

    def set_blob_diameter(self, ip, port, camera_id, min_px=None, max_px=None, timeout=2.0):
        return self._resp("set_blob_diameter", camera_id, timeout)

    def set_circularity_min(self, ip, port, camera_id, value, timeout=2.0):
        return self._resp("set_circularity_min", camera_id, timeout)

    def mask_start(self, ip, port, camera_id, timeout=2.0, **kwargs):
        return self._resp("mask_start", camera_id, timeout)

    def start(self, ip, port, camera_id, mode, timeout=2.0):
        return self._resp("start", camera_id, timeout)

    def stop(self, ip, port, camera_id, timeout=2.0):
        return self._resp("stop", camera_id, timeout)


def test_wand_points_mm_defaults() -> None:
    assert WAND_POINTS_MM == (
        (0.0, 0.0, 0.0),
        (168.0, 0.0, 0.0),
        (84.0, 121.5, 0.0),
        (0.0, 243.0, 0.0),
    )


def test_discovery_prefers_passive_ip(tmp_path: Path) -> None:
    inventory = tmp_path / "hosts.ini"
    inventory.write_text("pi-cam-01 192.168.1.101 pi-cam-01\n", encoding="utf-8")
    receiver = _FakeReceiver({"pi-cam-01": ("192.168.1.250", 5000)})

    session = WandSession(inventory_path=inventory, receiver=receiver, control=_FakeControl())
    targets = session.discover_targets()

    assert len(targets) == 1
    assert targets[0].camera_id == "pi-cam-01"
    assert targets[0].ip == "192.168.1.250"


def test_run_session_control_order(monkeypatch, tmp_path: Path) -> None:
    inventory = tmp_path / "hosts.ini"
    inventory.write_text(
        "pi-cam-01 192.168.1.101 pi-cam-01\npi-cam-02 192.168.1.102 pi-cam-02\n",
        encoding="utf-8",
    )
    fake_control = _FakeControl()
    session = WandSession(inventory_path=inventory, control=fake_control)
    monkeypatch.setattr("host.wand_session.time.sleep", lambda _duration: None)

    config = SessionConfig(
        exposure_us=12000,
        gain=8.0,
        fps=56,
        focus=5.215,
        threshold=200,
        blob_min_diameter_px=2.0,
        blob_max_diameter_px=20.0,
        circularity_min=0.4,
        duration_s=0.01,
        output_dir=tmp_path,
    )
    result = session.run_session(config)

    assert [target["camera_id"] for target in result["targets"]] == ["pi-cam-01", "pi-cam-02"]
    assert [step["step"] for step in result["ack_history"]] == [
        "set_exposure",
        "set_gain",
        "set_fps",
        "set_focus",
        "set_threshold",
        "set_blob_diameter",
        "set_circularity_min",
        "mask_start",
        "start",
        "stop",
    ]
    metadata_path = Path(result["metadata_path"])
    assert metadata_path.exists()
    assert metadata_path.parent == tmp_path
    mask_timeouts = [timeout for name, _camera_id, timeout in fake_control.calls if name == "mask_start"]
    assert mask_timeouts == [30.0, 30.0]


if __name__ == "__main__":
    from tempfile import TemporaryDirectory

    test_wand_points_mm_defaults()
    with TemporaryDirectory() as tmp:
        test_discovery_prefers_passive_ip(Path(tmp))
    print("wand_session smoke tests passed")
