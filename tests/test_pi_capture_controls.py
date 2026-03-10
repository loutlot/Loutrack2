import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pi.capture import Picamera2Backend  # noqa: E402


class _FakePicamera2:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def set_controls(self, controls: dict[str, object]) -> None:
        self.calls.append(dict(controls))


def test_picamera2_backend_disables_auto_for_manual_exposure_and_focus(monkeypatch) -> None:
    class _AfModeEnum:
        Manual = 7

    class _Controls:
        AfModeEnum = _AfModeEnum

    class _Libcamera:
        controls = _Controls()

    backend = Picamera2Backend()
    fake_picam2 = _FakePicamera2()
    backend._picam2 = fake_picam2
    backend._running = True

    monkeypatch.setattr("pi.capture.importlib.import_module", lambda name: _Libcamera())

    backend.set_exposure_us(12000)
    backend.set_gain(4.0)
    backend.set_fps(56)
    backend.set_focus(5.215)

    assert fake_picam2.calls
    controls = fake_picam2.calls[-1]
    assert controls["AeEnable"] is False
    assert controls["ExposureTime"] == 12000
    assert controls["AnalogueGain"] == 4.0
    assert controls["FrameDurationLimits"] == (17857, 17857)
    assert controls["AfMode"] == 7
    assert controls["LensPosition"] == 5.215
