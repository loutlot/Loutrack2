import sys
from pathlib import Path

import numpy as np


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


class _FakeRequest:
    def __init__(self, frame: np.ndarray, metadata: dict[str, object]) -> None:
        self._frame = frame
        self._metadata = metadata
        self.released = False

    def make_array(self, stream: str = "main") -> np.ndarray:
        _ = stream
        return self._frame

    def get_metadata(self) -> dict[str, object]:
        return dict(self._metadata)

    def release(self) -> None:
        self.released = True


class _FakePicamera2CaptureRequest:
    def __init__(self, frame: np.ndarray, metadata: dict[str, object]) -> None:
        self.request = _FakeRequest(frame, metadata)

    def capture_request(self) -> _FakeRequest:
        return self.request


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


def test_picamera2_backend_prefers_sensor_metadata_timestamp(monkeypatch) -> None:
    backend = Picamera2Backend()
    frame = np.zeros((12, 16, 3), dtype=np.uint8)
    fake_picam2 = _FakePicamera2CaptureRequest(frame, {"SensorTimestamp": 2_000_000_000})
    backend._picam2 = fake_picam2  # type: ignore[assignment]
    backend._running = True

    monkeypatch.setattr("pi.capture._clock_realtime_us", lambda: 5_000_000)
    monkeypatch.setattr("pi.capture._clock_monotonic_us", lambda: 4_000_000)

    captured = backend.next_captured_frame()

    assert captured.image.shape == frame.shape
    assert captured.timestamp_source == "sensor_metadata"
    assert captured.sensor_timestamp_ns == 2_000_000_000
    assert captured.timestamp_us == 3_000_000
    assert fake_picam2.request.released is True
