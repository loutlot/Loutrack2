from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CalibrationPoint2D:
    x: float
    y: float


@dataclass(frozen=True)
class CalibrationPoint3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class CalibrationFrameMeta:
    camera_id: str
    timestamp_us: int
    frame_index: int


@dataclass(frozen=True)
class CalibrationCameraObservation:
    frame: CalibrationFrameMeta
    points: tuple[CalibrationPoint2D, ...]
