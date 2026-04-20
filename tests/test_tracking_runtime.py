from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np

from host.geo import CameraParams
from host.rigid import MarkerPattern, RigidBodyPose
from host.tracking_runtime import (
    TrackingRuntime,
    compute_frustum_near_corners_world,
    compute_markers_world,
)


def test_compute_frustum_near_corners_world_identity_camera() -> None:
    camera = CameraParams(
        camera_id="cam-01",
        intrinsic_matrix=np.array(
            [[100.0, 0.0, 320.0], [0.0, 100.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        distortion_coeffs=np.zeros(5, dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        translation=np.zeros(3, dtype=np.float64),
        resolution=(640, 480),
    )

    corners = compute_frustum_near_corners_world(camera, z_near=0.5)
    expected = np.array(
        [
            [-1.6, -1.2, 0.5],
            [1.6, -1.2, 0.5],
            [1.6, 1.2, 0.5],
            [-1.6, 1.2, 0.5],
        ],
        dtype=np.float64,
    )

    assert np.allclose(np.array(corners), expected)


def test_compute_markers_world_applies_rigid_transform() -> None:
    pattern = MarkerPattern(
        name="test",
        marker_positions=np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]], dtype=np.float64),
    )
    rotation = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    pose = RigidBodyPose(
        timestamp=100,
        position=np.array([10.0, 20.0, 30.0], dtype=np.float64),
        rotation=rotation,
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.0,
        observed_markers=2,
        valid=True,
    )

    markers_world = compute_markers_world(pose, pattern)
    expected = np.array([[10.0, 21.0, 30.0], [8.0, 20.0, 30.0]], dtype=np.float64)
    assert np.allclose(np.array(markers_world), expected)


def test_tracking_runtime_start_stop_state_transition(monkeypatch) -> None:
    class _FakePipeline:
        last_instance = None

        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self.udp_port = udp_port
            self.calibration_path = calibration_path
            self.patterns = patterns or []
            self._running = False
            self._pose_callback = None
            self.geometry = type("_Geometry", (), {"camera_params": {}})()
            _FakePipeline.last_instance = self

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {"frames_processed": 12, "poses_estimated": 11}

        def get_status(self):
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": 12,
                "poses_estimated": 11,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 0,
                "points_3d": [],
                "reprojection_errors": [],
                "pair_timestamp_range_us": 0,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime(udp_port=7000)
    start_status = runtime.start(calibration_path="calibration", patterns=["waist"])
    assert start_status["running"] is True
    assert _FakePipeline.last_instance is not None
    assert _FakePipeline.last_instance.udp_port == 7000
    assert _FakePipeline.last_instance.calibration_path == "calibration"

    stop_result = runtime.stop()
    assert stop_result["frames_processed"] == 12
    assert stop_result["poses_estimated"] == 11

    status_after_stop = runtime.status()
    assert status_after_stop["running"] is False
    assert status_after_stop["calibration_loaded"] is True
    assert status_after_stop["patterns"] == ["waist"]
    assert status_after_stop["last_stop_summary"]["frames_processed"] == 12


def test_tracking_runtime_scene_resets_after_stop(monkeypatch) -> None:
    class _FakePipeline:
        last_instance = None

        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = False
            self.geometry = type("_Geometry", (), {"camera_params": {}})()
            _FakePipeline.last_instance = self

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {"frames_processed": 9, "poses_estimated": 7}

        def get_status(self):
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": 9,
                "poses_estimated": 7,
                "receiver": {},
                "metrics": {},
                "tracking": {},
                "sync": {},
                "uptime_seconds": 0.1,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 123456,
                "points_3d": [[1.0, 2.0, 3.0]],
                "reprojection_errors": [0.4],
                "pair_timestamp_range_us": 800,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    runtime.start(calibration_path="calibration", patterns=["waist"])
    pose = RigidBodyPose(
        timestamp=123456,
        position=np.array([1.0, 2.0, 3.0], dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.2,
        observed_markers=3,
        valid=True,
    )
    runtime._on_pose({"waist": pose})
    scene_running = runtime.scene_snapshot()
    assert scene_running["tracking"]["running"] is True
    assert len(scene_running["rigid_bodies"]) == 1
    assert len(scene_running["raw_points"]) == 1

    runtime.stop()
    scene_stopped = runtime.scene_snapshot()
    assert scene_stopped["tracking"]["running"] is False
    assert scene_stopped["rigid_bodies"] == []
    assert scene_stopped["raw_points"] == []


def test_tracking_runtime_updates_scene_even_without_valid_rigid_body(monkeypatch) -> None:
    class _FakePipeline:
        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = False
            self.geometry = type("_Geometry", (), {"camera_params": {}})()

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {"frames_processed": 3, "poses_estimated": 0}

        def get_status(self):
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": 3,
                "poses_estimated": 0,
                "receiver": {},
                "metrics": {},
                "tracking": {},
                "sync": {},
                "uptime_seconds": 0.1,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 789,
                "points_3d": [[4.0, 5.0, 6.0]],
                "reprojection_errors": [0.2],
                "pair_timestamp_range_us": 600,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    runtime.start(calibration_path="calibration", patterns=["waist"])
    initial_sequence = runtime.scene_snapshot()["sequence"]
    runtime._on_pose({})

    scene = runtime.scene_snapshot()
    waited_scene = runtime.wait_for_scene_update(initial_sequence, timeout=0.01)
    assert scene["tracking"]["running"] is True
    assert scene["rigid_bodies"] == []
    assert scene["raw_points"] == [[4.0, 5.0, 6.0]]
    assert scene["sequence"] > initial_sequence
    assert waited_scene["sequence"] == scene["sequence"]
    assert scene["coordinate_origin"] == "reference_camera"
    assert scene["coordinate_origin_source"] == "extrinsics_pose_reference"


def test_tracking_runtime_status_uses_short_lived_cache(monkeypatch) -> None:
    class _FakePipeline:
        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = True
            self.geometry = type("_Geometry", (), {"camera_params": {}})()
            self.get_status_calls = 0
            self.frames_processed = 0
            self.poses_estimated = 0

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {"frames_processed": 0, "poses_estimated": 0}

        @property
        def is_running(self):
            return self._running

        def get_status(self):
            self.get_status_calls += 1
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": self.frames_processed,
                "poses_estimated": self.poses_estimated,
                "receiver": {},
                "metrics": {},
                "tracking": {},
                "sync": {},
                "uptime_seconds": 0.1,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 0,
                "points_3d": [],
                "reprojection_errors": [],
                "pair_timestamp_range_us": 0,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    runtime.start(calibration_path="calibration", patterns=["waist"])
    runtime._status_cache_interval_s = 10.0
    pipeline = runtime._pipeline
    assert pipeline is not None

    first = runtime.status()
    second = runtime.status()

    assert first["running"] is True
    assert second["running"] is True
    assert pipeline.get_status_calls == 1


def test_tracking_runtime_reuses_cached_camera_scene_on_pose_updates(monkeypatch) -> None:
    class _FakePipeline:
        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = False
            self.frames_processed = 0
            self.poses_estimated = 0
            self.geometry = type("_Geometry", (), {"camera_params": {}})()

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {"frames_processed": 1, "poses_estimated": 1}

        @property
        def is_running(self):
            return self._running

        def get_status(self):
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": self.frames_processed,
                "poses_estimated": self.poses_estimated,
                "receiver": {},
                "metrics": {},
                "tracking": {},
                "sync": {},
                "uptime_seconds": 0.1,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 456,
                "points_3d": [[1.0, 2.0, 3.0]],
                "reprojection_errors": [],
                "pair_timestamp_range_us": 0,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    build_calls = []

    def _build_camera_scene(pipeline):
        build_calls.append("called")
        return [{"camera_id": "pi-cam-01", "position": [0.0, 0.0, 0.0], "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "frustum_near_corners": []}]

    runtime._build_camera_scene = _build_camera_scene  # type: ignore[method-assign]
    runtime.start(calibration_path="calibration", patterns=["waist"])

    pose = RigidBodyPose(
        timestamp=456,
        position=np.array([1.0, 2.0, 3.0], dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.2,
        observed_markers=3,
        valid=True,
    )
    pipeline = runtime._pipeline
    assert pipeline is not None
    pipeline.frames_processed = 1
    pipeline.poses_estimated = 1
    runtime._on_pose({"waist": pose})

    scene = runtime.scene_snapshot()

    assert build_calls == ["called"]
    assert scene["cameras"] == runtime._camera_scene
    assert scene["tracking"]["frames_processed"] == 1


def test_tracking_runtime_ignores_duplicate_scene_timestamps(monkeypatch) -> None:
    class _FakePipeline:
        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = False
            self.frames_processed = 0
            self.poses_estimated = 0
            self.geometry = type("_Geometry", (), {"camera_params": {}})()

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {"frames_processed": 1, "poses_estimated": 1}

        @property
        def is_running(self):
            return self._running

        def get_status(self):
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": self.frames_processed,
                "poses_estimated": self.poses_estimated,
                "receiver": {},
                "metrics": {},
                "tracking": {},
                "sync": {},
                "uptime_seconds": 0.1,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 789,
                "points_3d": [[1.0, 2.0, 3.0]],
                "reprojection_errors": [],
                "pair_timestamp_range_us": 0,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    runtime.start(calibration_path="calibration", patterns=["waist"])

    pose = RigidBodyPose(
        timestamp=789,
        position=np.array([1.0, 2.0, 3.0], dtype=np.float64),
        rotation=np.eye(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rms_error=0.2,
        observed_markers=3,
        valid=True,
    )
    runtime._on_pose({"waist": pose})
    first_sequence = runtime.scene_snapshot()["sequence"]
    runtime._on_pose({"waist": pose})

    assert runtime.scene_snapshot()["sequence"] == first_sequence


def test_tracking_runtime_scene_reports_wand_origin_from_geometry(monkeypatch) -> None:
    class _FakePipeline:
        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = False
            self.geometry = type(
                "_Geometry",
                (),
                {
                    "camera_params": {},
                    "coordinate_frame": "world",
                    "coordinate_origin": "wand",
                    "coordinate_origin_source": "floor_metric_capture",
                },
            )()

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {}

        def get_status(self):
            return {"running": self._running, "calibration_loaded": True}

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 321,
                "points_3d": [[0.0, 0.0, 0.0]],
                "reprojection_errors": [],
                "pair_timestamp_range_us": 0,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    runtime.start(calibration_path="calibration", patterns=["waist"])
    runtime._on_pose({})

    scene = runtime.scene_snapshot()

    assert scene["coordinate_frame"] == "world"
    assert scene["coordinate_origin"] == "wand"
    assert scene["coordinate_origin_source"] == "floor_metric_capture"


def test_tracking_runtime_reuses_status_cache_and_camera_scene(monkeypatch) -> None:
    class _FakePipeline:
        last_instance = None

        def __init__(self, udp_port=5000, calibration_path=None, patterns=None, **kwargs):
            self._running = False
            self.status_calls = 0
            self.geometry = type(
                "_Geometry",
                (),
                {
                    "camera_params": {},
                    "coordinate_frame": "camera_similarity",
                    "coordinate_origin": "reference_camera",
                    "coordinate_origin_source": "extrinsics_pose_reference",
                },
            )()
            _FakePipeline.last_instance = self

        def set_pose_callback(self, callback):
            self._pose_callback = callback

        def start(self, session_name=None):
            self._running = True

        def stop(self):
            self._running = False
            return {}

        def get_status(self):
            self.status_calls += 1
            return {
                "running": self._running,
                "calibration_loaded": True,
                "frames_processed": 5,
                "poses_estimated": 4,
                "receiver": {},
                "metrics": {},
                "tracking": {},
                "sync": {},
                "uptime_seconds": 0.1,
            }

        def get_latest_triangulation_snapshot(self):
            return {
                "timestamp": 456,
                "points_3d": [[1.0, 1.0, 1.0]],
                "reprojection_errors": [],
                "pair_timestamp_range_us": 0,
            }

    monkeypatch.setattr("host.tracking_runtime.TrackingPipeline", _FakePipeline)

    runtime = TrackingRuntime()
    build_calls = {"count": 0}
    original_build_camera_scene = runtime._build_camera_scene

    def _count_build_camera_scene(pipeline):
        build_calls["count"] += 1
        return original_build_camera_scene(pipeline)

    runtime._build_camera_scene = _count_build_camera_scene  # type: ignore[method-assign]
    runtime._status_cache_interval_s = 10.0

    runtime.start(calibration_path="calibration", patterns=["waist"])
    runtime._on_pose({})
    runtime._on_pose({})
    status = runtime.status()

    assert build_calls["count"] == 1
    assert _FakePipeline.last_instance is not None
    assert _FakePipeline.last_instance.status_calls == 1
    assert status["frames_processed"] == 5
