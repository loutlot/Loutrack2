from __future__ import annotations

import time
from typing import Any, Dict, List

from .geo import normalize_epipolar_threshold_px
from .scheduled_start import scheduled_start_kwargs


class GuiTrackingService:
    """Tracking orchestration for the host GUI backend."""

    _TRACKING_BLOCKER_MESSAGES = {
        "extrinsics_missing": "Generate extrinsics first",
        "no_cameras_selected": "Select at least one camera",
        "camera_unhealthy": "One or more selected cameras are offline",
        "mask_missing": "Build masks for the selected cameras first",
    }

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def get_tracking_status(self, cameras: List[Dict[str, Any]] | None = None) -> Dict[str, Any]:
        owner = self._owner
        status = owner.tracking_runtime.status()
        start_blockers = self._tracking_start_blockers(cameras)
        running = bool(status.get("running", False))
        start_allowed = not start_blockers and not running
        return {
            **status,
            "start_allowed": start_allowed,
            "start_blockers": start_blockers,
            "stop_allowed": running,
            "empty_state": None if start_allowed else self._tracking_blocker_message(start_blockers),
            "latest_extrinsics_path": str(owner.latest_extrinsics_path) if owner.latest_extrinsics_path else None,
            "latest_extrinsics_quality": owner.latest_extrinsics_quality,
            "sse": owner.get_tracking_sse_diagnostics()
            if hasattr(owner, "get_tracking_sse_diagnostics")
            else {},
        }

    def get_tracking_scene(self) -> Dict[str, Any]:
        owner = self._owner
        tracking_status = self.get_tracking_status()
        if not bool(tracking_status.get("start_allowed")) and not bool(tracking_status.get("running")):
            return {
                "tracking": {
                    "running": False,
                    "frames_processed": 0,
                    "poses_estimated": 0,
                },
                "cameras": [],
                "rigid_bodies": [],
                "raw_points": [],
                "timestamp_us": int(time.time() * 1_000_000),
                "empty_state": tracking_status.get("empty_state") or "Tracking is not ready",
            }
        return owner.tracking_runtime.scene_snapshot()

    def wait_tracking_scene(self, last_sequence: int | None, timeout: float = 15.0) -> Dict[str, Any]:
        owner = self._owner
        tracking_status = self.get_tracking_status()
        if not bool(tracking_status.get("start_allowed")) and not bool(tracking_status.get("running")):
            return self.get_tracking_scene()
        wait_for_scene = getattr(owner.tracking_runtime, "wait_for_scene_update", None)
        if callable(wait_for_scene):
            return wait_for_scene(last_sequence, timeout=timeout)
        return owner.tracking_runtime.scene_snapshot()

    def start_tracking(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        owner = self._owner
        resolved = owner._resolve_tracking_calibration_path(str(payload.get("calibration_path") or ""))
        patterns = payload.get("patterns", ["waist"])
        if not isinstance(patterns, list):
            patterns = ["waist"]
        epipolar_threshold_px = normalize_epipolar_threshold_px(
            payload.get("epipolar_threshold_px")
        )
        current_status = owner.tracking_runtime.status()
        if bool(current_status.get("running", False)):
            response = {
                "ok": True,
                **current_status,
                "running": True,
                "already_running": True,
                "camera_ids": list(owner._tracking_camera_ids),
                "latest_extrinsics_path": str(owner.latest_extrinsics_path) if owner.latest_extrinsics_path else None,
            }
            owner.last_result = {"tracking_start": response}
            return response
        owner._ensure_calibration_settings_valid()
        targets = self._resolve_tracking_targets(payload)
        camera_ids = [target.camera_id for target in targets]
        config_result = owner._apply_capture_settings(targets)
        optimization = self._prepare_pis_for_tracking(targets)
        self._pause_receiver_for_tracking()
        try:
            status = owner.tracking_runtime.start(
                str(resolved),
                [str(item) for item in patterns],
                epipolar_threshold_px=epipolar_threshold_px,
            )
            start_kwargs = scheduled_start_kwargs("pose_capture")
            stream_start = owner.session._broadcast(targets, "start", **start_kwargs)
            if not self.all_acked_or_already_running(stream_start):
                owner.tracking_runtime.stop()
                self._resume_receiver_after_tracking()
                raise ValueError(f"failed to start tracking camera streams: {stream_start}")
        except Exception:
            owner.tracking_runtime.stop()
            self._resume_receiver_after_tracking()
            raise
        owner._update_camera_status({"tracking_start": stream_start})
        owner._tracking_camera_ids = camera_ids
        response = {
            "ok": True,
            **status,
            "running": True,
            "camera_ids": camera_ids,
            "pi_config": config_result,
            "pi_tracking_optimization": optimization,
            "pi_stream_start": stream_start,
            "scheduled_start_at_us": start_kwargs["start_at_us"],
            "epipolar_threshold_px": epipolar_threshold_px,
        }
        owner.last_result = {"tracking_start": response}
        return response

    def stop_tracking(self) -> Dict[str, Any]:
        owner = self._owner
        stream_stop: Dict[str, Any] = {}
        camera_ids = list(owner._tracking_camera_ids)
        if camera_ids:
            targets = owner._discover_workflow_targets(camera_ids)
            if targets:
                stream_stop = owner.session._broadcast(targets, "stop")
        stop_result = owner.tracking_runtime.stop()
        self._resume_receiver_after_tracking()
        summary = stop_result.get("summary", stop_result) if isinstance(stop_result, dict) else {}
        status = owner.tracking_runtime.status()
        owner._update_camera_status({"tracking_stop": stream_stop})
        owner._tracking_camera_ids = []
        response = {"ok": True, "summary": summary, **status, "running": False, "pi_stream_stop": stream_stop}
        owner.last_result = {"tracking_stop": response}
        return response

    def _resolve_tracking_targets(self, payload: Dict[str, Any]) -> List[Any]:
        owner = self._owner
        camera_ids = owner._normalize_camera_ids(payload.get("camera_ids"))
        if not camera_ids:
            camera_ids = list(owner.selected_camera_ids)
        targets = owner._discover_workflow_targets(camera_ids if camera_ids else None)
        if not targets:
            raise ValueError("no target cameras discovered for tracking")
        return targets

    def _prepare_pis_for_tracking(self, targets: List[Any]) -> Dict[str, Any]:
        owner = self._owner
        result = owner.session._broadcast(
            targets,
            "set_preview",
            render_enabled=False,
            overlays={"blob": False, "mask": False, "text": False, "charuco": False},
        )
        return {
            "ok": True,
            "camera_ids": [target.camera_id for target in targets],
            "preview_render_enabled": False,
            "overlays": {"blob": False, "mask": False, "text": False, "charuco": False},
            "result": result,
        }

    def _pause_receiver_for_tracking(self) -> None:
        owner = self._owner
        if owner._receiver_paused_for_tracking:
            return
        is_running = bool(getattr(owner.receiver, "is_running", False))
        stop = getattr(owner.receiver, "stop", None)
        if is_running and callable(stop):
            stop()
            owner._receiver_paused_for_tracking = True

    def _resume_receiver_after_tracking(self) -> None:
        owner = self._owner
        if not owner._receiver_paused_for_tracking:
            return
        start = getattr(owner.receiver, "start", None)
        if callable(start):
            start()
        owner._receiver_paused_for_tracking = False

    @staticmethod
    def all_acked_or_already_running(result: Dict[str, Any]) -> bool:
        for response in result.values():
            if not isinstance(response, dict):
                return False
            if bool(response.get("ack")):
                continue
            error_message = str(response.get("error_message") or response.get("error") or "")
            if "already_running" in error_message:
                continue
            return False
        return True

    def _tracking_start_blockers(self, cameras: List[Dict[str, Any]] | None = None) -> List[str]:
        owner = self._owner
        blockers: List[str] = []
        if not owner._tracking_extrinsics_ready():
            blockers.append("extrinsics_missing")
        active_cameras = self._tracking_active_cameras(cameras)
        if not active_cameras:
            blockers.append("no_cameras_selected")
            return blockers
        if any(not bool(camera.get("healthy")) for camera in active_cameras):
            blockers.append("camera_unhealthy")
        if any(not self._camera_mask_ready(camera) for camera in active_cameras):
            blockers.append("mask_missing")
        return blockers

    def _tracking_active_cameras(self, cameras: List[Dict[str, Any]] | None = None) -> List[Dict[str, Any]]:
        owner = self._owner
        if cameras is None:
            cameras = list(owner.camera_status.values())
        selected = [camera for camera in cameras if bool(camera.get("selected"))]
        return selected if selected else list(cameras)

    @staticmethod
    def _camera_mask_ready(camera: Dict[str, Any]) -> bool:
        diagnostics = camera.get("diagnostics")
        if not isinstance(diagnostics, dict):
            diagnostics = {}
        state = str(diagnostics.get("state") or "").upper()
        mask_pixels = float(diagnostics.get("mask_pixels", 0) or 0)
        return state in ("READY", "RUNNING") and mask_pixels > 0.0

    def _tracking_blocker_message(self, blockers: List[str]) -> str | None:
        if not blockers:
            return None
        return self._TRACKING_BLOCKER_MESSAGES.get(blockers[0], "Tracking is not ready")
