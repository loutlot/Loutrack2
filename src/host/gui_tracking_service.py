from __future__ import annotations

import time
from typing import Any, Dict, List


class GuiTrackingService:
    """Tracking orchestration for the host GUI backend."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def get_tracking_status(self) -> Dict[str, Any]:
        owner = self._owner
        status = owner.tracking_runtime.status()
        start_allowed = owner._tracking_extrinsics_ready()
        return {
            **status,
            "start_allowed": start_allowed,
            "empty_state": None if start_allowed else "Generate extrinsics first",
            "latest_extrinsics_path": str(owner.latest_extrinsics_path) if owner.latest_extrinsics_path else None,
            "latest_extrinsics_quality": owner.latest_extrinsics_quality,
            "sse": owner.get_tracking_sse_diagnostics()
            if hasattr(owner, "get_tracking_sse_diagnostics")
            else {},
        }

    def get_tracking_scene(self) -> Dict[str, Any]:
        owner = self._owner
        start_allowed = owner._tracking_extrinsics_ready()
        if not start_allowed:
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
                "empty_state": "Generate extrinsics first",
            }
        return owner.tracking_runtime.scene_snapshot()

    def wait_tracking_scene(self, last_sequence: int | None, timeout: float = 15.0) -> Dict[str, Any]:
        owner = self._owner
        start_allowed = owner._tracking_extrinsics_ready()
        if not start_allowed:
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
        owner._ensure_calibration_settings_valid()
        targets = self._resolve_tracking_targets(payload)
        camera_ids = [target.camera_id for target in targets]
        config_result = owner._apply_capture_settings(targets)
        optimization = self._prepare_pis_for_tracking(targets)
        self._pause_receiver_for_tracking()
        try:
            status = owner.tracking_runtime.start(str(resolved), [str(item) for item in patterns])
            stream_start = owner.session._broadcast(targets, "start", mode="pose_capture")
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
