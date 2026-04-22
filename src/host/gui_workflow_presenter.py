from __future__ import annotations

from contextlib import nullcontext
from typing import Any, Dict, List


class GuiWorkflowPresenter:
    """Workflow summary presentation for the host GUI backend."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def summarize(self, cameras: List[Dict[str, Any]]) -> Dict[str, Any]:
        owner = self._owner
        selected = [camera for camera in cameras if bool(camera.get("selected"))]
        active_cameras = selected if selected else cameras
        with self._lock():
            active_capture_kind = owner._active_capture_kind

        def diagnostics(camera: Dict[str, Any]) -> Dict[str, Any]:
            value = camera.get("diagnostics")
            return value if isinstance(value, dict) else {}

        def blob_diagnostics(camera: Dict[str, Any]) -> Dict[str, Any]:
            value = diagnostics(camera).get("blob_diagnostics")
            return value if isinstance(value, dict) else {}

        total_count = len(cameras)
        selected_count = len(selected)
        healthy_count = sum(1 for camera in active_cameras if bool(camera.get("healthy")))
        blob_ready_count = sum(
            1
            for camera in active_cameras
            if int(blob_diagnostics(camera).get("last_blob_count", 0) or 0) > 0
        )
        mask_ready_count = sum(
            1
            for camera in active_cameras
            if diagnostics(camera).get("state") in ("READY", "RUNNING")
            and float(diagnostics(camera).get("mask_pixels", 0) or 0) > 0.0
        )
        running_count = sum(
            1 for camera in active_cameras if diagnostics(camera).get("state") == "RUNNING"
        )
        preview_enabled_count = sum(
            1 for camera in active_cameras if bool(diagnostics(camera).get("debug_preview_enabled"))
        )
        preview_active_count = sum(
            1 for camera in active_cameras if bool(diagnostics(camera).get("debug_preview_active"))
        )
        pose_capture_exists = owner.pose_capture_log_path.exists() and owner.pose_capture_log_path.is_file()
        pose_capture_complete = pose_capture_exists or bool(owner._capture_completed.get("pose_capture"))
        wand_metric_exists = owner.wand_metric_log_path.exists() and owner.wand_metric_log_path.is_file()
        wand_metric_complete = wand_metric_exists or bool(owner._capture_completed.get("wand_metric_capture"))
        extrinsics_ready = (
            owner.latest_extrinsics_path is not None
            and owner.latest_extrinsics_path.exists()
            and owner.latest_extrinsics_path.is_file()
        )

        active_segment = "blob"
        if running_count > 0:
            active_segment = "wand"
        elif pose_capture_complete and not wand_metric_complete:
            active_segment = "floor"
        elif pose_capture_complete:
            active_segment = "extrinsics"
        elif selected_count > 0 and mask_ready_count >= selected_count:
            active_segment = "wand"
        elif blob_ready_count > 0:
            active_segment = "mask"

        return {
            "total_count": total_count,
            "selected_count": selected_count,
            "healthy_count": healthy_count,
            "blob_ready_count": blob_ready_count,
            "mask_ready_count": mask_ready_count,
            "running_count": running_count,
            "preview_enabled_count": preview_enabled_count,
            "preview_active_count": preview_active_count,
            "pose_capture_complete": pose_capture_complete,
            "pose_capture_exists": pose_capture_exists,
            "pose_capture_log_path": str(owner.pose_capture_log_path),
            "wand_metric_complete": wand_metric_complete,
            "wand_metric_exists": wand_metric_exists,
            "wand_metric_log_path": str(owner.wand_metric_log_path),
            "extrinsics_ready": extrinsics_ready,
            "latest_extrinsics_path": str(owner.latest_extrinsics_path) if owner.latest_extrinsics_path else None,
            "latest_extrinsics_quality": owner.latest_extrinsics_quality,
            "latest_extrinsics_result": (
                owner.latest_extrinsics_result
                if isinstance(getattr(owner, "latest_extrinsics_result", None), dict)
                else
                owner.last_result.get("generate_extrinsics")
                if isinstance(owner.last_result, dict)
                and isinstance(owner.last_result.get("generate_extrinsics"), dict)
                else None
            ),
            "active_segment": active_segment,
            "active_capture_kind": active_capture_kind,
        }

    def build(self, cameras: List[Dict[str, Any]]) -> Dict[str, Any]:
        return self.summarize(cameras)

    def _lock(self) -> Any:
        owner_lock = getattr(self._owner, "lock", None)
        return owner_lock if owner_lock is not None else nullcontext()
