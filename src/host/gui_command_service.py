from __future__ import annotations

from typing import Any, Dict


class GuiCommandService:
    """Dispatches `/api/command` requests while keeping the public contract stable."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def run_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        owner = self._owner
        command = str(payload.get("command", "")).strip()

        if command == "refresh":
            result = {"cameras": owner.refresh_targets()}
            owner.last_result = result
            return result

        calibration_commands = {
            "mask_start",
            "mask_stop",
            "set_preview",
            "start",
            "stop",
            "start_pose_capture",
            "stop_pose_capture",
            "start_wand_metric_capture",
            "stop_wand_metric_capture",
        }
        if command in calibration_commands:
            owner._assert_calibration_unlocked(command)
        targets = owner._resolve_requested_targets(payload)
        if command in {"mask_start", "start", "start_pose_capture", "start_wand_metric_capture"}:
            owner._ensure_calibration_settings_valid()
        command_handlers = {
            "ping": lambda: owner.session._broadcast(targets, "ping"),
            "mask_start": lambda: owner.session._broadcast(targets, "mask_start", **owner._mask_params()),
            "mask_stop": lambda: owner.session._broadcast(targets, "mask_stop"),
        }
        handler = command_handlers.get(command)
        if command == "set_preview":
            kwargs: Dict[str, Any] = {}
            render_enabled = payload.get("render_enabled")
            if render_enabled is not None:
                kwargs["render_enabled"] = bool(render_enabled)
            overlays = payload.get("overlays")
            if isinstance(overlays, dict):
                kwargs["overlays"] = overlays
            charuco = payload.get("charuco")
            if isinstance(charuco, dict):
                kwargs["charuco"] = charuco
            result = owner.session._broadcast(targets, "set_preview", **kwargs)
            owner._update_camera_status({command: result})
            owner.last_result = {command: result}
            return owner.last_result
        if command in ("start", "start_pose_capture", "start_wand_metric_capture"):
            return owner._capture_log_service.start_capture(command, payload, targets)
        if command in ("stop", "stop_pose_capture", "stop_wand_metric_capture"):
            return owner._capture_log_service.stop_capture(command, targets)
        if handler is None:
            raise ValueError(f"Unsupported command: {command}")
        result = handler()

        owner._update_camera_status({command: result})
        owner.last_result = {command: result}
        return owner.last_result
