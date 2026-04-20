from __future__ import annotations

from contextlib import nullcontext
from typing import Any, Dict, List


class GuiCameraStatusStore:
    """Camera status aggregation for the host GUI backend."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def refresh_targets(self) -> List[Dict[str, Any]]:
        owner = self._owner
        discovered = owner._discover_workflow_targets()
        ping_results = owner.session._broadcast(discovered, "ping") if discovered else {}
        with self._lock():
            for target in discovered:
                ping_result = ping_results.get(target.camera_id, {})
                healthy = bool(ping_result.get("ack"))
                diagnostics = ping_result.get("result", {}) if isinstance(ping_result.get("result"), dict) else {}
                previous = owner.camera_status.get(target.camera_id, {})
                owner.camera_status[target.camera_id] = {
                    **previous,
                    "camera_id": target.camera_id,
                    "ip": target.ip,
                    "healthy": healthy,
                    "selected": not owner.selected_camera_ids or target.camera_id in owner.selected_camera_ids,
                    "diagnostics": diagnostics,
                    "last_ack": bool(ping_result.get("ack")) if ping_result else previous.get("last_ack"),
                    "last_error": ping_result.get("error") or ping_result.get("error_message"),
                }
            return sorted(owner.camera_status.values(), key=lambda item: item["camera_id"])

    def update_camera_status(self, result: Dict[str, Dict[str, Dict[str, Any]]]) -> None:
        owner = self._owner
        with self._lock():
            for responses in result.values():
                for camera_id, response in responses.items():
                    entry = owner.camera_status.setdefault(camera_id, {"camera_id": camera_id, "ip": "unknown"})
                    entry["last_ack"] = bool(response.get("ack"))
                    entry["last_error"] = response.get("error") or response.get("error_message")

    def apply_command_result(self, result: Dict[str, Dict[str, Dict[str, Any]]]) -> None:
        self.update_camera_status(result)

    def _lock(self) -> Any:
        owner_lock = getattr(self._owner, "lock", None)
        return owner_lock if owner_lock is not None else nullcontext()
