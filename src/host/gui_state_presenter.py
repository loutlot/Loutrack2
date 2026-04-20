from __future__ import annotations

from typing import Any, Dict

if __package__ in (None, ""):
    from gui_camera_status_store import GuiCameraStatusStore
    from gui_workflow_presenter import GuiWorkflowPresenter
else:
    from .gui_camera_status_store import GuiCameraStatusStore
    from .gui_workflow_presenter import GuiWorkflowPresenter


class GuiStatePresenter:
    """Top-level /api/state payload assembly for the host GUI backend."""

    def __init__(
        self,
        owner: Any,
        *,
        camera_status_store: GuiCameraStatusStore | None = None,
        workflow_presenter: GuiWorkflowPresenter | None = None,
    ) -> None:
        self._owner = owner
        self._camera_status_store = camera_status_store or GuiCameraStatusStore(owner)
        self._workflow_presenter = workflow_presenter or GuiWorkflowPresenter(owner)

    def get_state(self) -> Dict[str, Any]:
        owner = self._owner
        cameras = self._camera_status_store.refresh_targets()
        bundle = owner._load_settings_bundle()
        tracking_status = owner.get_tracking_status()
        return self.build(cameras=cameras, bundle=bundle, tracking_status=tracking_status)

    def build(
        self,
        *,
        cameras: list[Dict[str, Any]],
        bundle: Dict[str, Any],
        tracking_status: Dict[str, Any],
    ) -> Dict[str, Any]:
        owner = self._owner
        return {
            "config": owner._config_payload(),
            "cameras": cameras,
            "workflow": self._workflow_presenter.summarize(cameras),
            "extrinsics_methods": owner._generate_extrinsics_registry.to_payload(),
            "last_result": owner.last_result,
            "receiver": owner.receiver.stats,
            "tracking": tracking_status,
            "intrinsics_settings": owner._load_intrinsics_settings(),
            "settings_meta": {
                "validation": bundle.get("validation", {}),
                "runtime_hints": bundle.get("runtime_hints", {}),
            },
        }
