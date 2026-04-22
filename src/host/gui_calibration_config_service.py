from __future__ import annotations

from typing import Any, Dict


class GuiCalibrationConfigService:
    """Applies calibration config payloads and broadcasts the committed settings to Pis."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def apply_config(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        owner = self._owner
        targets = owner._resolve_requested_targets(payload)
        bundle = owner._load_settings_bundle()
        calibration = bundle.get("calibration", {})
        committed = calibration.get("committed", {}) if isinstance(calibration, dict) else {}
        draft = dict(calibration.get("draft", {})) if isinstance(calibration, dict) else {}
        calibration_patch = {
            key: payload[key]
            for key in (
                "exposure_us",
                "gain",
                "threshold",
                "blob_min_diameter_px",
                "blob_max_diameter_px",
                "mask_threshold",
                "mask_seconds",
                "wand_metric_seconds",
            )
            if key in payload
        }
        draft.update(calibration_patch)
        next_committed, errors = owner._validate_calibration(
            draft,
            committed if isinstance(committed, dict) else {},
        )
        if errors:
            raise ValueError(f"calibration settings invalid: {errors}")
        owner.config = owner._build_session_config(next_committed)
        owner._persist_config()

        result = owner._apply_capture_settings(targets)
        owner._update_camera_status(result)
        owner.last_result = result
        return result
