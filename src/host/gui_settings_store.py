from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List

from .wand_session import (
    FIXED_CIRCULARITY_MIN,
    FIXED_FOCUS,
    FIXED_FPS,
    FIXED_PAIR_WINDOW_US,
    MAX_EXPOSURE_US,
)


class GuiSettingsStore:
    """Owns GUI settings migration, normalization, validation, and persistence."""

    def __init__(
        self,
        *,
        project_root: Path,
        settings_path: Path,
        old_settings_path: Path,
        uses_default_settings_path: bool,
        default_pose_log_path: Path,
        default_wand_metric_log_path: Path,
        default_extrinsics_output_path: Path,
        default_wand_metric_duration_s: float,
    ) -> None:
        self.project_root = project_root
        self.settings_path = settings_path.resolve()
        self.old_settings_path = old_settings_path.resolve()
        self.uses_default_settings_path = uses_default_settings_path
        self.default_pose_log_path = default_pose_log_path
        self.default_wand_metric_log_path = default_wand_metric_log_path
        self.default_extrinsics_output_path = default_extrinsics_output_path
        self.default_wand_metric_duration_s = default_wand_metric_duration_s

    def migrate_once(self) -> None:
        if not self.uses_default_settings_path:
            return
        if not self.old_settings_path.exists() or self.old_settings_path == self.settings_path:
            return
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        if self.settings_path.exists():
            self.old_settings_path.unlink(missing_ok=True)
            return
        try:
            payload = json.loads(self.old_settings_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("legacy settings is not an object")
            self.settings_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self.old_settings_path.unlink(missing_ok=True)
            return
        except Exception:
            backup_path = self.old_settings_path.with_suffix(self.old_settings_path.suffix + ".invalid.bak")
            try:
                shutil.move(str(self.old_settings_path), str(backup_path))
            except Exception:
                self.old_settings_path.unlink(missing_ok=True)
            self.settings_path.write_text(
                json.dumps(self.default_settings_payload(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

    def ensure_settings_file_exists(self) -> None:
        if self.settings_path.exists():
            return
        self._write_settings_payload(self.default_settings_payload())

    def load_bundle(self) -> Dict[str, Any]:
        raw_payload = self._read_settings_payload()
        bundle = self._normalize_settings_payload(raw_payload)
        self._coerce_fixed_calibration_bundle(bundle)
        validation = self._refresh_committed_from_draft(bundle)
        bundle["validation"] = validation
        return bundle

    def save_bundle(self, bundle: Dict[str, Any]) -> None:
        self._coerce_fixed_calibration_bundle(bundle)
        payload = {
            "meta": bundle.get("meta", {}),
            "calibration": bundle.get("calibration", {}),
            "intrinsics": bundle.get("intrinsics", {}),
            "extrinsics": bundle.get("extrinsics", {}),
            "ui": bundle.get("ui", {}),
            "runtime_hints": bundle.get("runtime_hints", {}),
        }
        self._write_settings_payload(payload)

    def get_settings(self) -> Dict[str, Any]:
        bundle = self.load_bundle()
        return {
            "calibration": bundle.get("calibration", {}),
            "intrinsics": bundle.get("intrinsics", {}),
            "extrinsics": bundle.get("extrinsics", {}),
            "ui": bundle.get("ui", {}),
            "runtime_hints": bundle.get("runtime_hints", {}),
            "validation": bundle.get("validation", {}),
        }

    def apply_draft_patch(self, bundle: Dict[str, Any], patch: Dict[str, Any]) -> Dict[str, Any]:
        calibration_patch = patch.get("calibration", {})
        if isinstance(calibration_patch, dict):
            bundle["calibration"]["draft"].update(calibration_patch)
            self._coerce_fixed_calibration_payload(bundle["calibration"]["draft"])

        intrinsics_patch = patch.get("intrinsics", {})
        if isinstance(intrinsics_patch, dict):
            bundle["intrinsics"]["draft"].update(intrinsics_patch)

        extrinsics_patch = patch.get("extrinsics", {})
        if isinstance(extrinsics_patch, dict):
            bundle["extrinsics"]["draft"].update(extrinsics_patch)
            locks = bundle["extrinsics"]["locks"]
            if "pose_log_path" in extrinsics_patch:
                locks["pose_log_path_manual"] = True
            if "wand_metric_log_path" in extrinsics_patch:
                locks["wand_metric_log_path_manual"] = True

        reset_runtime = patch.get("reset_runtime")
        if isinstance(reset_runtime, list):
            hints = bundle.get("runtime_hints", {})
            for key in reset_runtime:
                if key == "pose_log_path":
                    bundle["extrinsics"]["locks"]["pose_log_path_manual"] = False
                    latest_pose = str(hints.get("pose_log_path", self.default_pose_log_path))
                    bundle["extrinsics"]["draft"]["pose_log_path"] = latest_pose
                if key == "wand_metric_log_path":
                    bundle["extrinsics"]["locks"]["wand_metric_log_path_manual"] = False
                    latest_wand = str(hints.get("wand_metric_log_path", self.default_wand_metric_log_path))
                    bundle["extrinsics"]["draft"]["wand_metric_log_path"] = latest_wand

        validation = self._refresh_committed_from_draft(bundle)
        bundle["validation"] = validation
        self.save_bundle(bundle)
        return bundle

    def update_runtime_hints(
        self,
        *,
        pose_log_path: str | None = None,
        wand_metric_log_path: str | None = None,
    ) -> None:
        bundle = self.load_bundle()
        hints = bundle["runtime_hints"]
        if pose_log_path is not None:
            hints["pose_log_path"] = str(pose_log_path)
        if wand_metric_log_path is not None:
            hints["wand_metric_log_path"] = str(wand_metric_log_path)
        validation = self._refresh_committed_from_draft(bundle)
        bundle["validation"] = validation
        self.save_bundle(bundle)

    def apply_ui(self, payload: Dict[str, Any], selected_camera_ids: List[str]) -> Dict[str, Any]:
        bundle = self.load_bundle()
        ui_payload = bundle.get("ui", {})
        if "active_page" in payload:
            ui_payload["active_page"] = str(payload.get("active_page") or "calibration")
        if "active_view" in payload:
            ui_payload["active_view"] = str(payload.get("active_view") or "cameras")
        if "selected_camera_ids" in payload:
            selected_ids = payload.get("selected_camera_ids")
            if isinstance(selected_ids, list):
                cleaned = list(dict.fromkeys(str(item).strip() for item in selected_ids if str(item).strip()))
                ui_payload["selected_camera_ids"] = cleaned
                selected_camera_ids[:] = cleaned
        bundle["ui"] = ui_payload
        self.save_bundle(bundle)
        return {"ok": True, "ui": bundle.get("ui", {})}

    def ensure_calibration_settings_valid(self) -> Dict[str, Any]:
        bundle = self.load_bundle()
        validation = bundle.get("validation", {}).get("calibration", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"calibration settings invalid: {validation}")
        calibration = bundle.get("calibration", {})
        committed = calibration.get("committed", {}) if isinstance(calibration, dict) else {}
        return committed if isinstance(committed, dict) else {}

    def persist_calibration_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        bundle = self.load_bundle()
        bundle["calibration"]["draft"].update(payload)
        bundle["calibration"]["committed"].update(payload)
        self._coerce_fixed_calibration_bundle(bundle)
        bundle["validation"] = self._refresh_committed_from_draft(bundle)
        self.save_bundle(bundle)
        return bundle

    def persist_intrinsics_payload(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        bundle = self.load_bundle()
        bundle["intrinsics"]["draft"].update(payload)
        bundle["intrinsics"]["committed"].update(payload)
        bundle["validation"] = self._refresh_committed_from_draft(bundle)
        self.save_bundle(bundle)
        return bundle

    def load_intrinsics_settings(self) -> Dict[str, Any]:
        defaults: Dict[str, Any] = self.default_intrinsics_payload()
        bundle = self.load_bundle()
        intrinsics = bundle.get("intrinsics", {})
        draft = intrinsics.get("draft", {}) if isinstance(intrinsics, dict) else {}
        if isinstance(draft, dict):
            for key in defaults:
                if key in draft:
                    defaults[key] = draft[key]
        return defaults

    def _read_settings_payload(self) -> Dict[str, Any]:
        try:
            payload = json.loads(self.settings_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        except Exception:
            return {}
        if isinstance(payload, dict):
            return payload
        return {}

    def _write_settings_payload(self, payload: Dict[str, Any]) -> None:
        self._coerce_fixed_calibration_bundle(payload)
        payload.setdefault("meta", {})
        if isinstance(payload["meta"], dict):
            payload["meta"]["version"] = 2
            payload["meta"]["updated_at"] = int(time.time())
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        self.settings_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _normalize_settings_payload(self, raw_payload: Dict[str, Any]) -> Dict[str, Any]:
        defaults = self.default_settings_payload()
        if not isinstance(raw_payload, dict):
            return defaults

        has_sectioned_shape = any(
            isinstance(raw_payload.get(key), dict) for key in ("calibration", "intrinsics", "extrinsics")
        )
        if has_sectioned_shape:
            normalized = defaults

            calibration_src = raw_payload.get("calibration")
            if isinstance(calibration_src, dict):
                for key in normalized["calibration"]["draft"]:
                    if key in calibration_src.get("draft", {}):
                        normalized["calibration"]["draft"][key] = calibration_src["draft"][key]
                    elif key in calibration_src:
                        normalized["calibration"]["draft"][key] = calibration_src[key]
                    if key in calibration_src.get("committed", {}):
                        normalized["calibration"]["committed"][key] = calibration_src["committed"][key]
                    elif key in calibration_src:
                        normalized["calibration"]["committed"][key] = calibration_src[key]
            else:
                for key in normalized["calibration"]["draft"]:
                    if key in raw_payload:
                        normalized["calibration"]["draft"][key] = raw_payload[key]
                        normalized["calibration"]["committed"][key] = raw_payload[key]

            intrinsics_src = raw_payload.get("intrinsics")
            if isinstance(intrinsics_src, dict):
                for key in normalized["intrinsics"]["draft"]:
                    if key in intrinsics_src.get("draft", {}):
                        normalized["intrinsics"]["draft"][key] = intrinsics_src["draft"][key]
                    elif key in intrinsics_src:
                        normalized["intrinsics"]["draft"][key] = intrinsics_src[key]
                    if key in intrinsics_src.get("committed", {}):
                        normalized["intrinsics"]["committed"][key] = intrinsics_src["committed"][key]
                    elif key in intrinsics_src:
                        normalized["intrinsics"]["committed"][key] = intrinsics_src[key]

            extrinsics_src = raw_payload.get("extrinsics")
            if isinstance(extrinsics_src, dict):
                for key in normalized["extrinsics"]["draft"]:
                    if key in extrinsics_src.get("draft", {}):
                        normalized["extrinsics"]["draft"][key] = extrinsics_src["draft"][key]
                    elif key in extrinsics_src:
                        normalized["extrinsics"]["draft"][key] = extrinsics_src[key]
                    if key in extrinsics_src.get("committed", {}):
                        normalized["extrinsics"]["committed"][key] = extrinsics_src["committed"][key]
                    elif key in extrinsics_src:
                        normalized["extrinsics"]["committed"][key] = extrinsics_src[key]
                locks = extrinsics_src.get("locks", {})
                if isinstance(locks, dict):
                    normalized["extrinsics"]["locks"]["pose_log_path_manual"] = bool(
                        locks.get("pose_log_path_manual", False)
                    )
                    normalized["extrinsics"]["locks"]["wand_metric_log_path_manual"] = bool(
                        locks.get("wand_metric_log_path_manual", False)
                    )

            ui_src = raw_payload.get("ui")
            if isinstance(ui_src, dict):
                normalized["ui"]["active_page"] = str(
                    ui_src.get("active_page", normalized["ui"]["active_page"])
                )
                normalized["ui"]["active_view"] = str(
                    ui_src.get("active_view", normalized["ui"]["active_view"])
                )
                selected_ids = ui_src.get("selected_camera_ids", [])
                if isinstance(selected_ids, list):
                    normalized["ui"]["selected_camera_ids"] = [
                        str(item).strip() for item in selected_ids if str(item).strip()
                    ]

            hints_src = raw_payload.get("runtime_hints")
            if isinstance(hints_src, dict):
                normalized["runtime_hints"]["pose_log_path"] = str(
                    hints_src.get("pose_log_path", normalized["runtime_hints"]["pose_log_path"])
                )
                normalized["runtime_hints"]["wand_metric_log_path"] = str(
                    hints_src.get("wand_metric_log_path", normalized["runtime_hints"]["wand_metric_log_path"])
                )

            meta_src = raw_payload.get("meta")
            if isinstance(meta_src, dict):
                normalized["meta"]["updated_at"] = int(meta_src.get("updated_at", normalized["meta"]["updated_at"]))
            self._coerce_fixed_calibration_bundle(normalized)
            return normalized

        normalized = defaults
        calibration_keys = set(normalized["calibration"]["draft"].keys())
        for key in calibration_keys:
            if key in raw_payload:
                normalized["calibration"]["draft"][key] = raw_payload[key]
                normalized["calibration"]["committed"][key] = raw_payload[key]

        intrinsics_src = raw_payload.get("intrinsics", {})
        if isinstance(intrinsics_src, dict):
            for key in normalized["intrinsics"]["draft"]:
                if key in intrinsics_src:
                    normalized["intrinsics"]["draft"][key] = intrinsics_src[key]
                    normalized["intrinsics"]["committed"][key] = intrinsics_src[key]

        if "selected_camera_ids" in raw_payload and isinstance(raw_payload["selected_camera_ids"], list):
            normalized["ui"]["selected_camera_ids"] = [
                str(item).strip() for item in raw_payload["selected_camera_ids"] if str(item).strip()
            ]
        if "active_page" in raw_payload:
            normalized["ui"]["active_page"] = str(raw_payload["active_page"])
        if "active_view" in raw_payload:
            normalized["ui"]["active_view"] = str(raw_payload["active_view"])
        self._coerce_fixed_calibration_bundle(normalized)
        return normalized

    @staticmethod
    def _coerce_fixed_calibration_payload(payload: Dict[str, Any]) -> None:
        payload["focus"] = FIXED_FOCUS
        payload["circularity_min"] = FIXED_CIRCULARITY_MIN
        payload.pop("fps", None)

    @classmethod
    def _coerce_fixed_calibration_bundle(cls, bundle: Dict[str, Any]) -> None:
        calibration = bundle.get("calibration", {})
        if not isinstance(calibration, dict):
            return
        for key in ("draft", "committed"):
            payload = calibration.get(key)
            if isinstance(payload, dict):
                cls._coerce_fixed_calibration_payload(payload)
        extrinsics = bundle.get("extrinsics", {})
        if not isinstance(extrinsics, dict):
            return
        for key in ("draft", "committed"):
            payload = extrinsics.get(key)
            if not isinstance(payload, dict):
                continue
            for pair_key in ("pair_window_us", "wand_pair_window_us"):
                if pair_key not in payload:
                    payload[pair_key] = FIXED_PAIR_WINDOW_US
                    continue
                try:
                    payload[pair_key] = max(1, min(cls._to_int(payload[pair_key]), FIXED_PAIR_WINDOW_US))
                except Exception:
                    payload[pair_key] = FIXED_PAIR_WINDOW_US

    def _refresh_committed_from_draft(self, bundle: Dict[str, Any]) -> Dict[str, Dict[str, str]]:
        calibration = bundle.get("calibration", {})
        intrinsics = bundle.get("intrinsics", {})
        extrinsics = bundle.get("extrinsics", {})

        cal_committed, cal_errors = self.validate_calibration(
            calibration.get("draft", {}),
            calibration.get("committed", {}),
        )
        int_committed, int_errors = self.validate_intrinsics(
            intrinsics.get("draft", {}),
            intrinsics.get("committed", {}),
        )
        ext_committed, ext_errors = self.validate_extrinsics(
            extrinsics.get("draft", {}),
            extrinsics.get("committed", {}),
        )
        calibration["committed"] = cal_committed
        intrinsics["committed"] = int_committed
        extrinsics["committed"] = ext_committed
        return {
            "calibration": cal_errors,
            "intrinsics": int_errors,
            "extrinsics": ext_errors,
        }

    @staticmethod
    def _to_int(value: Any) -> int:
        if isinstance(value, bool):
            raise ValueError("boolean is not an integer")
        if isinstance(value, str):
            text = value.strip()
            if text == "":
                raise ValueError("empty string")
            return int(float(text)) if "." in text else int(text)
        return int(value)

    @staticmethod
    def _to_float(value: Any) -> float:
        if isinstance(value, bool):
            raise ValueError("boolean is not a number")
        if isinstance(value, str):
            text = value.strip()
            if text == "":
                raise ValueError("empty string")
            return float(text)
        return float(value)

    @staticmethod
    def _to_nullable_float(value: Any) -> float | None:
        if value is None:
            return None
        if isinstance(value, str) and value.strip() == "":
            return None
        return GuiSettingsStore._to_float(value)

    @staticmethod
    def _to_string(value: Any) -> str:
        if value is None:
            return ""
        return str(value).strip()

    @classmethod
    def validate_calibration(
        cls,
        draft: Dict[str, Any],
        committed: Dict[str, Any],
    ) -> tuple[Dict[str, Any], Dict[str, str]]:
        next_committed = dict(committed)
        errors: Dict[str, str] = {}
        next_committed["focus"] = FIXED_FOCUS
        next_committed["circularity_min"] = FIXED_CIRCULARITY_MIN
        numeric_specs = {
            "exposure_us": (
                "int",
                lambda v: 0 < v <= MAX_EXPOSURE_US,
                f"must be integer in [1,{MAX_EXPOSURE_US}]",
            ),
            "gain": ("float", lambda v: v > 0.0, "must be > 0"),
            "threshold": ("int", lambda v: 0 <= v <= 255, "must be in [0,255]"),
            "mask_threshold": ("int", lambda v: 0 <= v <= 255, "must be in [0,255]"),
            "mask_seconds": ("float", lambda v: v > 0.0, "must be > 0"),
            "wand_metric_seconds": ("float", lambda v: v > 0.0, "must be > 0"),
        }
        for key, (kind, predicate, message) in numeric_specs.items():
            if key not in draft:
                continue
            raw_value = draft.get(key)
            try:
                parsed = cls._to_int(raw_value) if kind == "int" else cls._to_float(raw_value)
                if not predicate(parsed):
                    raise ValueError(message)
                next_committed[key] = parsed
            except Exception:
                errors[key] = message

        for key in ("blob_min_diameter_px", "blob_max_diameter_px"):
            if key not in draft:
                continue
            raw_value = draft.get(key)
            try:
                parsed_nullable = cls._to_nullable_float(raw_value)
                if parsed_nullable is not None and parsed_nullable <= 0.0:
                    raise ValueError("must be > 0 when set")
                next_committed[key] = parsed_nullable
            except Exception:
                errors[key] = "must be empty or > 0"

        min_blob = next_committed.get("blob_min_diameter_px")
        max_blob = next_committed.get("blob_max_diameter_px")
        if isinstance(min_blob, (int, float)) and isinstance(max_blob, (int, float)) and min_blob > max_blob:
            errors["blob_max_diameter_px"] = "must be >= blob_min_diameter_px"
        return next_committed, errors

    @classmethod
    def validate_intrinsics(
        cls,
        draft: Dict[str, Any],
        committed: Dict[str, Any],
    ) -> tuple[Dict[str, Any], Dict[str, str]]:
        next_committed = dict(committed)
        errors: Dict[str, str] = {}

        if "camera_id" in draft:
            value = cls._to_string(draft.get("camera_id"))
            if value:
                next_committed["camera_id"] = value
            else:
                errors["camera_id"] = "must not be empty"
        if "mjpeg_url" in draft:
            next_committed["mjpeg_url"] = cls._to_string(draft.get("mjpeg_url"))

        int_specs = {
            "squares_x": (2, "must be integer >= 2"),
            "squares_y": (2, "must be integer >= 2"),
            "min_frames": (5, "must be integer >= 5"),
        }
        for key, (lower_bound, message) in int_specs.items():
            if key not in draft:
                continue
            try:
                value = cls._to_int(draft.get(key))
                if value < lower_bound:
                    raise ValueError(message)
                next_committed[key] = value
            except Exception:
                errors[key] = message

        float_specs = {
            "square_length_mm": (lambda v: v > 0.0, "must be > 0"),
            "cooldown_s": (lambda v: v > 0.0, "must be > 0"),
        }
        for key, (predicate, message) in float_specs.items():
            if key not in draft:
                continue
            try:
                value = cls._to_float(draft.get(key))
                if not predicate(value):
                    raise ValueError(message)
                next_committed[key] = value
            except Exception:
                errors[key] = message

        if "marker_length_mm" in draft:
            try:
                marker_value = cls._to_nullable_float(draft.get("marker_length_mm"))
                if marker_value is not None and marker_value <= 0.0:
                    raise ValueError("must be > 0 when set")
                next_committed["marker_length_mm"] = marker_value
            except Exception:
                errors["marker_length_mm"] = "must be empty or > 0"

        square_length = next_committed.get("square_length_mm")
        marker_length = next_committed.get("marker_length_mm")
        if (
            isinstance(square_length, (int, float))
            and isinstance(marker_length, (int, float))
            and marker_length >= square_length
        ):
            errors["marker_length_mm"] = "must be smaller than square_length_mm"
        return next_committed, errors

    @classmethod
    def validate_extrinsics(
        cls,
        draft: Dict[str, Any],
        committed: Dict[str, Any],
    ) -> tuple[Dict[str, Any], Dict[str, str]]:
        next_committed = dict(committed)
        errors: Dict[str, str] = {}
        text_keys = ("intrinsics_path", "pose_log_path", "output_path")
        for key in text_keys:
            if key not in draft:
                continue
            value = cls._to_string(draft.get(key))
            if value:
                next_committed[key] = value
            else:
                errors[key] = "must not be empty"

        if "wand_metric_log_path" in draft:
            next_committed["wand_metric_log_path"] = cls._to_string(draft.get("wand_metric_log_path"))

        int_specs = {
            "pair_window_us": (
                f"must be integer in [1,{FIXED_PAIR_WINDOW_US}]",
                1,
            ),
            "wand_pair_window_us": (
                f"must be integer in [1,{FIXED_PAIR_WINDOW_US}]",
                1,
            ),
            "min_pairs": ("must be integer >= 1", 1),
        }
        for key, (message, lower_bound) in int_specs.items():
            if key not in draft:
                continue
            try:
                value = cls._to_int(draft.get(key))
                if value < lower_bound:
                    raise ValueError(message)
                if key in {"pair_window_us", "wand_pair_window_us"} and value > FIXED_PAIR_WINDOW_US:
                    raise ValueError(message)
                next_committed[key] = value
            except Exception:
                errors[key] = message
        if "wand_face" in draft:
            value = cls._to_string(draft.get("wand_face")).strip().lower()
            if value in ("front_up", "back_up"):
                next_committed["wand_face"] = value
            else:
                errors["wand_face"] = "must be front_up or back_up"
        return next_committed, errors

    def default_settings_payload(self) -> Dict[str, Any]:
        calibration_defaults = self.default_calibration_payload()
        intrinsics_defaults = self.default_intrinsics_payload()
        extrinsics_defaults = self.default_extrinsics_payload()
        return {
            "meta": {"version": 2, "updated_at": int(time.time())},
            "calibration": {
                "draft": dict(calibration_defaults),
                "committed": dict(calibration_defaults),
            },
            "intrinsics": {
                "draft": dict(intrinsics_defaults),
                "committed": dict(intrinsics_defaults),
            },
            "extrinsics": {
                "draft": dict(extrinsics_defaults),
                "committed": dict(extrinsics_defaults),
                "locks": {
                    "pose_log_path_manual": False,
                    "wand_metric_log_path_manual": False,
                },
            },
            "ui": {
                "active_page": "calibration",
                "active_view": "cameras",
                "selected_camera_ids": [],
            },
            "runtime_hints": {
                "pose_log_path": str(self.default_pose_log_path),
                "wand_metric_log_path": str(self.default_wand_metric_log_path),
            },
        }

    def default_calibration_payload(self) -> Dict[str, Any]:
        return {
            "exposure_us": 5000,
            "gain": 8.0,
            "focus": FIXED_FOCUS,
            "threshold": 200,
            "blob_min_diameter_px": None,
            "blob_max_diameter_px": None,
            "circularity_min": FIXED_CIRCULARITY_MIN,
            "mask_threshold": 200,
            "mask_seconds": 0.5,
            "wand_metric_seconds": self.default_wand_metric_duration_s,
        }

    @staticmethod
    def default_intrinsics_payload() -> Dict[str, Any]:
        return {
            "camera_id": "pi-cam-01",
            "mjpeg_url": "",
            "square_length_mm": 60.0,
            "marker_length_mm": None,
            "squares_x": 6,
            "squares_y": 8,
            "min_frames": 25,
            "cooldown_s": 1.5,
        }

    def default_extrinsics_payload(self) -> Dict[str, Any]:
        return {
            "intrinsics_path": "calibration",
            "pose_log_path": str(self.default_pose_log_path),
            "wand_metric_log_path": str(self.default_wand_metric_log_path),
            "output_path": str(self.default_extrinsics_output_path),
            "pair_window_us": FIXED_PAIR_WINDOW_US,
            "wand_pair_window_us": FIXED_PAIR_WINDOW_US,
            "min_pairs": 8,
            "wand_face": "front_up",
        }
