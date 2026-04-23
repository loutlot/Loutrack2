from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, List

from .gui_settings_store import GuiSettingsStore


class GuiTrackingRigidStore:
    """Persists custom rigid definitions separately from GUI settings."""

    def __init__(self, *, store_path: Path) -> None:
        self.store_path = store_path.resolve()

    def load(self) -> List[Dict[str, Any]]:
        try:
            payload = json.loads(self.store_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return []
        except Exception:
            return []
        if not isinstance(payload, dict):
            return []
        return GuiSettingsStore._normalize_custom_rigids(payload.get("custom_rigids", []))

    def save(self, custom_rigids: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        normalized = GuiSettingsStore._normalize_custom_rigids(custom_rigids)
        payload = {
            "meta": {
                "version": 1,
                "updated_at": int(time.time()),
            },
            "custom_rigids": normalized,
        }
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        self.store_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return normalized
