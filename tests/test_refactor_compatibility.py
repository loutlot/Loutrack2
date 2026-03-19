from __future__ import annotations

import importlib
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import host.loutrack_gui as loutrack_gui


class _FakeReceiver:
    def __init__(self) -> None:
        self._frame_callback = None

    def set_frame_callback(self, callback) -> None:  # noqa: ANN001
        self._frame_callback = callback

    @property
    def stats(self) -> Dict[str, int]:
        return {"frames_received": 0, "cameras_discovered": 0}


class _FakeSession:
    def discover_targets(self, camera_ids=None):  # noqa: ANN001
        _ = camera_ids
        return []

    def _broadcast(self, targets, fn_name, **kwargs):  # noqa: ANN001
        _ = targets
        _ = fn_name
        _ = kwargs
        return {}


def test_settings_migration_from_old_to_new_and_delete_old(
    tmp_path: Path,
    monkeypatch,
) -> None:
    new_settings = tmp_path / "loutrack_gui_settings.json"
    old_settings = tmp_path / "wand_gui_settings.json"
    old_settings.write_text(
        json.dumps({"exposure_us": 31000, "gain": 3.0, "fps": 48}),
        encoding="utf-8",
    )
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_SETTINGS_PATH", new_settings)
    monkeypatch.setattr("host.loutrack_gui.OLD_SETTINGS_PATH", old_settings)

    state = loutrack_gui.LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
    )
    assert state.config.exposure_us == 31000
    assert state.config.fps == 48
    assert new_settings.exists()
    assert not old_settings.exists()


def test_settings_migration_prefers_new_and_deletes_old(
    tmp_path: Path,
    monkeypatch,
) -> None:
    new_settings = tmp_path / "loutrack_gui_settings.json"
    old_settings = tmp_path / "wand_gui_settings.json"
    new_settings.write_text(json.dumps({"exposure_us": 12000, "fps": 56}), encoding="utf-8")
    old_settings.write_text(json.dumps({"exposure_us": 31000, "fps": 48}), encoding="utf-8")
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_SETTINGS_PATH", new_settings)
    monkeypatch.setattr("host.loutrack_gui.OLD_SETTINGS_PATH", old_settings)

    state = loutrack_gui.LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
    )
    assert state.config.exposure_us == 12000
    assert state.config.fps == 56
    assert not old_settings.exists()


def test_settings_migration_invalid_old_creates_backup_and_defaults(
    tmp_path: Path,
    monkeypatch,
) -> None:
    new_settings = tmp_path / "loutrack_gui_settings.json"
    old_settings = tmp_path / "wand_gui_settings.json"
    old_settings.write_text("{invalid-json", encoding="utf-8")
    backup = tmp_path / "wand_gui_settings.json.invalid.bak"
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_SETTINGS_PATH", new_settings)
    monkeypatch.setattr("host.loutrack_gui.OLD_SETTINGS_PATH", old_settings)

    state = loutrack_gui.LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
    )
    assert state.config.exposure_us == 12000
    assert state.config.fps == 56
    assert new_settings.exists()
    assert backup.exists()
    assert not old_settings.exists()


def test_default_settings_path_resolves_from_project_root_not_cwd(
    tmp_path: Path,
    monkeypatch,
) -> None:
    repo_root = tmp_path / "repo"
    settings_rel = Path("logs") / "loutrack_gui_settings.json"
    settings_abs = repo_root / settings_rel
    settings_abs.parent.mkdir(parents=True, exist_ok=True)
    settings_abs.write_text(
        json.dumps(
            {
                "calibration": {
                    "draft": {"focus": 0.35},
                    "committed": {"focus": 0.35},
                },
                "intrinsics": {"draft": {}, "committed": {}},
                "extrinsics": {"draft": {}, "committed": {}, "locks": {}},
                "ui": {},
                "runtime_hints": {},
            }
        ),
        encoding="utf-8",
    )

    monkeypatch.setattr("host.loutrack_gui.PROJECT_ROOT", repo_root)
    monkeypatch.setattr("host.loutrack_gui.DEFAULT_SETTINGS_PATH", settings_rel)
    monkeypatch.chdir(tmp_path)

    state = loutrack_gui.LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
    )
    assert state.settings_path == settings_abs
    assert state.config.focus == 0.35


def test_partial_new_shape_settings_preserve_focus_and_square_length(tmp_path: Path) -> None:
    settings_path = tmp_path / "loutrack_gui_settings.json"
    settings_path.write_text(
        json.dumps(
            {
                "meta": {"version": 2},
                "calibration": {"draft": {"focus": 0.35}, "committed": {"focus": 0.35}},
                "intrinsics": {"draft": {"square_length_mm": 42.0}, "committed": {"square_length_mm": 42.0}},
                "extrinsics": {"draft": {}, "committed": {}},
            }
        ),
        encoding="utf-8",
    )

    state = loutrack_gui.LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=settings_path,
    )
    settings = state.get_settings()
    assert settings["calibration"]["draft"]["focus"] == 0.35
    assert settings["calibration"]["committed"]["focus"] == 0.35
    assert settings["intrinsics"]["draft"]["square_length_mm"] == 42.0
    assert settings["intrinsics"]["committed"]["square_length_mm"] == 42.0


def test_generate_extrinsics_supports_registry_methods(tmp_path: Path) -> None:
    state = loutrack_gui.LoutrackGuiState(
        session=_FakeSession(),
        receiver=_FakeReceiver(),
        settings_path=tmp_path / "settings.json",
    )
    pose_log = tmp_path / "pose.jsonl"
    pose_log.write_text("{}", encoding="utf-8")
    output_path = tmp_path / "extrinsics_pose_v2.json"

    def _solve_charuco(**kwargs: Any) -> Dict[str, Any]:
        payload = {
            "camera_order": ["pi-cam-01"],
            "pose": {"camera_poses": [], "solve_summary": {}},
            "metric": {"status": "skipped"},
            "world": {"status": "skipped"},
        }
        Path(str(kwargs["output_path"])).write_text(json.dumps(payload), encoding="utf-8")
        return payload

    state._generate_extrinsics_registry.register(
        name="charuco_pose_v1",
        solve=_solve_charuco,
        description="charuco pose solver placeholder",
    )
    result = state.generate_extrinsics(
        {
            "pose_log_path": str(pose_log),
            "output_path": str(output_path),
            "extrinsics_method": "charuco_pose_v1",
        }
    )
    assert result["generate_extrinsics"]["ok"] is True
    assert result["generate_extrinsics"]["extrinsics_method"] == "charuco_pose_v1"


def test_removed_legacy_modules_are_not_importable() -> None:
    for module_name in ("host.wand_gui", "pi.capture", "src.pi.capture", "wand_model", "src.wand_model"):
        try:
            importlib.import_module(module_name)
        except ModuleNotFoundError:
            continue
        raise AssertionError(f"legacy module should be removed: {module_name}")


def test_no_legacy_import_paths_remain_in_python_sources() -> None:
    root = Path(__file__).resolve().parents[1]
    py_files = list((root / "src").rglob("*.py")) + list((root / "tests").rglob("*.py"))
    banned_patterns = (
        re.compile(r"^\s*from\s+host\.wand_gui\b", re.MULTILINE),
        re.compile(r"^\s*import\s+host\.wand_gui\b", re.MULTILINE),
        re.compile(r"^\s*from\s+pi\.capture\b", re.MULTILINE),
        re.compile(r"^\s*import\s+pi\.capture\b", re.MULTILINE),
        re.compile(r"^\s*from\s+src\.pi\.capture\b", re.MULTILINE),
        re.compile(r"^\s*import\s+src\.pi\.capture\b", re.MULTILINE),
        re.compile(r"^\s*from\s+wand_model\b", re.MULTILINE),
        re.compile(r"^\s*import\s+wand_model\b", re.MULTILINE),
        re.compile(r"^\s*from\s+src\.wand_model\b", re.MULTILINE),
        re.compile(r"^\s*import\s+src\.wand_model\b", re.MULTILINE),
    )

    violations: list[str] = []
    for path in py_files:
        text = path.read_text(encoding="utf-8")
        for pattern in banned_patterns:
            if pattern.search(text):
                violations.append(f"{path.relative_to(root)}: {pattern.pattern}")
    assert not violations, "legacy import path remains:\n" + "\n".join(violations)
