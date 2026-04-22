from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlCommandSpec:
    name: str
    schema_visible: bool = True
    supported: bool = True
    host_cli: bool = True


CONTROL_COMMAND_SPECS = (
    ControlCommandSpec("ping"),
    ControlCommandSpec("start"),
    ControlCommandSpec("stop"),
    ControlCommandSpec("set_exposure"),
    ControlCommandSpec("set_gain"),
    ControlCommandSpec("set_fps"),
    ControlCommandSpec("set_threshold"),
    ControlCommandSpec("set_blob_diameter"),
    ControlCommandSpec("mask_start"),
    ControlCommandSpec("mask_stop"),
    ControlCommandSpec("set_preview"),
    ControlCommandSpec("intrinsics_start"),
    ControlCommandSpec("intrinsics_stop"),
    ControlCommandSpec("intrinsics_clear"),
    ControlCommandSpec("intrinsics_get_corners"),
    ControlCommandSpec("intrinsics_status"),
    ControlCommandSpec("led_on", supported=False, host_cli=False),
    ControlCommandSpec("led_off", supported=False, host_cli=False),
    ControlCommandSpec("set_resolution", supported=False, host_cli=False),
)

CONTROL_COMMAND_SPECS_BY_NAME = {spec.name: spec for spec in CONTROL_COMMAND_SPECS}

SCHEMA_COMMANDS = tuple(spec.name for spec in CONTROL_COMMAND_SPECS if spec.schema_visible)
SCHEMA_COMMAND_SET = frozenset(SCHEMA_COMMANDS)

MVP_SUPPORTED_COMMANDS = tuple(spec.name for spec in CONTROL_COMMAND_SPECS if spec.supported)
MVP_SUPPORTED_COMMAND_SET = frozenset(MVP_SUPPORTED_COMMANDS)

HOST_CONTROL_CLI_COMMANDS = tuple(
    spec.name for spec in CONTROL_COMMAND_SPECS if spec.supported and spec.host_cli
)
HOST_CONTROL_CLI_COMMAND_SET = frozenset(HOST_CONTROL_CLI_COMMANDS)

UNSUPPORTED_SCHEMA_COMMANDS = tuple(
    spec.name for spec in CONTROL_COMMAND_SPECS if spec.schema_visible and not spec.supported
)
UNSUPPORTED_SCHEMA_COMMAND_SET = frozenset(UNSUPPORTED_SCHEMA_COMMANDS)
