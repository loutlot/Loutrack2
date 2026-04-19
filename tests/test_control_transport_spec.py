from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from host import control
from pi.service.capture_runtime import ControlServerConfig, ERROR_UNKNOWN_CMD
from pi.service.control_manifest import (
    HOST_CONTROL_CLI_COMMANDS,
    MVP_SUPPORTED_COMMANDS,
    SCHEMA_COMMANDS,
    UNSUPPORTED_SCHEMA_COMMANDS,
)
from pi.service.control_server import ControlServer


def _subparser_names(parser: argparse.ArgumentParser) -> tuple[str, ...]:
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            return tuple(action.choices)
    raise AssertionError("subparsers not found")


def test_control_schema_enum_matches_runtime_manifest() -> None:
    schema_path = ROOT / "schema" / "control.json"
    payload = json.loads(schema_path.read_text(encoding="utf-8"))
    command_enum = tuple(payload["definitions"]["command"]["enum"])
    assert set(command_enum) == set(SCHEMA_COMMANDS)


def test_host_cli_commands_match_public_manifest() -> None:
    parser = control.build_parser()
    assert control.public_cli_commands() == HOST_CONTROL_CLI_COMMANDS
    assert _subparser_names(parser) == HOST_CONTROL_CLI_COMMANDS


def test_schema_only_commands_are_rejected_by_runtime() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    try:
        for command_name in UNSUPPORTED_SCHEMA_COMMANDS:
            request = {
                "request_id": f"req-{command_name}",
                "camera_id": "pi-cam-01",
                "cmd": command_name,
                "params": {},
            }
            response = server._process_line(json.dumps(request).encode("utf-8"))
            assert response["ack"] is False
            assert response["error_code"] == ERROR_UNKNOWN_CMD
            assert response["error_message"] == f"unknown_cmd: {command_name}"
    finally:
        server.shutdown()


def test_ping_supported_commands_match_runtime_manifest() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    try:
        response = server._handle_ping("req-1", "pi-cam-01")
        assert response["ack"] is True
        result = response["result"]
        assert isinstance(result, dict)
        assert result["supported_commands"] == list(MVP_SUPPORTED_COMMANDS)
    finally:
        server.shutdown()
