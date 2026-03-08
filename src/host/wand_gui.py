#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import threading
import importlib.util
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

MODULE_SRC_ROOT = Path(__file__).resolve().parents[1]

if __package__ in (None, ""):
    if str(MODULE_SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(MODULE_SRC_ROOT))
    from host.receiver import UDPReceiver
    from host.wand_session import SessionConfig, WandSession
else:
    from .receiver import UDPReceiver
    from .wand_session import SessionConfig, WandSession


HTML_PAGE = """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Loutrack2 Wand Control</title>
  <style>
    :root {
      --bg: #f2efe8;
      --panel: #fffaf2;
      --ink: #1d2a33;
      --accent: #b24a2c;
      --accent-2: #2f6c73;
      --line: #d8cdbf;
    }
    body {
      margin: 0;
      font-family: "Avenir Next", "Helvetica Neue", sans-serif;
      color: var(--ink);
      background: radial-gradient(circle at top left, #fff8e8, var(--bg) 58%);
    }
    main {
      max-width: 1100px;
      margin: 0 auto;
      padding: 24px;
    }
    h1 { margin: 0 0 12px; font-size: 32px; }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 16px;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 30px rgba(29, 42, 51, 0.08);
    }
    label { display: block; margin: 12px 0 6px; font-weight: 600; }
    input[type="range"] { width: 100%; }
    button {
      border: 0;
      border-radius: 999px;
      padding: 10px 14px;
      margin: 6px 8px 0 0;
      background: var(--accent);
      color: white;
      cursor: pointer;
    }
    button.secondary { background: var(--accent-2); }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
      font-size: 14px;
    }
    th, td {
      text-align: left;
      padding: 10px 8px;
      border-bottom: 1px solid var(--line);
    }
    pre {
      background: #1d2a33;
      color: #f7f3ea;
      padding: 12px;
      border-radius: 12px;
      overflow: auto;
      min-height: 120px;
    }
  </style>
</head>
<body>
  <main>
    <h1>Wand Control Console</h1>
    <div class="grid">
      <section class="card">
        <h2>Sync Sliders</h2>
        <label for="exposure">Exposure (<span id="exposureValue"></span> us)</label>
        <input id="exposure" type="range" min="100" max="30000" step="100" value="1200">
        <label for="gain">Gain (<span id="gainValue"></span>)</label>
        <input id="gain" type="range" min="1" max="16" step="0.1" value="4">
        <label for="fps">FPS (<span id="fpsValue"></span>)</label>
        <input id="fps" type="range" min="15" max="120" step="1" value="80">
        <label for="focus">Focus (<span id="focusValue"></span>)</label>
        <input id="focus" type="range" min="0.0" max="10.0" step="0.001" value="5.215">
        <label for="threshold">Threshold (<span id="thresholdValue"></span>)</label>
        <input id="threshold" type="range" min="0" max="255" step="1" value="200">
        <label for="circularity">Circularity Min (<span id="circularityValue"></span>)</label>
        <input id="circularity" type="range" min="0.0" max="1.0" step="0.01" value="0.0">
        <label for="blobMin">Blob Min Diameter px (<span id="blobMinValue"></span>)</label>
        <input id="blobMin" type="range" min="0" max="100" step="0.5" value="0">
        <label for="blobMax">Blob Max Diameter px (<span id="blobMaxValue"></span>)</label>
        <input id="blobMax" type="range" min="0" max="100" step="0.5" value="0">
        <button id="applyConfig">Apply To Selected</button>
      </section>
      <section class="card">
        <h2>Commands</h2>
        <label for="maskThreshold">Mask Threshold (<span id="maskThresholdValue"></span>)</label>
        <input id="maskThreshold" type="range" min="0" max="255" step="1" value="200">
        <label for="maskSeconds">Mask Seconds (<span id="maskSecondsValue"></span>)</label>
        <input id="maskSeconds" type="range" min="0.1" max="5.0" step="0.1" value="0.5">
        <button class="secondary" data-command="refresh">Refresh</button>
        <button class="secondary" data-command="ping">Ping</button>
        <button class="secondary" data-command="mask_start">Mask Start</button>
        <button class="secondary" data-command="start">Start</button>
        <button class="secondary" data-command="stop">Stop</button>
      </section>
      <section class="card">
        <h2>Extrinsics</h2>
        <label for="intrinsicsPath">Intrinsics Dir</label>
        <input id="intrinsicsPath" type="text" value="calibration" style="width:100%;">
        <label for="logPath">Log Path</label>
        <input id="logPath" type="text" value="logs/wand_capture.jsonl" style="width:100%;">
        <label for="outputPath">Output Path</label>
        <input id="outputPath" type="text" value="calibration/calibration_extrinsics_v1.json" style="width:100%;">
        <button id="generateExtrinsics">Generate Extrinsics</button>
      </section>
      <section class="card">
        <h2>Status</h2>
        <pre id="status"></pre>
      </section>
    </div>
    <section class="card" style="margin-top: 16px;">
      <h2>Cameras</h2>
      <table>
        <thead>
          <tr>
            <th>Select</th>
            <th>Camera</th>
            <th>IP</th>
            <th>State</th>
            <th>Blobs</th>
            <th>Reject</th>
            <th>Healthy</th>
            <th>Last Ack</th>
            <th>Last Error</th>
          </tr>
        </thead>
        <tbody id="cameraRows"></tbody>
      </table>
    </section>
  </main>
  <script>
    const sliders = ["exposure", "gain", "fps", "focus", "threshold", "circularity", "blobMin", "blobMax", "maskThreshold", "maskSeconds"];
    const sliderNames = new Set(sliders);
    const values = {
      exposure: document.getElementById("exposureValue"),
      gain: document.getElementById("gainValue"),
      fps: document.getElementById("fpsValue"),
      focus: document.getElementById("focusValue"),
      threshold: document.getElementById("thresholdValue"),
      circularity: document.getElementById("circularityValue"),
      blobMin: document.getElementById("blobMinValue"),
      blobMax: document.getElementById("blobMaxValue"),
      maskThreshold: document.getElementById("maskThresholdValue"),
      maskSeconds: document.getElementById("maskSecondsValue"),
    };
    const elements = {
      exposure: document.getElementById("exposure"),
      gain: document.getElementById("gain"),
      fps: document.getElementById("fps"),
      focus: document.getElementById("focus"),
      threshold: document.getElementById("threshold"),
      circularity: document.getElementById("circularity"),
      blobMin: document.getElementById("blobMin"),
      blobMax: document.getElementById("blobMax"),
      maskThreshold: document.getElementById("maskThreshold"),
      maskSeconds: document.getElementById("maskSeconds"),
      rows: document.getElementById("cameraRows"),
      status: document.getElementById("status"),
      intrinsicsPath: document.getElementById("intrinsicsPath"),
      logPath: document.getElementById("logPath"),
      outputPath: document.getElementById("outputPath"),
    };
    const sliderUpdateTimers = {};
    const SLIDER_DEBOUNCE_MS = 200;

    function selectedCameraIds() {
      return [...document.querySelectorAll("input[data-camera]:checked")].map((item) => item.dataset.camera);
    }

    function configPayload() {
      return {
        camera_ids: selectedCameraIds(),
        exposure_us: Number(elements.exposure.value),
        gain: Number(elements.gain.value),
        fps: Number(elements.fps.value),
        focus: Number(elements.focus.value),
        threshold: Number(elements.threshold.value),
        circularity_min: Number(elements.circularity.value),
        blob_min_diameter_px: Number(elements.blobMin.value) > 0 ? Number(elements.blobMin.value) : null,
        blob_max_diameter_px: Number(elements.blobMax.value) > 0 ? Number(elements.blobMax.value) : null,
        mask_threshold: Number(elements.maskThreshold.value),
        mask_seconds: Number(elements.maskSeconds.value),
      };
    }

    async function loadState() {
      const response = await fetch("/api/state");
      const state = await response.json();
      const configMap = {
        exposure: state.config.exposure_us,
        gain: state.config.gain,
        fps: state.config.fps,
        focus: state.config.focus,
        threshold: state.config.threshold,
        circularity: state.config.circularity_min,
        blobMin: state.config.blob_min_diameter_px ?? 0,
        blobMax: state.config.blob_max_diameter_px ?? 0,
        maskThreshold: state.config.mask_threshold,
        maskSeconds: state.config.mask_seconds,
      };
      sliders.forEach((name) => {
        if (document.activeElement === elements[name] || sliderUpdateTimers[name]) {
          return;
        }
        elements[name].value = configMap[name];
      });
      sliders.forEach((name) => { values[name].textContent = elements[name].value; });
      elements.rows.innerHTML = state.cameras.map((camera) => `
        <tr>
          <td><input type="checkbox" data-camera="${camera.camera_id}" ${camera.selected ? "checked" : ""}></td>
          <td>${camera.camera_id}</td>
          <td>${camera.ip}</td>
          <td>${camera.diagnostics?.state ?? ""}</td>
          <td>${camera.diagnostics?.blob_diagnostics?.last_blob_count ?? ""}</td>
          <td>${(camera.diagnostics?.blob_diagnostics?.rejected_by_diameter ?? 0) + (camera.diagnostics?.blob_diagnostics?.rejected_by_circularity ?? 0)}</td>
          <td>${camera.healthy ? "yes" : "no"}</td>
          <td>${camera.last_ack ?? ""}</td>
          <td>${camera.last_error ?? ""}</td>
        </tr>
      `).join("");
      elements.status.textContent = JSON.stringify(state.last_result, null, 2);
    }

    async function postJson(url, payload) {
      const response = await fetch(url, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(payload),
      });
      return response.json();
    }

    document.getElementById("applyConfig").addEventListener("click", async () => {
      await postJson("/api/config", configPayload());
      await loadState();
    });

    document.querySelectorAll("button[data-command]").forEach((button) => {
      button.addEventListener("click", async () => {
        await postJson("/api/command", {
          command: button.dataset.command,
          camera_ids: selectedCameraIds(),
        });
        await loadState();
      });
    });

    document.getElementById("generateExtrinsics").addEventListener("click", async () => {
      await postJson("/api/generate_extrinsics", {
        intrinsics_path: elements.intrinsicsPath.value,
        log_path: elements.logPath.value,
        output_path: elements.outputPath.value,
      });
      await loadState();
    });

    sliders.forEach((name) => {
      elements[name].addEventListener("input", () => {
        values[name].textContent = elements[name].value;
        if (!sliderNames.has(name)) {
          return;
        }
        if (sliderUpdateTimers[name]) {
          clearTimeout(sliderUpdateTimers[name]);
        }
        sliderUpdateTimers[name] = setTimeout(async () => {
          try {
            await postJson("/api/config", configPayload());
            await loadState();
          } finally {
            sliderUpdateTimers[name] = null;
          }
        }, SLIDER_DEBOUNCE_MS);
      });
    });

    loadState();
    setInterval(loadState, 3000);
  </script>
</body>
</html>
"""


class WandGuiState:
    def __init__(self, session: WandSession, receiver: UDPReceiver) -> None:
        self.session = session
        self.receiver = receiver
        self.lock = threading.Lock()
        self.config = SessionConfig(exposure_us=1200, gain=4.0, fps=80, duration_s=60.0)
        self.selected_camera_ids: List[str] = []
        self.camera_status: Dict[str, Dict[str, Any]] = {}
        self.last_result: Dict[str, Any] = {"status": "idle"}
        self._extrinsics_solver = _load_extrinsics_solver()

    def _mask_params(self) -> Dict[str, Any]:
        mask = dict(self.config.mask_params or {})
        return {
            "threshold": int(mask.get("threshold", 200)),
            "seconds": float(mask.get("seconds", 0.5)),
            "hit_ratio": float(mask.get("hit_ratio", 0.7)),
            "min_area": int(mask.get("min_area", 4)),
            "dilate": int(mask.get("dilate", 2)),
        }

    def _config_payload(self) -> Dict[str, Any]:
        mask = self._mask_params()
        return {
            "exposure_us": self.config.exposure_us,
            "gain": self.config.gain,
            "fps": self.config.fps,
            "focus": self.config.focus,
            "threshold": self.config.threshold,
            "blob_min_diameter_px": self.config.blob_min_diameter_px,
            "blob_max_diameter_px": self.config.blob_max_diameter_px,
            "circularity_min": self.config.circularity_min,
            "mask_threshold": mask["threshold"],
            "mask_seconds": mask["seconds"],
        }

    def _build_session_config(self, payload: Dict[str, Any]) -> SessionConfig:
        mask = self._mask_params()
        mask["threshold"] = int(payload.get("mask_threshold", mask["threshold"]))
        mask["seconds"] = float(payload.get("mask_seconds", mask["seconds"]))
        return SessionConfig(
            exposure_us=int(payload.get("exposure_us", self.config.exposure_us)),
            gain=float(payload.get("gain", self.config.gain)),
            fps=int(payload.get("fps", self.config.fps)),
            focus=float(payload.get("focus", self.config.focus)),
            threshold=int(payload.get("threshold", self.config.threshold)),
            blob_min_diameter_px=payload.get("blob_min_diameter_px", self.config.blob_min_diameter_px),
            blob_max_diameter_px=payload.get("blob_max_diameter_px", self.config.blob_max_diameter_px),
            circularity_min=float(payload.get("circularity_min", self.config.circularity_min)),
            duration_s=self.config.duration_s,
            camera_ids=self.selected_camera_ids or None,
            mask_params=mask,
            mask_retry=self.config.mask_retry,
            output_dir=self.config.output_dir,
        )

    def _apply_capture_settings(self, targets: List[Any]) -> Dict[str, Dict[str, Dict[str, Any]]]:
        return {
            "set_exposure": self.session._broadcast(targets, "set_exposure", value=self.config.exposure_us),
            "set_gain": self.session._broadcast(targets, "set_gain", value=self.config.gain),
            "set_fps": self.session._broadcast(targets, "set_fps", value=self.config.fps),
            "set_focus": self.session._broadcast(targets, "set_focus", value=self.config.focus),
            "set_threshold": self.session._broadcast(targets, "set_threshold", value=self.config.threshold),
            "set_blob_diameter": self.session._broadcast(
                targets,
                "set_blob_diameter",
                min_px=self.config.blob_min_diameter_px,
                max_px=self.config.blob_max_diameter_px,
            ),
            "set_circularity_min": self.session._broadcast(
                targets, "set_circularity_min", value=self.config.circularity_min
            ),
        }

    def refresh_targets(self) -> List[Dict[str, Any]]:
        discovered = self.session.discover_targets(self.selected_camera_ids or None)
        status_rows: List[Dict[str, Any]] = []
        ping_results = self.session._broadcast(discovered, "ping") if discovered else {}
        with self.lock:
            for target in discovered:
                ping_result = ping_results.get(target.camera_id, {})
                healthy = bool(ping_result.get("ack"))
                diagnostics = ping_result.get("result", {}) if isinstance(ping_result.get("result"), dict) else {}
                previous = self.camera_status.get(target.camera_id, {})
                self.camera_status[target.camera_id] = {
                    **previous,
                    "camera_id": target.camera_id,
                    "ip": target.ip,
                    "healthy": healthy,
                    "selected": not self.selected_camera_ids or target.camera_id in self.selected_camera_ids,
                    "diagnostics": diagnostics,
                    "last_ack": bool(ping_result.get("ack")) if ping_result else previous.get("last_ack"),
                    "last_error": ping_result.get("error") or ping_result.get("error_message"),
                }
            status_rows = sorted(self.camera_status.values(), key=lambda item: item["camera_id"])
        return status_rows

    def get_state(self) -> Dict[str, Any]:
        cameras = self.refresh_targets()
        return {
            "config": self._config_payload(),
            "cameras": cameras,
            "last_result": self.last_result,
            "receiver": self.receiver.stats,
        }

    def apply_config(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        camera_ids = list(payload.get("camera_ids", []))
        if camera_ids:
            self.selected_camera_ids = camera_ids
        self.config = self._build_session_config(payload)

        targets = self.session.discover_targets(self.selected_camera_ids or None)
        result = self._apply_capture_settings(targets)
        self._update_camera_status(result)
        self.last_result = result
        return result

    def run_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command = str(payload.get("command", "")).strip()
        camera_ids = list(payload.get("camera_ids", []))
        if camera_ids:
            self.selected_camera_ids = camera_ids

        if command == "refresh":
            result = {"cameras": self.refresh_targets()}
            self.last_result = result
            return result

        targets = self.session.discover_targets(self.selected_camera_ids or None)
        command_handlers = {
            "ping": lambda: self.session._broadcast(targets, "ping"),
            "mask_start": lambda: self.session._broadcast(targets, "mask_start", **self._mask_params()),
            "start": lambda: self.session._broadcast(targets, "start", mode="wand_capture"),
            "stop": lambda: self.session._broadcast(targets, "stop"),
        }
        handler = command_handlers.get(command)
        if handler is None:
            raise ValueError(f"Unsupported command: {command}")
        result = handler()

        self._update_camera_status({command: result})
        self.last_result = {command: result}
        return self.last_result

    def generate_extrinsics(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        intrinsics_path = str(payload.get("intrinsics_path", "calibration")).strip()
        log_path = str(payload.get("log_path", "")).strip()
        output_path = str(
            payload.get("output_path", "calibration/calibration_extrinsics_v1.json")
        ).strip()
        if not log_path:
            raise ValueError("log_path is required")

        result = self._extrinsics_solver(
            intrinsics_path=intrinsics_path,
            log_path=log_path,
            output_path=output_path,
        )
        summary = {
            "ok": True,
            "reference_camera_id": result.get("reference_camera_id"),
            "camera_count": len(result.get("cameras", [])),
            "output_path": output_path,
        }
        self.last_result = {"generate_extrinsics": summary}
        return self.last_result

    def _update_camera_status(self, result: Dict[str, Dict[str, Dict[str, Any]]]) -> None:
        with self.lock:
            for responses in result.values():
                for camera_id, response in responses.items():
                    entry = self.camera_status.setdefault(camera_id, {"camera_id": camera_id, "ip": "unknown"})
                    entry["last_ack"] = bool(response.get("ack"))
                    entry["last_error"] = response.get("error") or response.get("error_message")


class WandGuiHandler(BaseHTTPRequestHandler):
    state: WandGuiState

    def do_GET(self) -> None:
        if self.path == "/":
            self._send_html(HTML_PAGE)
            return
        if self.path == "/api/state":
            self._send_json(self.state.get_state())
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            payload = self._read_json()
            if self.path == "/api/config":
                self._send_json(self.state.apply_config(payload))
                return
            if self.path == "/api/command":
                self._send_json(self.state.run_command(payload))
                return
            if self.path == "/api/generate_extrinsics":
                self._send_json(self.state.generate_extrinsics(payload))
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _send_html(self, body: str) -> None:
        data = body.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal web GUI for wand capture control")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP bind port")
    parser.add_argument("--udp-port", type=int, default=5000, help="Passive discovery UDP port")
    parser.add_argument("--inventory", default=None, help="Optional hosts.ini path")
    return parser.parse_args()


def _load_extrinsics_solver():
    script_path = MODULE_SRC_ROOT / "camera-calibration" / "calibrate_extrinsics.py"
    spec = importlib.util.spec_from_file_location("calibrate_extrinsics", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load solver module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["calibrate_extrinsics"] = module
    spec.loader.exec_module(module)
    solver = getattr(module, "solve_wand_extrinsics", None)
    if not callable(solver):
        raise RuntimeError("solve_wand_extrinsics is missing in calibrate_extrinsics.py")
    return solver


def main() -> None:
    args = parse_args()
    receiver = UDPReceiver(port=args.udp_port)
    receiver.start()
    session = WandSession(
        inventory_path=Path(args.inventory) if args.inventory else None,
        receiver=receiver,
    )
    state = WandGuiState(session=session, receiver=receiver)
    WandGuiHandler.state = state
    server = ThreadingHTTPServer((args.host, args.port), WandGuiHandler)
    try:
        print(f"wand GUI listening on http://{args.host}:{args.port}")
        server.serve_forever()
    finally:
        server.server_close()
        receiver.stop()


if __name__ == "__main__":
    main()
