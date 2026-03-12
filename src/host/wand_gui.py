#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
import threading
import importlib.util
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_SRC_ROOT = Path(__file__).resolve().parents[1]

if __package__ in (None, ""):
    if str(MODULE_SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(MODULE_SRC_ROOT))
    from host.receiver import UDPReceiver
    from host.logger import FrameLogger
    from host.tracking_runtime import TrackingRuntime
    from host.wand_session import SessionConfig, WandSession
else:
    from .receiver import UDPReceiver
    from .logger import FrameLogger
    from .tracking_runtime import TrackingRuntime
    from .wand_session import SessionConfig, WandSession


DEFAULT_SETTINGS_PATH = Path("logs") / "wand_gui_settings.json"
DEFAULT_POSE_LOG_PATH = Path("logs") / "extrinsics_pose_capture.jsonl"
DEFAULT_WAND_METRIC_LOG_PATH = Path("logs") / "extrinsics_wand_metric.jsonl"
DEFAULT_EXTRINSICS_OUTPUT_PATH = Path("calibration") / "calibration_extrinsics_v1.json"
DEFAULT_CAPTURE_LOG_DIR = Path("logs")
STATIC_DIR_CANDIDATES = (
    PROJECT_ROOT / "static",
    MODULE_SRC_ROOT / "static",
)


HTML_PAGE = """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Loutrack2 Wand Control</title>
  <style>
    :root {
      --bg: #f3efe5;
      --panel: rgba(255, 251, 245, 0.92);
      --panel-strong: #fffdf8;
      --ink: #1f2a30;
      --muted: #66747c;
      --line: rgba(79, 71, 59, 0.14);
      --warm: #c15b31;
      --teal: #1f6b70;
      --olive: #647447;
      --danger: #9e3f33;
      --shadow: 0 18px 40px rgba(31, 42, 48, 0.09);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: var(--ink);
      font-family: "Avenir Next", "Helvetica Neue", sans-serif;
      background:
        radial-gradient(circle at top left, rgba(255, 248, 230, 0.95), transparent 32%),
        radial-gradient(circle at top right, rgba(203, 234, 236, 0.42), transparent 28%),
        linear-gradient(180deg, #f8f3e8 0%, #f0ebe0 100%);
    }
    main {
      max-width: 1240px;
      margin: 0 auto;
      padding: 28px 20px 40px;
    }
    h1 {
      margin: 0;
      font-size: clamp(34px, 5vw, 52px);
      line-height: 1;
      letter-spacing: -0.03em;
    }
    h2, h3 {
      margin: 0;
      letter-spacing: -0.02em;
    }
    p {
      margin: 0;
      color: var(--muted);
      line-height: 1.55;
    }
    .hero,
    .card,
    .step-card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 24px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(10px);
    }
    .hero {
      padding: 24px;
      display: grid;
      gap: 18px;
    }
    .hero-top {
      display: grid;
      gap: 12px;
      align-items: start;
    }
    .hero-actions,
    .button-row {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }
    .page-nav {
      display: inline-flex;
      flex-wrap: wrap;
      gap: 10px;
    }
    .page-tab {
      background: rgba(31, 42, 48, 0.08);
      color: var(--ink);
    }
    .page-tab.active {
      background: var(--teal);
      color: #fff;
    }
    .page {
      display: none;
    }
    .page.active {
      display: block;
    }
    .tracking-empty {
      margin-bottom: 14px;
      padding: 12px 14px;
      border-radius: 16px;
      border: 1px solid var(--line);
      background: rgba(193, 91, 49, 0.10);
      color: var(--warm);
      font-weight: 700;
    }
    .hero-note {
      padding: 14px 16px;
      border-radius: 18px;
      background: linear-gradient(135deg, rgba(31, 107, 112, 0.12), rgba(193, 91, 49, 0.10));
      color: var(--ink);
    }
    .step-rail {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 10px;
    }
    .step-pill {
      padding: 14px 16px;
      border-radius: 18px;
      border: 1px solid var(--line);
      background: rgba(255, 255, 255, 0.66);
      min-height: 88px;
      display: grid;
      gap: 6px;
    }
    .step-pill small,
    .eyebrow {
      text-transform: uppercase;
      letter-spacing: 0.14em;
      font-size: 11px;
      color: var(--muted);
    }
    .step-pill[data-status="current"] {
      border-color: rgba(193, 91, 49, 0.38);
      background: rgba(255, 242, 235, 0.92);
    }
    .step-pill[data-status="done"] {
      border-color: rgba(100, 116, 71, 0.30);
      background: rgba(244, 249, 236, 0.92);
    }
    .hero-metrics,
    .camera-summary {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 10px;
    }
    .metric {
      padding: 14px 16px;
      border-radius: 18px;
      background: rgba(255, 255, 255, 0.7);
      border: 1px solid var(--line);
    }
    .metric strong {
      display: block;
      font-size: 26px;
      line-height: 1.1;
      margin-top: 6px;
    }
    .grid {
      display: grid;
      grid-template-columns: minmax(0, 1.35fr) minmax(320px, 0.95fr);
      gap: 18px;
      margin-top: 18px;
      align-items: start;
    }
    .grid > * {
      min-width: 0;
    }
    .tracking-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.4fr) minmax(280px, 0.8fr);
      gap: 18px;
      margin-top: 18px;
      align-items: stretch;
    }
    .tracking-main {
      display: grid;
      gap: 12px;
    }
    .tracking-viewer-card {
      display: grid;
      gap: 10px;
    }
    .tracking-viewer-inner {
      position: relative;
      min-height: 360px;
      border-radius: 20px;
      border: 1px solid var(--line);
      overflow: hidden;
      background: #fff;
    }
    .tracking-viewer-inner canvas {
      width: 100%;
      height: 100%;
      display: block;
    }
    .tracking-viewer-empty {
      position: absolute;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      padding: 16px;
      font-weight: 700;
      color: var(--warm);
      background: rgba(255, 255, 255, 0.85);
      pointer-events: none;
      font-size: 15px;
    }
    .tracking-empty-sm {
      padding: 10px 14px;
      border-radius: 12px;
      border: 1px dashed var(--line);
      background: rgba(255, 255, 255, 0.8);
      text-align: center;
      color: var(--muted);
      font-size: 13px;
    }
    .tracking-viewer-footer {
      color: var(--muted);
      font-size: 13px;
      display: flex;
      justify-content: space-between;
      gap: 12px;
    }
    .tracking-viewer-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
    }
    .tracking-state-badge {
      border-radius: 999px;
      padding: 6px 14px;
      background: rgba(31, 42, 48, 0.12);
      font-size: 12px;
      font-weight: 700;
    }
    .tracking-state-badge[data-state="running"] {
      background: var(--teal);
      color: #fff;
    }
    .tracking-state-badge[data-state="error"] {
      background: var(--danger);
      color: #fff;
    }
    .tracking-side {
      display: grid;
      gap: 16px;
    }
    .tracking-controls-row {
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      gap: 16px;
      align-items: center;
    }
    .tracking-control-buttons {
      display: flex;
      gap: 8px;
    }
    .tracking-control-footer {
      margin-top: 10px;
      font-size: 13px;
    }
    .tracking-health-card,
    .tracking-rigid-card {
      display: grid;
      gap: 10px;
    }
    .tracking-health-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 10px;
    }
    .tracking-health-grid {
      display: grid;
      gap: 10px;
    }
    .tracking-camera-card {
      padding: 12px;
      border-radius: 16px;
      background: var(--panel-strong);
      border: 1px solid var(--line);
      display: grid;
      gap: 6px;
    }
    .tracking-camera-card strong {
      font-size: 16px;
    }
    .tracking-card-metrics {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      gap: 6px;
      font-size: 13px;
      color: var(--muted);
    }
    .tracking-card-title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
    }
    .tracking-card-badge {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      padding: 4px 10px;
      border-radius: 999px;
      font-size: 12px;
      font-weight: 700;
    }
    .tracking-card-badge.ok {
      background: rgba(100, 116, 71, 0.16);
      color: var(--olive);
    }
    .tracking-card-badge.warn {
      background: rgba(193, 91, 49, 0.12);
      color: var(--warm);
    }
    .tracking-rigid-list {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .tracking-rigid-row {
      border-radius: 16px;
      border: 1px solid var(--line);
      padding: 12px;
      background: var(--panel-strong);
      display: grid;
      gap: 6px;
    }
    .tracking-rigid-row.invalid {
      border-color: rgba(158, 63, 51, 0.4);
    }
    .tracking-rigid-row header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
    }
    .tracking-rigid-row small {
      color: var(--muted);
    }
    .tracking-json-stack {
      display: grid;
      gap: 12px;
    }
    .stack {
      display: grid;
      gap: 18px;
      min-width: 0;
      align-content: start;
    }
    .card,
    .step-card {
      padding: 22px;
      min-width: 0;
      overflow: hidden;
    }
    .step-card {
      position: relative;
    }
    .step-card::after {
      content: "";
      position: absolute;
      inset: auto -40px -40px auto;
      width: 140px;
      height: 140px;
      border-radius: 999px;
      opacity: 0.12;
      background: radial-gradient(circle, var(--step-accent, var(--warm)), transparent 70%);
      pointer-events: none;
    }
    .step-card[data-status="current"] {
      border-color: rgba(193, 91, 49, 0.32);
    }
    .step-card[data-status="done"] {
      border-color: rgba(100, 116, 71, 0.30);
    }
    .step-card[data-step="blob"] { --step-accent: var(--warm); }
    .step-card[data-step="mask"] { --step-accent: var(--teal); }
    .step-card[data-step="wand"] { --step-accent: var(--olive); }
    .step-card[data-step="extrinsics"] { --step-accent: var(--danger); }
    .step-head {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      align-items: start;
      margin-bottom: 12px;
    }
    .step-title {
      display: grid;
      gap: 6px;
    }
    .step-status {
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 8px 12px;
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      background: rgba(31, 42, 48, 0.06);
      color: var(--muted);
      white-space: nowrap;
    }
    .step-card[data-status="current"] .step-status {
      background: rgba(193, 91, 49, 0.14);
      color: var(--warm);
    }
    .step-card[data-status="done"] .step-status {
      background: rgba(100, 116, 71, 0.16);
      color: var(--olive);
    }
    .step-summary {
      margin-bottom: 14px;
      padding: 12px 14px;
      border-radius: 16px;
      background: rgba(255, 255, 255, 0.74);
      border: 1px solid var(--line);
      color: var(--ink);
    }
    .controls-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px 18px;
    }
    label {
      display: grid;
      gap: 6px;
      font-weight: 700;
      font-size: 14px;
      color: var(--ink);
    }
    input[type="range"],
    input[type="text"] {
      width: 100%;
    }
    input[type="text"] {
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px 14px;
      background: rgba(255, 255, 255, 0.82);
      color: var(--ink);
      font: inherit;
    }
    .hint {
      font-size: 12px;
      color: var(--muted);
      font-weight: 500;
    }
    button {
      border: 0;
      border-radius: 999px;
      padding: 11px 16px;
      background: var(--warm);
      color: white;
      cursor: pointer;
      font: inherit;
      font-weight: 700;
      letter-spacing: 0.01em;
    }
    button.secondary { background: var(--teal); }
    button.ghost {
      background: rgba(31, 42, 48, 0.08);
      color: var(--ink);
    }
    .selection-list {
      margin-top: 14px;
      overflow-x: auto;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
      min-width: 760px;
    }
    th, td {
      text-align: left;
      padding: 12px 10px;
      border-bottom: 1px solid var(--line);
      vertical-align: middle;
    }
    th {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }
    .badge {
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 5px 10px;
      font-size: 12px;
      font-weight: 700;
      background: rgba(31, 42, 48, 0.08);
      color: var(--ink);
    }
    .badge.ok {
      background: rgba(100, 116, 71, 0.16);
      color: var(--olive);
    }
    .badge.warn {
      background: rgba(193, 91, 49, 0.14);
      color: var(--warm);
    }
    .badge.muted {
      background: rgba(31, 42, 48, 0.06);
      color: var(--muted);
    }
    .console {
      background: #202a31;
      color: #f5efe2;
      padding: 14px;
      border-radius: 18px;
      overflow: auto;
      min-height: 240px;
      max-width: 100%;
      max-height: min(70vh, 960px);
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      word-break: break-word;
    }
    @media (max-width: 980px) {
      .grid {
        grid-template-columns: 1fr;
      }
      .tracking-grid {
        grid-template-columns: 1fr;
      }
    }
    @media (max-width: 720px) {
      main {
        padding: 20px 14px 32px;
      }
      .controls-grid {
        grid-template-columns: 1fr;
      }
      .tracking-grid {
        grid-template-columns: 1fr;
      }
      .tracking-controls-row {
        flex-direction: column;
        align-items: flex-start;
      }
    }
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <div class="hero-top">
        <div>
          <div class="eyebrow">Loutrack2 Wand Workflow</div>
          <h1>Blob -> Mask -> Wand -> Extrinsics</h1>
        </div>
        <p>操作を 4 セグメントに固定し、選択中カメラの状態を段階ごとに確認しながら外部較正まで進める。</p>
      </div>
      <div class="hero-note">
        Pi を <code>--debug-preview</code> 付きで起動していれば、blob 調整、mask 結果、wand 収録中の OpenCV preview が Pi デスクトップに継続表示されます。
      </div>
      <div class="hero-actions">
        <div class="page-nav" aria-label="page navigation">
          <button id="tabCalibration" class="page-tab active" type="button">Calibration</button>
          <button id="tabTracking" class="page-tab" type="button">Tracking</button>
        </div>
        <button class="secondary" data-command="refresh">Refresh</button>
        <button class="ghost" data-command="ping">Ping</button>
      </div>
      <div class="step-rail" id="workflowRail"></div>
      <div class="hero-metrics">
        <div class="metric"><span class="eyebrow">Selection</span><strong id="selectionMetric">0 / 0</strong><p id="selectionSummary"></p></div>
        <div class="metric"><span class="eyebrow">Blob Ready</span><strong id="blobMetric">0</strong><p id="blobMetricSummary"></p></div>
        <div class="metric"><span class="eyebrow">Mask Ready</span><strong id="maskMetric">0</strong><p id="maskMetricSummary"></p></div>
        <div class="metric"><span class="eyebrow">Preview</span><strong id="previewMetric">0</strong><p id="previewMetricSummary"></p></div>
      </div>
    </section>

    <section id="pageCalibration" class="page active">
    <div class="grid">
      <div class="stack">
        <section class="card">
          <div class="step-head">
            <div class="step-title">
              <div class="eyebrow">Selection</div>
              <h2>Camera Fleet</h2>
            </div>
          </div>
          <div class="camera-summary" id="cameraSummary"></div>
          <div class="selection-list">
            <table>
              <thead>
                <tr>
                  <th>Select</th>
                  <th>Camera</th>
                  <th>IP</th>
                  <th>State</th>
                  <th>Blobs</th>
                  <th>Reject</th>
                  <th>Mask</th>
                  <th>Preview</th>
                  <th>Healthy</th>
                  <th>Last Error</th>
                </tr>
              </thead>
              <tbody id="cameraRows"></tbody>
            </table>
          </div>
        </section>

        <section class="step-card" id="stepBlob" data-step="blob">
          <div class="step-head">
            <div class="step-title">
              <div class="eyebrow">Step 01</div>
              <h2>Blob Detection Adjustment</h2>
            </div>
            <span class="step-status" id="stepBlobStatus">Pending</span>
          </div>
          <div class="step-summary" id="blobSummary"></div>
          <div class="controls-grid">
            <label for="exposure">Exposure <span class="hint"><span id="exposureValue"></span> us</span><input id="exposure" type="range" min="100" max="30000" step="100" value="12000"></label>
            <label for="gain">Gain <span class="hint"><span id="gainValue"></span></span><input id="gain" type="range" min="1" max="16" step="0.1" value="8"></label>
            <label for="fps">FPS <span class="hint"><span id="fpsValue"></span></span><input id="fps" type="range" min="15" max="120" step="1" value="56"></label>
            <label for="focus">Focus <span class="hint"><span id="focusValue"></span></span><input id="focus" type="range" min="0.0" max="10.0" step="0.001" value="5.215"></label>
            <label for="threshold">Threshold <span class="hint"><span id="thresholdValue"></span></span><input id="threshold" type="range" min="0" max="255" step="1" value="200"></label>
            <label for="circularity">Circularity Min <span class="hint"><span id="circularityValue"></span></span><input id="circularity" type="range" min="0.0" max="1.0" step="0.01" value="0.0"></label>
            <label for="blobMin">Blob Min Diameter <span class="hint"><span id="blobMinValue"></span> px</span><input id="blobMin" type="range" min="0" max="100" step="0.5" value="0"></label>
            <label for="blobMax">Blob Max Diameter <span class="hint"><span id="blobMaxValue"></span> px</span><input id="blobMax" type="range" min="0" max="100" step="0.5" value="0"></label>
          </div>
          <div class="button-row">
            <button id="applyConfig">Apply Blob Settings</button>
          </div>
        </section>

        <section class="step-card" id="stepMask" data-step="mask">
          <div class="step-head">
            <div class="step-title">
              <div class="eyebrow">Step 02</div>
              <h2>Mask Adjustment</h2>
            </div>
            <span class="step-status" id="stepMaskStatus">Pending</span>
          </div>
          <div class="step-summary" id="maskSummary"></div>
          <div class="controls-grid">
            <label for="maskThreshold">Mask Threshold <span class="hint"><span id="maskThresholdValue"></span></span><input id="maskThreshold" type="range" min="0" max="255" step="1" value="200"></label>
            <label for="maskSeconds">Mask Seconds <span class="hint"><span id="maskSecondsValue"></span> s</span><input id="maskSeconds" type="range" min="0.1" max="5.0" step="0.1" value="0.5"></label>
          </div>
          <div class="button-row">
            <button class="secondary" data-command="mask_start">Build Mask</button>
            <button class="ghost" data-command="mask_stop">Clear Mask</button>
          </div>
        </section>

        <section class="step-card" id="stepWand" data-step="wand">
          <div class="step-head">
            <div class="step-title">
              <div class="eyebrow">Step 03</div>
              <h2>Wand Capture</h2>
            </div>
            <span class="step-status" id="stepWandStatus">Pending</span>
          </div>
          <div class="step-summary" id="wandSummary"></div>
          <div class="button-row">
            <button class="secondary" data-command="start">Start Pose Capture</button>
            <button class="ghost" data-command="stop">Stop Capture</button>
            <button class="secondary" data-command="start_wand_metric_capture">Start Wand Metric Capture</button>
            <button class="ghost" data-command="stop_wand_metric_capture">Stop Metric Capture</button>
          </div>
        </section>

        <section class="step-card" id="stepExtrinsics" data-step="extrinsics">
          <div class="step-head">
            <div class="step-title">
              <div class="eyebrow">Step 04</div>
              <h2>Extrinsics Generation</h2>
            </div>
            <span class="step-status" id="stepExtrinsicsStatus">Pending</span>
          </div>
          <div class="step-summary" id="extrinsicsSummary"></div>
          <div class="controls-grid">
            <label for="intrinsicsPath">Intrinsics Dir<input id="intrinsicsPath" type="text" value="calibration"></label>
             <label for="logPath">Pose Log Path<input id="logPath" type="text" value="logs/extrinsics_pose_capture.jsonl"></label>
            <label for="wandMetricLogPath">Wand Metric Log<input id="wandMetricLogPath" type="text" value="logs/extrinsics_wand_metric.jsonl"></label>
            <label for="outputPath">Output Path<input id="outputPath" type="text" value="calibration/calibration_extrinsics_v1.json"></label>
            <label for="pairWindowUs">Pair Window (us)<input id="pairWindowUs" type="number" min="1" step="100" value="8000"></label>
            <label for="minPairs">Min Pairs<input id="minPairs" type="number" min="1" step="1" value="8"></label>
            <label for="maxWandMetricSamples">Wand Metric Samples<input id="maxWandMetricSamples" type="number" min="1" step="1" value="16"></label>
            <label for="expectedBaselineM">Expected Baseline (m)<input id="expectedBaselineM" type="number" min="0" step="0.01" value=""></label>
          </div>
          <div class="button-row">
            <button id="generateExtrinsics">Generate Extrinsics</button>
            <button class="secondary" id="applyWandScaleFloor">Apply Wand Scale/Floor</button>
          </div>
        </section>
      </div>

      <div class="stack">
        <section class="card">
          <div class="step-head">
            <div class="step-title">
              <div class="eyebrow">Console</div>
              <h2>Last Result</h2>
            </div>
          </div>
          <pre class="console" id="status"></pre>
        </section>
      </div>
    </div>
    </section>

    <section id="pageTracking" class="page">
      <div class="tracking-grid">
        <div class="tracking-main">
          <section class="card tracking-viewer-card">
            <div class="tracking-viewer-header">
              <div class="tracking-viewer-title">
                <div class="eyebrow">Tracking Scene</div>
                <h2>3D Monitoring</h2>
              </div>
              <span class="tracking-state-badge" id="trackingStatusBadge">Idle</span>
            </div>
            <div class="tracking-viewer-inner">
              <canvas id="trackingViewerCanvas" aria-label="Tracking scene viewport"></canvas>
              <div class="tracking-viewer-empty" id="trackingViewerEmpty">Waiting for scene data</div>
            </div>
            <div class="tracking-viewer-footer">
              <span id="trackingSceneTimestamp">Last scene: --</span>
              <span id="trackingSceneHint"></span>
            </div>
          </section>
          <div class="tracking-json-stack">
            <section class="card">
              <div class="step-head">
                <div class="step-title">
                  <h2>Status JSON</h2>
                </div>
              </div>
              <pre class="console" id="trackingStatus"></pre>
            </section>
            <section class="card">
              <div class="step-head">
                <div class="step-title">
                  <h2>Scene Snapshot JSON</h2>
                </div>
              </div>
              <pre class="console" id="trackingScene"></pre>
            </section>
          </div>
        </div>
        <div class="tracking-side">
          <section class="card tracking-controls">
            <div class="tracking-controls-row">
              <div>
                <p class="eyebrow">Tracking Control</p>
                <p id="trackingExtrinsicsPath"></p>
              </div>
              <div class="tracking-control-buttons">
                <button id="trackingStart" class="secondary" type="button">Start Tracking</button>
                <button id="trackingStop" class="ghost" type="button">Stop Tracking</button>
              </div>
            </div>
            <div class="tracking-control-footer">
              <span id="trackingLastUpdate">waiting for data</span>
            </div>
          </section>
          <section class="card tracking-health-card">
            <div class="tracking-health-header">
              <div class="eyebrow">Camera Health</div>
              <span id="trackingSyncCoverage"></span>
            </div>
            <div id="trackingCameraHealth" class="tracking-health-grid"></div>
          </section>
          <section class="card tracking-rigid-card">
            <div class="tracking-health-header">
              <div class="eyebrow">Rigid Bodies</div>
              <span id="trackingRigidCount">0 tracked</span>
            </div>
            <div id="trackingRigidList" class="tracking-rigid-list"></div>
          </section>
        </div>
      </div>
    </section>
  </main>
  <script type="module">
    let THREE = null;
    async function ensureThreeLoaded() {
      if (THREE) {
        return THREE;
      }
      THREE = await import("/static/vendor/three.module.min.js");
      return THREE;
    }
    const sliders = ["exposure", "gain", "fps", "focus", "threshold", "circularity", "blobMin", "blobMax", "maskThreshold", "maskSeconds"];  
    const sliderNames = new Set(sliders);
    const stepOrder = ["blob", "mask", "wand", "extrinsics"];
    const stepLabels = {
      blob: "Blob",
      mask: "Mask",
      wand: "Wand",
      extrinsics: "Extrinsics",
    };
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
      cameraSummary: document.getElementById("cameraSummary"),
      workflowRail: document.getElementById("workflowRail"),
      status: document.getElementById("status"),
      intrinsicsPath: document.getElementById("intrinsicsPath"),
      logPath: document.getElementById("logPath"),
      wandMetricLogPath: document.getElementById("wandMetricLogPath"),
      outputPath: document.getElementById("outputPath"),
      pairWindowUs: document.getElementById("pairWindowUs"),
      minPairs: document.getElementById("minPairs"),
      maxWandMetricSamples: document.getElementById("maxWandMetricSamples"),
      expectedBaselineM: document.getElementById("expectedBaselineM"),
      selectionMetric: document.getElementById("selectionMetric"),
      selectionSummary: document.getElementById("selectionSummary"),
      blobMetric: document.getElementById("blobMetric"),
      blobMetricSummary: document.getElementById("blobMetricSummary"),
      maskMetric: document.getElementById("maskMetric"),
      maskMetricSummary: document.getElementById("maskMetricSummary"),
      previewMetric: document.getElementById("previewMetric"),
      previewMetricSummary: document.getElementById("previewMetricSummary"),
      blobSummary: document.getElementById("blobSummary"),
      maskSummary: document.getElementById("maskSummary"),
      wandSummary: document.getElementById("wandSummary"),
      extrinsicsSummary: document.getElementById("extrinsicsSummary"),
      stepBlob: document.getElementById("stepBlob"),
      stepMask: document.getElementById("stepMask"),
      stepWand: document.getElementById("stepWand"),
      stepExtrinsics: document.getElementById("stepExtrinsics"),
      stepBlobStatus: document.getElementById("stepBlobStatus"),
      stepMaskStatus: document.getElementById("stepMaskStatus"),
      stepWandStatus: document.getElementById("stepWandStatus"),
      stepExtrinsicsStatus: document.getElementById("stepExtrinsicsStatus"),
      tabCalibration: document.getElementById("tabCalibration"),
      tabTracking: document.getElementById("tabTracking"),
      pageCalibration: document.getElementById("pageCalibration"),
      pageTracking: document.getElementById("pageTracking"),
      trackingViewerCanvas: document.getElementById("trackingViewerCanvas"),
      trackingViewerEmpty: document.getElementById("trackingViewerEmpty"),
      trackingStatusBadge: document.getElementById("trackingStatusBadge"),
      trackingSceneTimestamp: document.getElementById("trackingSceneTimestamp"),
      trackingSceneHint: document.getElementById("trackingSceneHint"),
      trackingLastUpdate: document.getElementById("trackingLastUpdate"),
      trackingSyncCoverage: document.getElementById("trackingSyncCoverage"),
      trackingCameraHealth: document.getElementById("trackingCameraHealth"),
      trackingRigidList: document.getElementById("trackingRigidList"),
      trackingRigidCount: document.getElementById("trackingRigidCount"),
      trackingStart: document.getElementById("trackingStart"),
      trackingStop: document.getElementById("trackingStop"),
      trackingStatus: document.getElementById("trackingStatus"),
      trackingScene: document.getElementById("trackingScene"),
      trackingExtrinsicsPath: document.getElementById("trackingExtrinsicsPath"),
    };
    let trackingViewer = createTrackingViewer();
    const sliderUpdateTimers = {};
    function createTrackingViewer() {
      const canvas = elements.trackingViewerCanvas;
      const overlay = elements.trackingViewerEmpty;
      if (!canvas || !overlay) {
        return { update: () => {}, setEmpty: () => {}, resize: () => {} };
      }
      if (!THREE) {
        const fallbackMessage = "3D viewer loading...";
        overlay.textContent = fallbackMessage;
        overlay.style.display = "flex";
        return {
          update: () => {},
          setEmpty(message) {
            overlay.textContent = message || fallbackMessage;
            overlay.style.display = "flex";
          },
          resize: () => {},
        };
      }
      let renderer;
      try {
        renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
      } catch (error) {
        const fallbackMessage = "3D viewer unavailable (WebGL unsupported). Calibration controls still work.";
        overlay.textContent = fallbackMessage;
        overlay.style.display = "flex";
        return {
          update: () => {},
          setEmpty(message) {
            overlay.textContent = message || fallbackMessage;
            overlay.style.display = "flex";
          },
          resize: () => {},
        };
      }
      renderer.setPixelRatio(window.devicePixelRatio || 1);
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0xf7f3ee);
      const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 25);
      camera.position.set(0, 1.2, 2.4);

      function resizeViewer() {
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;
        if (!width || !height) {
          return;
        }
        renderer.setSize(width, height, false);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
      }

      const target = new THREE.Vector3(0, 1, 0);
      let azimuth = 0;
      let polar = Math.PI / 3;
      let distance = 2.4;
      let dragging = false;
      const cursor = { x: 0, y: 0 };

      function updateCameraPosition() {
        const spherical = new THREE.Spherical(distance, polar, azimuth);
        const offset = new THREE.Vector3();
        offset.setFromSpherical(spherical);
        offset.add(target);
        camera.position.copy(offset);
        camera.lookAt(target);
      }

      function handlePointerDown(event) {
        dragging = true;
        cursor.x = event.clientX;
        cursor.y = event.clientY;
        canvas.setPointerCapture?.(event.pointerId);
      }

      function handlePointerMove(event) {
        if (!dragging) {
          return;
        }
        const deltaX = event.clientX - cursor.x;
        const deltaY = event.clientY - cursor.y;
        cursor.x = event.clientX;
        cursor.y = event.clientY;
        azimuth -= deltaX * 0.005;
        polar = Math.min(Math.PI - 0.1, Math.max(0.1, polar - deltaY * 0.005));
      }

      function handlePointerUp(event) {
        dragging = false;
        canvas.releasePointerCapture?.(event.pointerId);
      }

      canvas.addEventListener("pointerdown", handlePointerDown);
      canvas.addEventListener("pointermove", handlePointerMove);
      canvas.addEventListener("pointerup", handlePointerUp);
      canvas.addEventListener("pointerleave", () => {
        dragging = false;
      });
      canvas.addEventListener(
        "wheel",
        (event) => {
          event.preventDefault();
          distance = Math.min(8, Math.max(0.6, distance + event.deltaY * 0.01));
        },
        { passive: false }
      );

      resizeViewer();
      window.addEventListener("resize", resizeViewer);

      const grid = new THREE.GridHelper(6, 24, 0xd6d0c6, 0xf4efe6);
      grid.rotation.x = Math.PI / 2;
      scene.add(grid);
      const axisHelper = new THREE.AxesHelper(0.45);
      scene.add(axisHelper);

      const rawPoints = new THREE.Points(
        new THREE.BufferGeometry(),
        new THREE.PointsMaterial({ color: 0xff6b35, size: 0.04, transparent: true, opacity: 0.75 })
      );
      scene.add(rawPoints);

      const cameraHelpers = new Map();
      const rigidsContainer = new THREE.Group();
      scene.add(rigidsContainer);
      const rigidHelpers = new Map();

      function animate() {
        updateCameraPosition();
        renderer.render(scene, camera);
        requestAnimationFrame(animate);
      }
      requestAnimationFrame(animate);

      function buildFrustumSegments(cameraData) {
        const corners = Array.isArray(cameraData.frustum_near_corners)
          ? cameraData.frustum_near_corners.map((corner) => new THREE.Vector3(corner[0], corner[1], corner[2]))
          : [];
        const origin = new THREE.Vector3(...(cameraData.position || [0, 0, 0]));
        const positions = [];
        if (corners.length === 4) {
          corners.forEach((corner) => {
            positions.push(origin.x, origin.y, origin.z, corner.x, corner.y, corner.z);
          });
          for (let i = 0; i < corners.length; i += 1) {
            const current = corners[i];
            const next = corners[(i + 1) % corners.length];
            positions.push(current.x, current.y, current.z, next.x, next.y, next.z);
          }
        }
        return positions;
      }

      function updateCameras(list) {
        const seen = new Set();
        (list || []).forEach((cameraData) => {
          const cameraId = String(cameraData.camera_id || "camera");
          seen.add(cameraId);
          let entry = cameraHelpers.get(cameraId);
          if (!entry) {
            const frustumGeo = new THREE.BufferGeometry();
            const frustum = new THREE.LineSegments(
              frustumGeo,
              new THREE.LineBasicMaterial({ color: 0x1f6b70 })
            );
            const originSphere = new THREE.Mesh(
              new THREE.SphereGeometry(0.03, 10, 10),
              new THREE.MeshBasicMaterial({ color: 0x1f6b70 })
            );
            const group = new THREE.Group();
            group.add(frustum);
            group.add(originSphere);
            scene.add(group);
            entry = { group, frustum, originSphere };
            cameraHelpers.set(cameraId, entry);
          }
          const positions = buildFrustumSegments(cameraData);
          entry.frustum.geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
          entry.frustum.geometry.attributes.position.needsUpdate = true;
          const pos = cameraData.position || [0, 0, 0];
          entry.originSphere.position.set(pos[0], pos[1], pos[2]);
        });
        cameraHelpers.forEach((entry, id) => {
          if (!seen.has(id)) {
            scene.remove(entry.group);
            cameraHelpers.delete(id);
          }
        });
      }

      function updateRigids(list) {
        const seen = new Set();
        (list || []).forEach((body) => {
          const name = String(body.name || "rigid");
          seen.add(name);
          let entry = rigidHelpers.get(name);
          if (!entry) {
            const group = new THREE.Group();
            const axisGeo = new THREE.BufferGeometry();
            axisGeo.setAttribute(
              "position",
              new THREE.Float32BufferAttribute(
                [0, 0, 0, 0.18, 0, 0, 0, 0, 0, 0, 0.18, 0, 0, 0, 0, 0, 0, 0.18],
                3
              )
            );
            const axisMaterial = new THREE.LineBasicMaterial({ color: 0xffffff });
            const axis = new THREE.LineSegments(axisGeo, axisMaterial);
            const trailLine = new THREE.Line(
              new THREE.BufferGeometry(),
              new THREE.LineBasicMaterial({ color: 0x1f6b70, transparent: true, opacity: 0.8 })
            );
            const markerParent = new THREE.Group();
            group.add(axis);
            group.add(trailLine);
            group.add(markerParent);
            rigidsContainer.add(group);
            entry = { group, axis, trailLine, markerParent };
            rigidHelpers.set(name, entry);
          }
          const { group, axis, trailLine, markerParent } = entry;
          const position = body.position || [0, 0, 0];
          group.position.set(position[0], position[1], position[2]);
          const quaternion = body.quaternion || [1, 0, 0, 0];
          group.quaternion.set(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
          axis.material.color.set(body.valid ? 0x1f6b70 : 0x9e3f33);
          const trailPoints = [];
          (body.trail || []).forEach((point) => {
            trailPoints.push(point[0], point[1], point[2]);
          });
          if (trailPoints.length) {
            trailLine.geometry.setAttribute(
              "position",
              new THREE.Float32BufferAttribute(trailPoints, 3)
            );
            trailLine.geometry.attributes.position.needsUpdate = true;
            trailLine.visible = true;
          } else {
            trailLine.geometry.setAttribute("position", new THREE.Float32BufferAttribute([], 3));
            trailLine.geometry.attributes.position.needsUpdate = true;
            trailLine.visible = false;
          }
          while (markerParent.children.length) {
            markerParent.remove(markerParent.children[0]);
          }
          (body.markers_world || []).forEach((marker) => {
            const sphere = new THREE.Mesh(
              new THREE.SphereGeometry(0.011, 8, 8),
              new THREE.MeshBasicMaterial({ color: 0xffc107 })
            );
            sphere.position.set(marker[0], marker[1], marker[2]);
            markerParent.add(sphere);
          });
        });
        rigidHelpers.forEach((entry, id) => {
          if (!seen.has(id)) {
            rigidsContainer.remove(entry.group);
            rigidHelpers.delete(id);
          }
        });
      }

      function updateRawPoints(points = []) {
        if (!Array.isArray(points) || points.length === 0) {
          rawPoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute([], 3));
          rawPoints.geometry.attributes.position.needsUpdate = true;
          rawPoints.visible = false;
          return;
        }
        const buffer = new Float32Array(points.length * 3);
        points.forEach((point, index) => {
          buffer[index * 3 + 0] = point[0];
          buffer[index * 3 + 1] = point[1];
          buffer[index * 3 + 2] = point[2];
        });
        rawPoints.geometry.setAttribute("position", new THREE.BufferAttribute(buffer, 3));
        rawPoints.geometry.attributes.position.needsUpdate = true;
        rawPoints.visible = true;
      }

      function showOverlay(message) {
        overlay.textContent = message || "";
        overlay.style.display = message ? "flex" : "none";
      }

      return {
        update(sceneSnapshot = {}, emptyMessage = "") {
          updateCameras(sceneSnapshot.cameras || []);
          updateRigids(sceneSnapshot.rigid_bodies || []);
          updateRawPoints(sceneSnapshot.raw_points || []);
          const hasGeometry =
            (Array.isArray(sceneSnapshot.cameras) && sceneSnapshot.cameras.length > 0) ||
            (Array.isArray(sceneSnapshot.rigid_bodies) && sceneSnapshot.rigid_bodies.length > 0);
          if (!hasGeometry) {
            showOverlay(emptyMessage || "Waiting for scene data");
          } else {
            showOverlay("");
          }
        },
        setEmpty(message) {
          showOverlay(message);
        },
        resize: resizeViewer,
      };
    }
    const SLIDER_DEBOUNCE_MS = 200;
    let activePage = "calibration";
    let loadStateInFlight = false;

    function reportUiError(context, error) {
      const message = error instanceof Error ? error.message : String(error);
      elements.status.textContent = JSON.stringify({ error: `${context}: ${message}` }, null, 2);
    }

    async function initTrackingViewer() {
      try {
        await ensureThreeLoaded();
        trackingViewer = createTrackingViewer();
      } catch (error) {
        reportUiError("three_import", error);
        trackingViewer.setEmpty("3D viewer unavailable (three.js import failed). Calibration controls still work.");
      }
    }

    function selectedCameraIds() {
      return [...document.querySelectorAll("input[data-camera]:checked")].map((item) => item.dataset.camera);
    }

    function normalizedBlobRange() {
      let minValue = Number(elements.blobMin.value);
      let maxValue = Number(elements.blobMax.value);

      minValue = Number.isFinite(minValue) && minValue > 0 ? minValue : 0;
      maxValue = Number.isFinite(maxValue) && maxValue > 0 ? maxValue : 0;

      if (minValue > 0 && maxValue > 0 && minValue > maxValue) {
        maxValue = minValue;
        elements.blobMax.value = String(maxValue);
        values.blobMax.textContent = elements.blobMax.value;
      }

      return {
        min: minValue > 0 ? minValue : null,
        max: maxValue > 0 ? maxValue : null,
      };
    }

    function configPayload() {
      const blobRange = normalizedBlobRange();
      return {
        camera_ids: selectedCameraIds(),
        exposure_us: Number(elements.exposure.value),
        gain: Number(elements.gain.value),
        fps: Number(elements.fps.value),
        focus: Number(elements.focus.value),
        threshold: Number(elements.threshold.value),
        circularity_min: Number(elements.circularity.value),
        blob_min_diameter_px: blobRange.min,
        blob_max_diameter_px: blobRange.max,
        mask_threshold: Number(elements.maskThreshold.value),
        mask_seconds: Number(elements.maskSeconds.value),
      };
    }

    function escapeHtml(value) {
      return String(value ?? "")
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;");
    }

    function badge(label, variant) {
      return `<span class="badge ${variant || "muted"}">${escapeHtml(label)}</span>`;
    }

    function statusName(status) {
      if (status === "done") return "Complete";
      if (status === "current") return "Current";
      return "Pending";
    }

    function segmentStatuses(workflow) {
      const activeIndex = Math.max(0, stepOrder.indexOf(workflow.active_segment || "blob"));
      return Object.fromEntries(stepOrder.map((step, index) => {
        let status = "pending";
        if (index < activeIndex) {
          status = "done";
        } else if (index === activeIndex) {
          status = "current";
        }
        if (step === "extrinsics" && workflow.metric_floor_applied) {
          status = "done";
        }
        if (step === "wand" && workflow.pose_capture_complete) {
          status = "done";
        }
        return [step, status];
      }));
    }

    function renderWorkflow(state) {
      const workflow = state.workflow || {};
      const selectedCount = workflow.selected_count ?? 0;
      const totalCount = workflow.total_count ?? state.cameras.length;
      const statuses = segmentStatuses(workflow);

      elements.workflowRail.innerHTML = stepOrder.map((step, index) => `
        <div class="step-pill" data-status="${statuses[step]}">
          <small>Step ${String(index + 1).padStart(2, "0")}</small>
          <strong>${stepLabels[step]}</strong>
          <span>${statusName(statuses[step])}</span>
        </div>
      `).join("");

      elements.selectionMetric.textContent = `${selectedCount} / ${totalCount}`;
      elements.selectionSummary.textContent = `${workflow.healthy_count ?? 0} healthy, ${workflow.running_count ?? 0} running`;
      elements.blobMetric.textContent = `${workflow.blob_ready_count ?? 0}`;
      elements.blobMetricSummary.textContent = `${selectedCount || totalCount}台のうち検出中`;
      elements.maskMetric.textContent = `${workflow.mask_ready_count ?? 0}`;
      elements.maskMetricSummary.textContent = `${selectedCount || totalCount}台のうち mask 準備済み`;
      elements.previewMetric.textContent = `${workflow.preview_enabled_count ?? 0}`;
      elements.previewMetricSummary.textContent = `${workflow.preview_active_count ?? 0}台で preview active`;

      elements.blobSummary.textContent = `${workflow.blob_ready_count ?? 0}/${selectedCount || totalCount} 台で blob が見えています。Pi preview 上で反射マーカーが安定して 3 点前後に見えるまで threshold / diameter / circularity を詰めます。`;
      elements.maskSummary.textContent = `${workflow.mask_ready_count ?? 0}/${selectedCount || totalCount} 台で mask が準備できています。preview に mask overlay が出ていることと、mask ratio warning が出ていないことを確認します。`;
      elements.wandSummary.textContent = (workflow.running_count ?? 0) > 0
        ? `${workflow.running_count} 台が wand capture 中です。Pi デスクトップ preview で wand の追従と誤検出の有無を見ます。`
        : workflow.pose_capture_complete
        ? `Pose capture は完了しています（${workflow.pose_capture_log_path || "logs/extrinsics_pose_capture.jsonl"}）。続けて Wand Metric Capture を完了させます。`
        : `mask 完了後に Start Pose Capture を実行します。収録中は Pi preview で単一点ターゲット追従を監視します。`;
      elements.extrinsicsSummary.textContent = workflow.metric_floor_applied
        ? `Extrinsics と wand floor metric は適用済みです。tracking に進めます。`
        : workflow.extrinsics_ready
        ? `Similarity extrinsics は生成済みです。Wand Metric Log を確認して Apply Wand Scale/Floor を実行します。`
        : `pose capture 後に Generate Extrinsics を実行し、その後 Apply Wand Scale/Floor で floor と world 座標系を確定します。`;

      const stepMap = {
        blob: elements.stepBlob,
        mask: elements.stepMask,
        wand: elements.stepWand,
        extrinsics: elements.stepExtrinsics,
      };
      const stepStatusMap = {
        blob: elements.stepBlobStatus,
        mask: elements.stepMaskStatus,
        wand: elements.stepWandStatus,
        extrinsics: elements.stepExtrinsicsStatus,
      };
      stepOrder.forEach((step) => {
        stepMap[step].dataset.status = statuses[step];
        stepStatusMap[step].textContent = statusName(statuses[step]);
      });
    }

    function renderCameraSummary(cameras) {
      const selected = cameras.filter((camera) => camera.selected);
      const healthy = selected.filter((camera) => camera.healthy).length;
      const running = selected.filter((camera) => camera.diagnostics?.state === "RUNNING").length;
      const preview = selected.filter((camera) => camera.diagnostics?.debug_preview_enabled).length;
      elements.cameraSummary.innerHTML = [
        `<div class="metric"><span class="eyebrow">Selected</span><strong>${selected.length}</strong><p>現在の操作対象</p></div>`,
        `<div class="metric"><span class="eyebrow">Healthy</span><strong>${healthy}</strong><p>ping ack=true</p></div>`,
        `<div class="metric"><span class="eyebrow">Running</span><strong>${running}</strong><p>wand capture 実行中</p></div>`,
        `<div class="metric"><span class="eyebrow">Preview Enabled</span><strong>${preview}</strong><p>Pi desktop OpenCV</p></div>`,
      ].join("");
    }

    function renderCameraRows(cameras) {
      elements.rows.innerHTML = cameras.map((camera) => {
        const diagnostics = camera.diagnostics || {};
        const blob = diagnostics.blob_diagnostics || {};
        const rejectCount = Number(blob.rejected_by_diameter || 0) + Number(blob.rejected_by_circularity || 0);
        const maskRatio = diagnostics.mask_ratio != null ? `${(Number(diagnostics.mask_ratio) * 100).toFixed(1)}%` : "-";
        const previewState = diagnostics.debug_preview_enabled
          ? diagnostics.debug_preview_active ? badge("Active", "ok") : badge("Enabled", "warn")
          : badge("Off", "muted");
        return `
          <tr>
            <td><input type="checkbox" data-camera="${escapeHtml(camera.camera_id)}" ${camera.selected ? "checked" : ""}></td>
            <td>${escapeHtml(camera.camera_id)}</td>
            <td>${escapeHtml(camera.ip)}</td>
            <td>${escapeHtml(diagnostics.state || "")}</td>
            <td>${escapeHtml(blob.last_blob_count ?? "")}</td>
            <td>${escapeHtml(rejectCount)}</td>
            <td>${escapeHtml(maskRatio)}</td>
            <td>${previewState}</td>
            <td>${camera.healthy ? badge("OK", "ok") : badge("No Ack", "warn")}</td>
            <td>${escapeHtml(camera.last_error || "")}</td>
        </tr>
      `;
      }).join("");
    }

    function formatTrackingAge(timestampUs) {
      if (!timestampUs) {
        return "waiting for data";
      }
      const diffMs = Math.max(0, Date.now() - timestampUs / 1000);
      const seconds = Math.round(diffMs / 1000);
      return `Updated ${seconds}s ago`;
    }

    function renderTrackingBadge(status) {
      const running = Boolean(status.running);
      const state = running ? "running" : status.calibration_loaded ? "idle" : "error";
      const text = running ? "Tracking" : status.calibration_loaded ? "Ready" : "Waiting";
      elements.trackingStatusBadge.textContent = text;
      elements.trackingStatusBadge.dataset.state = state;
    }

    function renderTrackingFooter(status, scene) {
      const timestampUs = scene.timestamp_us || 0;
      const timestamp = timestampUs ? new Date(timestampUs / 1000) : null;
      elements.trackingSceneTimestamp.textContent = timestamp
        ? `Last scene: ${timestamp.toLocaleTimeString()}`
        : "Last scene: --";
      elements.trackingLastUpdate.textContent = formatTrackingAge(timestampUs);
      const cameraCount = (scene.cameras || []).length;
      const rigidCount = (scene.rigid_bodies || []).length;
      elements.trackingSceneHint.textContent = `${cameraCount} cameras · ${rigidCount} bodies`;
      elements.trackingRigidCount.textContent = `${rigidCount} tracked`;
      const coverage = status.sync?.coverage_5000us ?? status.sync?.coverage_target ?? null;
      elements.trackingSyncCoverage.textContent = coverage != null
        ? `Sync coverage ${(coverage * 100).toFixed(1)}%`
        : "";
    }

    function renderCameraHealth(status) {
      const cameras = status.metrics?.cameras || {};
      const entries = Object.entries(cameras);
      if (!entries.length) {
        elements.trackingCameraHealth.innerHTML = '<div class="tracking-empty-sm">Waiting for camera metrics</div>';
        return;
      }
      elements.trackingCameraHealth.innerHTML = entries
        .map(([cameraId, metrics]) => {
          const latency = Number(metrics.latency_ms ?? 0);
          const fps = Number(metrics.fps ?? 0);
          const blobAvg = Number(metrics.blob_count_avg ?? 0);
          const statusClass = latency > 120 ? "warn" : "ok";
          return `
            <article class="tracking-camera-card">
              <div class="tracking-card-title">
                <strong>${escapeHtml(cameraId)}</strong>
                <span class="tracking-card-badge ${statusClass}">${latency.toFixed(0)}ms latency</span>
              </div>
              <div class="tracking-card-metrics">
                <span>FPS ${fps.toFixed(1)}</span>
                <span>Frames ${metrics.frame_count ?? 0}</span>
                <span>Missing ${metrics.missing_frames ?? 0}</span>
                <span>Blob avg ${blobAvg.toFixed(1)}</span>
              </div>
            </article>
          `;
        })
        .join("");
    }

    function renderRigidHealth(scene) {
      const rigids = scene.rigid_bodies || [];
      if (!rigids.length) {
        elements.trackingRigidList.innerHTML = '<div class="tracking-empty-sm">Waiting for rigid body data</div>';
        return;
      }
      elements.trackingRigidList.innerHTML = rigids
        .map((body) => {
          const validClass = body.valid ? "" : " invalid";
          const badgeClass = body.valid ? "ok" : "warn";
          const rms = Number(body.rms_error ?? 0).toFixed(3);
          const trailLength = (body.trail || []).length;
          return `
            <article class="tracking-rigid-row${validClass}">
              <header>
                <strong>${escapeHtml(body.name)}</strong>
                <span class="tracking-card-badge ${badgeClass}">${body.valid ? "Valid" : "Lost"}</span>
              </header>
              <div class="tracking-card-metrics">
                <span>Observed ${body.observed_markers ?? 0}</span>
                <span>RMS ${rms}</span>
                <span>Trail ${trailLength}</span>
              </div>
            </article>
          `;
        })
        .join("");
    }

    function setActivePage(pageName) {
      activePage = pageName === "tracking" ? "tracking" : "calibration";
      const calibrationActive = activePage === "calibration";
      elements.pageCalibration.classList.toggle("active", calibrationActive);
      elements.pageTracking.classList.toggle("active", !calibrationActive);
      elements.tabCalibration.classList.toggle("active", calibrationActive);
      elements.tabTracking.classList.toggle("active", !calibrationActive);
      if (!calibrationActive) {
        trackingViewer.resize();
      }
    }

    async function loadTracking() {
      const [statusResponse, sceneResponse] = await Promise.all([
        fetch("/api/tracking/status"),
        fetch("/api/tracking/scene"),
      ]);
      if (!statusResponse.ok || !sceneResponse.ok) {
        throw new Error(`tracking_fetch_failed status=${statusResponse.status} scene=${sceneResponse.status}`);
      }
      const status = await statusResponse.json();
      const scene = await sceneResponse.json();
      elements.trackingStart.disabled = !status.start_allowed;
      elements.trackingExtrinsicsPath.textContent = `Extrinsics: ${status.latest_extrinsics_path || "(none)"}`;
      elements.trackingStatus.textContent = JSON.stringify(status, null, 2);
      elements.trackingScene.textContent = JSON.stringify(scene, null, 2);
      const emptyText = status.empty_state || scene.empty_state || "";
      renderTrackingBadge(status);
      renderTrackingFooter(status, scene);
      renderCameraHealth(status);
      renderRigidHealth(scene);
      trackingViewer.update(scene, emptyText);
    }

    async function loadState() {
      const response = await fetch("/api/state");
      if (!response.ok) {
        throw new Error(`state_fetch_failed status=${response.status}`);
      }
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
      sliders.forEach((name) => {
        values[name].textContent = elements[name].value;
      });
      renderCameraRows(state.cameras || []);
      renderCameraSummary(state.cameras || []);
      renderWorkflow(state);
      elements.status.textContent = JSON.stringify(state.last_result, null, 2);
      await loadTracking();
    }

    async function postJson(url, payload) {
      let response;
      try {
        response = await fetch(url, {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify(payload),
        });
      } catch (error) {
        throw new Error(`network_error url=${url} reason=${error instanceof Error ? error.message : String(error)}`);
      }
      let body = {};
      try {
        body = await response.json();
      } catch (error) {
        throw new Error(`invalid_json_response url=${url} status=${response.status}`);
      }
      if (!response.ok) {
        const reason = body && typeof body === "object" && "error" in body ? body.error : `http_${response.status}`;
        throw new Error(`request_failed url=${url} reason=${reason}`);
      }
      return body;
    }

    async function safeLoadState() {
      if (loadStateInFlight) {
        return;
      }
      loadStateInFlight = true;
      try {
        await loadState();
      } catch (error) {
        reportUiError("load_state", error);
      } finally {
        loadStateInFlight = false;
      }
    }

    document.getElementById("applyConfig").addEventListener("click", async () => {
      try {
        await postJson("/api/config", configPayload());
        await safeLoadState();
      } catch (error) {
        reportUiError("apply_config", error);
      }
    });

    document.querySelectorAll("button[data-command]").forEach((button) => {
      button.addEventListener("click", async () => {
        try {
          await postJson("/api/command", {
            command: button.dataset.command,
            camera_ids: selectedCameraIds(),
          });
          await safeLoadState();
        } catch (error) {
          reportUiError(`command_${button.dataset.command || "unknown"}`, error);
        }
      });
    });

    document.getElementById("generateExtrinsics").addEventListener("click", async () => {
      try {
        await postJson("/api/generate_extrinsics", {
          intrinsics_path: elements.intrinsicsPath.value,
          pose_log_path: elements.logPath.value,
          output_path: elements.outputPath.value,
          pair_window_us: Number(elements.pairWindowUs.value),
          min_pairs: Number(elements.minPairs.value),
        });
        await safeLoadState();
      } catch (error) {
        reportUiError("generate_extrinsics", error);
      }
    });

    document.getElementById("applyWandScaleFloor").addEventListener("click", async () => {
      try {
        await postJson("/api/apply_wand_scale_floor", {
          intrinsics_path: elements.intrinsicsPath.value,
          wand_metric_log_path: elements.wandMetricLogPath.value,
          output_path: elements.outputPath.value,
          pair_window_us: Number(elements.pairWindowUs.value),
          max_wand_metric_samples: Number(elements.maxWandMetricSamples.value),
          expected_baseline_m: elements.expectedBaselineM.value === "" ? null : Number(elements.expectedBaselineM.value),
        });
        await safeLoadState();
      } catch (error) {
        reportUiError("apply_wand_scale_floor", error);
      }
    });

    elements.tabCalibration.addEventListener("click", () => setActivePage("calibration"));
    elements.tabTracking.addEventListener("click", () => setActivePage("tracking"));

    elements.trackingStart.addEventListener("click", async () => {
      try {
        await postJson("/api/tracking/start", {
          patterns: ["waist"],
        });
        setActivePage("tracking");
        await safeLoadState();
      } catch (error) {
        reportUiError("tracking_start", error);
      }
    });

    elements.trackingStop.addEventListener("click", async () => {
      try {
        await postJson("/api/tracking/stop", {});
        setActivePage("tracking");
        await safeLoadState();
      } catch (error) {
        reportUiError("tracking_stop", error);
      }
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
            await safeLoadState();
          } catch (error) {
            reportUiError(`slider_${name}`, error);
          } finally {
            sliderUpdateTimers[name] = null;
          }
        }, SLIDER_DEBOUNCE_MS);
      });
    });

    setActivePage(activePage);
    initTrackingViewer();
    safeLoadState();
    setInterval(safeLoadState, 3000);
  </script>
</body>
</html>
"""


class WandGuiState:
    def __init__(
        self,
        session: WandSession,
        receiver: UDPReceiver,
        settings_path: Path | None = None,
        tracking_runtime: TrackingRuntime | None = None,
    ) -> None:
        self.session = session
        self.receiver = receiver
        self.lock = threading.Lock()
        self.settings_path = settings_path or DEFAULT_SETTINGS_PATH
        self.selected_camera_ids: List[str] = []
        self.config = self._load_initial_config()
        self.camera_status: Dict[str, Dict[str, Any]] = {}
        self.last_result: Dict[str, Any] = {"status": "idle"}
        self.capture_log_dir: Path = DEFAULT_CAPTURE_LOG_DIR
        self.pose_capture_log_path: Path = DEFAULT_POSE_LOG_PATH
        self.wand_metric_log_path: Path = DEFAULT_WAND_METRIC_LOG_PATH
        self._capture_logger: FrameLogger | None = None
        self._capture_log_active: bool = False
        self._capture_completed: Dict[str, bool] = {"pose_capture": False, "wand_metric_capture": False}
        self._active_capture_kind: str | None = None
        self._receiver_frame_callback = getattr(receiver, "_frame_callback", None)
        if hasattr(self.receiver, "set_frame_callback"):
            self.receiver.set_frame_callback(self._on_frame_received)
        solve_extrinsics_fn, apply_wand_scale_floor_fn = _load_extrinsics_solvers()
        self._generate_extrinsics_solver = solve_extrinsics_fn
        self._apply_wand_scale_floor_solver = apply_wand_scale_floor_fn
        self.tracking_runtime = tracking_runtime or TrackingRuntime()
        self.latest_extrinsics_path: Path | None = None
        self.latest_extrinsics_quality: Dict[str, Any] | None = None
        self.latest_extrinsics_metric_applied: bool = False
        self._restore_latest_extrinsics(DEFAULT_EXTRINSICS_OUTPUT_PATH)

    def _default_config(self) -> SessionConfig:
        return SessionConfig(exposure_us=12000, gain=8.0, fps=56, duration_s=60.0)

    def _load_initial_config(self) -> SessionConfig:
        config = self._default_config()
        path = self.settings_path
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return config
        except Exception:
            return config
        if not isinstance(payload, dict):
            return config
        return self._build_session_config(payload, base_config=config)

    def _persist_config(self) -> None:
        payload = self._config_payload()
        path = self.settings_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            return

    def _mask_params(self, config: SessionConfig | None = None) -> Dict[str, Any]:
        source = config or self.config
        mask = dict(source.mask_params or {})
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

    def _build_session_config(
        self,
        payload: Dict[str, Any],
        base_config: SessionConfig | None = None,
    ) -> SessionConfig:
        source = base_config or self.config
        mask = self._mask_params(source)
        mask["threshold"] = int(payload.get("mask_threshold", mask["threshold"]))
        mask["seconds"] = float(payload.get("mask_seconds", mask["seconds"]))
        return SessionConfig(
            exposure_us=int(payload.get("exposure_us", source.exposure_us)),
            gain=float(payload.get("gain", source.gain)),
            fps=int(payload.get("fps", source.fps)),
            focus=float(payload.get("focus", source.focus)),
            threshold=int(payload.get("threshold", source.threshold)),
            blob_min_diameter_px=payload.get("blob_min_diameter_px", source.blob_min_diameter_px),
            blob_max_diameter_px=payload.get("blob_max_diameter_px", source.blob_max_diameter_px),
            circularity_min=float(payload.get("circularity_min", source.circularity_min)),
            duration_s=source.duration_s,
            camera_ids=self.selected_camera_ids or None,
            mask_params=mask,
            mask_retry=source.mask_retry,
            output_dir=source.output_dir,
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

    def _workflow_summary(self, cameras: List[Dict[str, Any]]) -> Dict[str, Any]:
        selected = [camera for camera in cameras if bool(camera.get("selected"))]
        active_cameras = selected if selected else cameras

        def diagnostics(camera: Dict[str, Any]) -> Dict[str, Any]:
            value = camera.get("diagnostics")
            return value if isinstance(value, dict) else {}

        def blob_diagnostics(camera: Dict[str, Any]) -> Dict[str, Any]:
            value = diagnostics(camera).get("blob_diagnostics")
            return value if isinstance(value, dict) else {}

        total_count = len(cameras)
        selected_count = len(active_cameras)
        healthy_count = sum(1 for camera in active_cameras if bool(camera.get("healthy")))
        blob_ready_count = sum(
            1
            for camera in active_cameras
            if int(blob_diagnostics(camera).get("last_blob_count", 0) or 0) > 0
        )
        mask_ready_count = sum(
            1
            for camera in active_cameras
            if diagnostics(camera).get("state") in ("READY", "RUNNING")
            and float(diagnostics(camera).get("mask_pixels", 0) or 0) > 0.0
        )
        running_count = sum(
            1 for camera in active_cameras if diagnostics(camera).get("state") == "RUNNING"
        )
        preview_enabled_count = sum(
            1 for camera in active_cameras if bool(diagnostics(camera).get("debug_preview_enabled"))
        )
        preview_active_count = sum(
            1 for camera in active_cameras if bool(diagnostics(camera).get("debug_preview_active"))
        )
        pose_capture_exists = self.pose_capture_log_path.exists() and self.pose_capture_log_path.is_file()
        wand_metric_exists = self.wand_metric_log_path.exists() and self.wand_metric_log_path.is_file()
        pose_capture_complete = bool(self._capture_completed.get("pose_capture")) and pose_capture_exists
        wand_metric_capture_complete = bool(self._capture_completed.get("wand_metric_capture")) and wand_metric_exists
        extrinsics_ready = self.latest_extrinsics_path is not None and self.latest_extrinsics_path.exists() and self.latest_extrinsics_path.is_file()
        metric_floor_applied = self.latest_extrinsics_metric_applied

        active_segment = "blob"
        if metric_floor_applied:
            active_segment = "extrinsics"
        elif running_count > 0:
            active_segment = "wand"
        elif pose_capture_complete:
            active_segment = "extrinsics"
        elif selected_count > 0 and mask_ready_count >= selected_count:
            active_segment = "wand"
        elif blob_ready_count > 0:
            active_segment = "mask"

        return {
            "total_count": total_count,
            "selected_count": selected_count,
            "healthy_count": healthy_count,
            "blob_ready_count": blob_ready_count,
            "mask_ready_count": mask_ready_count,
            "running_count": running_count,
            "preview_enabled_count": preview_enabled_count,
            "preview_active_count": preview_active_count,
            "pose_capture_complete": pose_capture_complete,
            "pose_capture_log_path": str(self.pose_capture_log_path),
            "wand_metric_capture_complete": wand_metric_capture_complete,
            "wand_metric_log_path": str(self.wand_metric_log_path),
            "extrinsics_ready": extrinsics_ready,
            "metric_floor_applied": metric_floor_applied,
            "latest_extrinsics_path": str(self.latest_extrinsics_path) if self.latest_extrinsics_path else None,
            "latest_extrinsics_quality": self.latest_extrinsics_quality,
            "active_segment": active_segment,
        }

    def _tracking_extrinsics_ready(self) -> bool:
        path = self.latest_extrinsics_path
        return bool(path is not None and path.exists() and path.is_file() and self.latest_extrinsics_metric_applied)

    @staticmethod
    def _summarize_extrinsics_quality(camera_rows: Any) -> Dict[str, Any]:
        quality_rows: List[Dict[str, Any]] = []
        if isinstance(camera_rows, list):
            for row in camera_rows:
                if isinstance(row, dict) and isinstance(row.get("quality"), dict):
                    quality_rows.append(row["quality"])
        return {
            "pair_count_total": int(sum(float(item.get("pair_count", 0) or 0) for item in quality_rows)),
            "median_reproj_error_px_max": float(
                max((float(item.get("median_reproj_error_px", 0.0) or 0.0) for item in quality_rows), default=0.0)
            ),
            "inlier_ratio_min": float(
                min((float(item.get("inlier_ratio", 1.0) or 1.0) for item in quality_rows), default=1.0)
            ),
        }

    @staticmethod
    def _summarize_validation(validation: Any) -> Dict[str, Any]:
        if not isinstance(validation, dict):
            return {}
        summary: Dict[str, Any] = {}
        for key in (
            "median_reproj_error_px",
            "p90_reproj_error_px",
            "positive_depth_ratio",
            "triangulation_angle_deg_p50",
            "triangulation_angle_deg_p90",
            "floor_residual_mm",
            "world_up_consistency",
            "baseline_range_units",
            "baseline_range_m",
        ):
            if key in validation:
                summary[key] = validation.get(key)
        return summary

    def _restore_latest_extrinsics(self, path: Path) -> None:
        if not path.exists() or not path.is_file():
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        self.latest_extrinsics_path = path
        session_meta = payload.get("session_meta", {})
        if isinstance(session_meta, dict):
            scale_source = str(session_meta.get("scale_source", "none") or "none")
            floor_source = str(session_meta.get("floor_source", "none") or "none")
            self.latest_extrinsics_metric_applied = scale_source != "none" and floor_source != "none"
            if self.latest_extrinsics_metric_applied:
                self.latest_extrinsics_quality = self._summarize_validation(session_meta.get("wand_metric_validation"))
            else:
                self.latest_extrinsics_quality = self._summarize_validation(session_meta.get("pose_validation"))
        else:
            self.latest_extrinsics_metric_applied = False
            self.latest_extrinsics_quality = self._summarize_extrinsics_quality(payload.get("cameras"))

    @staticmethod
    def _resolve_project_path(raw_path: str, fallback: Path) -> Path:
        candidate = raw_path.strip() if raw_path.strip() else str(fallback)
        resolved = Path(candidate)
        if not resolved.is_absolute():
            resolved = PROJECT_ROOT / resolved
        return resolved

    def _resolve_tracking_calibration_path(self, raw_path: str) -> Path:
        candidate = raw_path.strip()
        if not candidate and self.latest_extrinsics_path is not None:
            candidate = str(self.latest_extrinsics_path)
        if not candidate:
            raise ValueError("generate extrinsics first")

        resolved = Path(candidate)
        if not resolved.is_absolute():
            resolved = PROJECT_ROOT / resolved
        if resolved.is_dir():
            return resolved
        if resolved.is_file() and resolved.name.startswith("calibration_extrinsics_v1"):
            return resolved.parent
        raise ValueError("tracking calibration_path must be a calibration directory or extrinsics file")

    def get_tracking_status(self) -> Dict[str, Any]:
        status = self.tracking_runtime.status()
        start_allowed = self._tracking_extrinsics_ready()
        return {
            **status,
            "start_allowed": start_allowed,
            "empty_state": None if start_allowed else "Generate extrinsics first",
            "latest_extrinsics_path": str(self.latest_extrinsics_path) if self.latest_extrinsics_path else None,
            "latest_extrinsics_quality": self.latest_extrinsics_quality,
        }

    def get_tracking_scene(self) -> Dict[str, Any]:
        start_allowed = self._tracking_extrinsics_ready()
        if not start_allowed:
            return {
                "tracking": {
                    "running": False,
                    "frames_processed": 0,
                    "poses_estimated": 0,
                },
                "cameras": [],
                "rigid_bodies": [],
                "raw_points": [],
                "timestamp_us": int(time.time() * 1_000_000),
                "empty_state": "Generate extrinsics first",
            }
        return self.tracking_runtime.scene_snapshot()

    def start_tracking(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        resolved = self._resolve_tracking_calibration_path(str(payload.get("calibration_path") or ""))
        patterns = payload.get("patterns", ["waist"])
        if not isinstance(patterns, list):
            patterns = ["waist"]
        status = self.tracking_runtime.start(str(resolved), [str(item) for item in patterns])
        response = {"ok": True, **status, "running": True}
        self.last_result = {"tracking_start": response}
        return response

    def stop_tracking(self) -> Dict[str, Any]:
        stop_result = self.tracking_runtime.stop()
        summary = stop_result.get("summary", stop_result) if isinstance(stop_result, dict) else {}
        status = self.tracking_runtime.status()
        response = {"ok": True, "summary": summary, **status, "running": False}
        self.last_result = {"tracking_stop": response}
        return response

    def get_state(self) -> Dict[str, Any]:
        cameras = self.refresh_targets()
        return {
            "config": self._config_payload(),
            "cameras": cameras,
            "workflow": self._workflow_summary(cameras),
            "last_result": self.last_result,
            "receiver": self.receiver.stats,
            "tracking": self.get_tracking_status(),
        }

    def _sync_selected_camera_ids(self, payload: Dict[str, Any]) -> None:
        if "camera_ids" not in payload:
            return
        raw_camera_ids = payload.get("camera_ids")
        if raw_camera_ids is None:
            self.selected_camera_ids = []
            return
        if not isinstance(raw_camera_ids, list):
            raise ValueError("camera_ids must be a list")
        self.selected_camera_ids = [
            str(camera_id).strip()
            for camera_id in raw_camera_ids
            if str(camera_id).strip()
        ]

    def apply_config(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._sync_selected_camera_ids(payload)
        self.config = self._build_session_config(payload)
        self._persist_config()

        targets = self.session.discover_targets(self.selected_camera_ids or None)
        result = self._apply_capture_settings(targets)
        self._update_camera_status(result)
        self.last_result = result
        return result

    def run_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command = str(payload.get("command", "")).strip()
        self._sync_selected_camera_ids(payload)

        if command == "refresh":
            result = {"cameras": self.refresh_targets()}
            self.last_result = result
            return result

        targets = self.session.discover_targets(self.selected_camera_ids or None)
        command_handlers = {
            "ping": lambda: self.session._broadcast(targets, "ping"),
            "mask_start": lambda: self.session._broadcast(targets, "mask_start", **self._mask_params()),
            "mask_stop": lambda: self.session._broadcast(targets, "mask_stop"),
        }
        handler = command_handlers.get(command)
        if command in ("start", "start_pose_capture", "start_wand_metric_capture"):
            capture_kind = "pose_capture"
            if command == "start_wand_metric_capture":
                capture_kind = "wand_metric_capture"
            if command == "start":
                capture_kind = "pose_capture"
            log_path = self._start_capture_log(capture_kind)
            result = self.session._broadcast(targets, "start", mode=capture_kind)
            payload_out: Dict[str, Any] = {command: result, "capture_log": {"path": str(log_path)}}
            if not self._all_acked(result):
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"].update(stop_meta)
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if command in ("stop", "stop_pose_capture", "stop_wand_metric_capture"):
            result = self.session._broadcast(targets, "stop")
            payload_out = {command: result}
            stop_meta = self._stop_capture_log()
            if stop_meta is not None:
                payload_out["capture_log"] = stop_meta
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if handler is None:
            raise ValueError(f"Unsupported command: {command}")
        result = handler()

        self._update_camera_status({command: result})
        self.last_result = {command: result}
        return self.last_result

    def generate_extrinsics(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        intrinsics_raw = str(payload.get("intrinsics_path", "calibration")).strip()
        log_raw = str(payload.get("pose_log_path", payload.get("log_path", str(DEFAULT_POSE_LOG_PATH)))).strip()
        output_raw = str(payload.get("output_path", str(DEFAULT_EXTRINSICS_OUTPUT_PATH))).strip()
        intrinsics_path = self._resolve_project_path(intrinsics_raw, Path("calibration"))
        resolved_log_path = self._resolve_project_path(log_raw, DEFAULT_POSE_LOG_PATH)
        resolved_output = self._resolve_project_path(output_raw, DEFAULT_EXTRINSICS_OUTPUT_PATH)
        pair_window_us = int(payload.get("pair_window_us", 8000))
        min_pairs = int(payload.get("min_pairs", 8))
        if pair_window_us < 1:
            raise ValueError("pair_window_us must be >= 1")
        if min_pairs < 1:
            raise ValueError("min_pairs must be >= 1")

        if not resolved_log_path.exists():
            raise ValueError(f"log_path does not exist: {resolved_log_path}")
        if not resolved_log_path.is_file():
            raise ValueError(f"log_path is not a file: {resolved_log_path}")

        try:
            raw_result = self._generate_extrinsics_solver(
                intrinsics_path=str(intrinsics_path),
                pose_log_path=str(resolved_log_path),
                output_path=str(resolved_output),
                pair_window_us=pair_window_us,
                min_pairs=min_pairs,
            )
        except FileNotFoundError as exc:
            missing_path = Path(getattr(exc, "filename", "") or str(resolved_log_path))
            raise ValueError(f"log_path does not exist: {missing_path}") from exc
        if not isinstance(raw_result, dict):
            raise ValueError("extrinsics solver returned invalid response")

        camera_rows = raw_result.get("cameras")
        camera_count = len(camera_rows) if isinstance(camera_rows, list) else 0
        session_meta = raw_result.get("session_meta", {})
        if not isinstance(session_meta, dict):
            session_meta = {}
        quality_summary = self._summarize_validation(session_meta.get("pose_validation"))

        summary = {
            "ok": True,
            "reference_camera_id": raw_result.get("reference_camera_id"),
            "camera_count": camera_count,
            "output_path": str(resolved_output),
            "quality": quality_summary,
            "metric_floor_applied": False,
            "wand_validation_applied": False,
        }
        self.latest_extrinsics_path = resolved_output
        self.latest_extrinsics_quality = quality_summary
        self.latest_extrinsics_metric_applied = False
        self.last_result = {"generate_extrinsics": summary}
        return self.last_result

    def apply_wand_scale_floor(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        intrinsics_raw = str(payload.get("intrinsics_path", "calibration")).strip()
        wand_metric_raw = str(payload.get("wand_metric_log_path", str(DEFAULT_WAND_METRIC_LOG_PATH))).strip()
        output_raw = str(payload.get("output_path", str(DEFAULT_EXTRINSICS_OUTPUT_PATH))).strip()
        intrinsics_path = self._resolve_project_path(intrinsics_raw, Path("calibration"))
        resolved_wand_metric = self._resolve_project_path(wand_metric_raw, DEFAULT_WAND_METRIC_LOG_PATH)
        resolved_output = self._resolve_project_path(output_raw, DEFAULT_EXTRINSICS_OUTPUT_PATH)
        pair_window_us = int(payload.get("pair_window_us", 8000))
        max_wand_metric_samples = int(payload.get("max_wand_metric_samples", 16))
        expected_baseline_raw = payload.get("expected_baseline_m")
        if pair_window_us < 1:
            raise ValueError("pair_window_us must be >= 1")
        if max_wand_metric_samples < 1:
            raise ValueError("max_wand_metric_samples must be >= 1")
        expected_baseline_m = None if expected_baseline_raw in (None, "") else float(expected_baseline_raw)
        if expected_baseline_m is not None and expected_baseline_m <= 0.0:
            raise ValueError("expected_baseline_m must be > 0")
        if not resolved_output.exists() or not resolved_output.is_file():
            raise ValueError(f"output_path does not exist: {resolved_output}")
        if not resolved_wand_metric.exists() or not resolved_wand_metric.is_file():
            raise ValueError(f"wand_metric_log_path does not exist: {resolved_wand_metric}")

        try:
            raw_result = self._apply_wand_scale_floor_solver(
                intrinsics_path=str(intrinsics_path),
                extrinsics_path=str(resolved_output),
                wand_metric_log_path=str(resolved_wand_metric),
                output_path=str(resolved_output),
                pair_window_us=pair_window_us,
                max_wand_metric_samples=max_wand_metric_samples,
                expected_baseline_m=expected_baseline_m,
            )
        except FileNotFoundError as exc:
            missing_path = Path(getattr(exc, "filename", "") or str(resolved_wand_metric))
            raise ValueError(f"wand_metric_log_path does not exist: {missing_path}") from exc
        if not isinstance(raw_result, dict):
            raise ValueError("wand scale/floor solver returned invalid response")

        camera_rows = raw_result.get("cameras")
        camera_count = len(camera_rows) if isinstance(camera_rows, list) else 0
        session_meta = raw_result.get("session_meta", {})
        if not isinstance(session_meta, dict):
            session_meta = {}
        quality_summary = self._summarize_validation(session_meta.get("wand_metric_validation"))
        summary = {
            "ok": True,
            "reference_camera_id": raw_result.get("reference_camera_id"),
            "camera_count": camera_count,
            "output_path": str(resolved_output),
            "quality": quality_summary,
            "wand_metric_validation": self._summarize_validation(session_meta.get("wand_metric_validation")),
            "scale_source": session_meta.get("scale_source"),
            "floor_source": session_meta.get("floor_source"),
            "wand_metric_frames": int(session_meta.get("wand_metric_frames", 0) or 0),
        }
        self.latest_extrinsics_path = resolved_output
        self.latest_extrinsics_quality = quality_summary
        self.latest_extrinsics_metric_applied = bool(
            summary.get("scale_source") not in (None, "none") and summary.get("floor_source") not in (None, "none")
        )
        self.last_result = {"apply_wand_scale_floor": summary}
        return self.last_result

    def _update_camera_status(self, result: Dict[str, Dict[str, Dict[str, Any]]]) -> None:
        with self.lock:
            for responses in result.values():
                for camera_id, response in responses.items():
                    entry = self.camera_status.setdefault(camera_id, {"camera_id": camera_id, "ip": "unknown"})
                    entry["last_ack"] = bool(response.get("ack"))
                    entry["last_error"] = response.get("error") or response.get("error_message")

    def _on_frame_received(self, frame: Any) -> None:
        previous = self._receiver_frame_callback
        if callable(previous):
            previous(frame)
        logger: FrameLogger | None = None
        with self.lock:
            if self._capture_log_active:
                logger = self._capture_logger
        if logger is None:
            return
        frame_dict = frame.to_dict() if hasattr(frame, "to_dict") else dict(frame)
        try:
            logger.log_frame(frame_dict)
        except Exception:
            return

    def _start_capture_log(self, capture_kind: str) -> Path:
        with self.lock:
            if self._capture_logger is not None and self._capture_log_active:
                default_path = self.pose_capture_log_path if capture_kind == "pose_capture" else self.wand_metric_log_path
                return Path(self._capture_logger.current_log_file or str(default_path))
            self.capture_log_dir.mkdir(parents=True, exist_ok=True)
            target_path = self.pose_capture_log_path if capture_kind == "pose_capture" else self.wand_metric_log_path
            if target_path.exists():
                target_path.unlink(missing_ok=True)
            logger = FrameLogger(log_dir=str(self.capture_log_dir))
            log_file = logger.start_recording(session_name=target_path.stem)
            self._capture_logger = logger
            self._capture_log_active = True
            self._active_capture_kind = capture_kind
            self._capture_completed[capture_kind] = False
            if capture_kind == "pose_capture":
                self.pose_capture_log_path = Path(log_file)
                return self.pose_capture_log_path
            self.wand_metric_log_path = Path(log_file)
            return self.wand_metric_log_path

    def _stop_capture_log(self) -> Dict[str, Any] | None:
        with self.lock:
            logger = self._capture_logger
            active = self._capture_log_active
        if logger is None or not active:
            return None
        metadata = logger.stop_recording()
        with self.lock:
            self._capture_log_active = False
            active_kind = self._active_capture_kind
            self._active_capture_kind = None
            if isinstance(metadata.get("log_file"), str):
                path = Path(str(metadata["log_file"]))
                if active_kind == "pose_capture":
                    self.pose_capture_log_path = path
                elif active_kind == "wand_metric_capture":
                    self.wand_metric_log_path = path
            if active_kind == "pose_capture":
                self._capture_completed["pose_capture"] = self.pose_capture_log_path.exists() and self.pose_capture_log_path.is_file()
            elif active_kind == "wand_metric_capture":
                self._capture_completed["wand_metric_capture"] = self.wand_metric_log_path.exists() and self.wand_metric_log_path.is_file()
        return metadata

    @staticmethod
    def _all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
        return all(bool(resp.get("ack")) for resp in responses.values())


class WandGuiHandler(BaseHTTPRequestHandler):
    state: WandGuiState

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path.startswith("/static/"):
            self._serve_static(path[len("/static/"):])
            return
        if path in ("/", "/index.html"):
            self._send_html(HTML_PAGE)
            return
        if path == "/api/state":
            self._send_json(self.state.get_state())
            return
        if path == "/api/tracking/status":
            self._send_json(self.state.get_tracking_status())
            return
        if path == "/api/tracking/scene":
            self._send_json(self.state.get_tracking_scene())
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            payload = self._read_json()
            if self.path == "/api/config":
                self._send_json(self.state.apply_config(payload))
                return
            if self.path == "/api/command":
                self._debug_log(
                    f"POST /api/command command={payload.get('command')} camera_ids={payload.get('camera_ids')}"
                )
                self._send_json(self.state.run_command(payload))
                return
            if self.path == "/api/generate_extrinsics":
                self._debug_log("POST /api/generate_extrinsics")
                self._send_json(self.state.generate_extrinsics(payload))
                return
            if self.path == "/api/apply_wand_scale_floor":
                self._debug_log("POST /api/apply_wand_scale_floor")
                self._send_json(self.state.apply_wand_scale_floor(payload))
                return
            if self.path == "/api/tracking/start":
                self._debug_log("POST /api/tracking/start")
                self._send_json(self.state.start_tracking(payload))
                return
            if self.path == "/api/tracking/stop":
                self._debug_log("POST /api/tracking/stop")
                self._send_json(self.state.stop_tracking())
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._debug_log(f"BAD_REQUEST path={self.path} error={exc}")
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except Exception as exc:  # noqa: BLE001
            self._debug_log(f"INTERNAL_ERROR path={self.path} error={exc}")
            self._send_json({"error": f"internal_error: {exc}"}, status=HTTPStatus.INTERNAL_SERVER_ERROR)

    def log_message(self, format: str, *args: Any) -> None:
        return

    @staticmethod
    def _debug_log(message: str) -> None:
        print(f"[wand_gui] {message}", flush=True)

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

    def _serve_static(self, rel_path: str) -> None:
        target = _resolve_static_asset(rel_path)
        if target is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            data = target.read_bytes()
        except Exception:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", self._guess_content_type(target.suffix))
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    @staticmethod
    def _guess_content_type(suffix: str) -> str:
        mapping = {
            ".js": "application/javascript",
            ".mjs": "application/javascript",
            ".css": "text/css",
            ".json": "application/json",
            ".html": "text/html",
        }
        return mapping.get(suffix.lower(), "application/octet-stream")


def _resolve_static_asset(rel_path: str) -> Optional[Path]:
    candidate = Path(rel_path.lstrip("/"))
    for static_dir in STATIC_DIR_CANDIDATES:
        try:
            static_root = static_dir.resolve()
            target = (static_root / candidate).resolve()
            target.relative_to(static_root)
        except Exception:
            continue
        if target.exists() and target.is_file():
            return target
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal web GUI for wand capture control")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP bind port")
    parser.add_argument("--udp-port", type=int, default=5000, help="Passive discovery UDP port")
    parser.add_argument("--inventory", default=None, help="Optional hosts.ini path")
    return parser.parse_args()


def _load_extrinsics_solvers():
    script_path = MODULE_SRC_ROOT / "camera-calibration" / "calibrate_extrinsics.py"
    spec = importlib.util.spec_from_file_location("calibrate_extrinsics", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load solver module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["calibrate_extrinsics"] = module
    spec.loader.exec_module(module)
    solve_extrinsics = getattr(module, "solve_extrinsics", None)
    apply_wand_scale_floor = getattr(module, "apply_wand_scale_floor", None)
    if not callable(solve_extrinsics):
        raise RuntimeError("solve_extrinsics is missing in calibrate_extrinsics.py")
    if not callable(apply_wand_scale_floor):
        raise RuntimeError("apply_wand_scale_floor is missing in calibrate_extrinsics.py")
    return solve_extrinsics, apply_wand_scale_floor


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
