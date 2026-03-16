#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
import threading
import importlib.util
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from collections import Counter

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_SRC_ROOT = Path(__file__).resolve().parents[1]

if __package__ in (None, ""):
    if str(MODULE_SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(MODULE_SRC_ROOT))
    from host.receiver import UDPReceiver
    from host.logger import FrameLogger
    from host.tracking_runtime import TrackingRuntime
    from host.wand_session import CalibrationSession, CalibrationSessionConfig
    from host.intrinsics_capture import IntrinsicsCapture, IntrinsicsConfig
    from host.extrinsics_methods import ExtrinsicsMethodRegistry, build_default_extrinsics_registry
else:
    from .receiver import UDPReceiver
    from .logger import FrameLogger
    from .tracking_runtime import TrackingRuntime
    from .wand_session import CalibrationSession, CalibrationSessionConfig
    from .intrinsics_capture import IntrinsicsCapture, IntrinsicsConfig
    from .extrinsics_methods import ExtrinsicsMethodRegistry, build_default_extrinsics_registry


DEFAULT_SETTINGS_PATH = Path("logs") / "loutrack_gui_settings.json"
OLD_SETTINGS_PATH = Path("logs") / "wand_gui_settings.json"
DEFAULT_POSE_LOG_PATH = Path("logs") / "extrinsics_pose_capture.jsonl"
DEFAULT_WAND_METRIC_LOG_PATH = Path("logs") / "extrinsics_wand_metric.jsonl"
DEFAULT_EXTRINSICS_OUTPUT_PATH = Path("calibration") / "extrinsics_pose_v2.json"
DEFAULT_CAPTURE_LOG_DIR = Path("logs")
DEFAULT_WAND_METRIC_DURATION_S = 3.0
STATIC_DIR_CANDIDATES = (
    PROJECT_ROOT / "static",
    MODULE_SRC_ROOT / "static",
)


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Loutrack GUI</title>
  <style>
    :root {
      --bg: #0F172A;
      --sidebar-bg: #0B1120;
      --panel: #1E293B;
      --border: rgba(148,163,184,0.12);
      --ink: #E2E8F0;
      --ink-muted: #64748B;
      --accent: #22C55E;
      --teal: #1f6b70;
      --warm: #F97316;
      --danger: #EF4444;
      --sidebar-w: 220px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body { width: 100%; height: 100%; }
    body {
      background: var(--bg);
      color: var(--ink);
      font-family: "Inter", "Avenir Next", "Helvetica Neue", sans-serif;
      font-size: 13px;
      overflow: hidden;
      width: 100vw;
      height: 100vh;
      min-width: 0;
      min-height: 0;
    }
    .app-shell {
      display: flex;
      flex-direction: row;
      width: 100vw;
      height: 100vh;
      overflow: hidden;
    }

    /* ── Sidebar ─────────────────────────────────────── */
    .sidebar {
      width: var(--sidebar-w);
      min-width: var(--sidebar-w);
      max-width: var(--sidebar-w);
      background: var(--sidebar-bg);
      border-right: 1px solid var(--border);
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }
    .sidebar-logo {
      padding: 16px 14px 12px;
      font-size: 15px;
      font-weight: 800;
      color: var(--ink);
      letter-spacing: -0.02em;
      border-bottom: 1px solid var(--border);
    }
    .sidebar-logo span { color: var(--accent); }
    .sidebar-nav {
      flex: 1;
      overflow-y: auto;
      padding: 8px 8px;
      display: flex;
      flex-direction: column;
      gap: 2px;
    }
    .sidebar-divider {
      height: 1px;
      background: var(--border);
      margin: 6px 4px;
    }
    .nav-item {
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 7px 10px;
      border-radius: 8px;
      background: none;
      border: none;
      color: var(--ink-muted);
      font-size: 13px;
      font-weight: 500;
      cursor: pointer;
      text-align: left;
      width: 100%;
      transition: background 0.12s, color 0.12s;
    }
    .nav-item:hover { background: rgba(148,163,184,0.08); color: var(--ink); }
    .nav-item.active { background: rgba(34,197,94,0.12); color: var(--accent); font-weight: 700; }
    .nav-item.done { color: var(--accent); }
    .nav-item.disabled { opacity: 0.4; cursor: not-allowed; }
    .step-icon { font-size: 12px; min-width: 14px; text-align: center; }
    .nav-badge {
      margin-left: auto;
      font-size: 10px;
      font-weight: 700;
      padding: 2px 6px;
      border-radius: 999px;
      background: rgba(148,163,184,0.15);
      color: var(--ink-muted);
    }
    .nav-item.active .nav-badge { background: rgba(34,197,94,0.2); color: var(--accent); }
    @keyframes spin { to { transform: rotate(360deg); } }
    .nav-item.inprogress .step-icon { display: inline-block; animation: spin 1.2s linear infinite; }
    .sidebar-bottom {
      padding: 10px 8px;
      border-top: 1px solid var(--border);
      display: flex;
      gap: 6px;
    }

    /* ── Main area ────────────────────────────────────── */
    .main-area {
      flex: 1;
      position: relative;
      overflow: hidden;
    }

    /* ── Views ────────────────────────────────────────── */
    .view {
      position: absolute;
      inset: 0;
      display: none;
      flex-direction: column;
      padding: 16px;
      overflow: hidden;
    }
    .view.active { display: flex; }
    .view-title {
      font-size: 16px;
      font-weight: 700;
      color: var(--ink);
      margin-bottom: 12px;
    }
    .view-sub {
      font-size: 11px;
      color: var(--ink-muted);
      text-transform: uppercase;
      letter-spacing: 0.1em;
      margin-bottom: 4px;
    }

    /* ── Split layout ─────────────────────────────────── */
    .split {
      display: flex;
      flex-direction: row;
      gap: 12px;
      flex: 1;
      min-height: 0;
    }
    .split-left { display: flex; flex-direction: column; min-width: 0; }
    .split-right { display: flex; flex-direction: column; min-width: 0; }
    .calib-split-left { flex: 1.4; display: flex; flex-direction: column; min-height: 0; }
    .calib-split-right { width: 340px; min-width: 280px; display: flex; flex-direction: column; min-height: 0; }

    /* ── Panels / Cards ───────────────────────────────── */
    .panel {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
    }
    .panel + .panel { margin-top: 10px; }
    .panel-title {
      font-size: 12px;
      font-weight: 700;
      color: var(--ink);
      margin-bottom: 10px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
    }

    /* ── Buttons ──────────────────────────────────────── */
    button {
      border: none;
      border-radius: 8px;
      padding: 7px 14px;
      font: inherit;
      font-size: 12px;
      font-weight: 600;
      cursor: pointer;
      background: var(--warm);
      color: #fff;
      transition: opacity 0.12s;
    }
    button:hover { opacity: 0.85; }
    button:disabled { opacity: 0.4; cursor: not-allowed; }
    button.btn-green { background: var(--accent); color: #000; }
    button.btn-teal { background: var(--teal); color: #fff; }
    button.btn-ghost {
      background: rgba(148,163,184,0.12);
      color: var(--ink);
    }
    button.btn-danger { background: var(--danger); color: #fff; }
    button.btn-sm { padding: 5px 10px; font-size: 11px; }
    .btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 10px; }

    /* ── Tables ───────────────────────────────────────── */
    .tbl-wrap { overflow: auto; flex: 1; }
    table { width: 100%; border-collapse: collapse; font-size: 12px; }
    th {
      text-align: left;
      padding: 6px 8px;
      color: var(--ink-muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.07em;
      border-bottom: 1px solid var(--border);
      position: sticky; top: 0;
      background: var(--panel);
      z-index: 1;
    }
    td {
      padding: 6px 8px;
      border-bottom: 1px solid var(--border);
      vertical-align: middle;
    }
    tr:last-child td { border-bottom: none; }

    /* ── Badge ────────────────────────────────────────── */
    .badge {
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 2px 8px;
      font-size: 11px;
      font-weight: 700;
      background: rgba(148,163,184,0.15);
      color: var(--ink-muted);
    }
    .badge.ok { background: rgba(34,197,94,0.15); color: var(--accent); }
    .badge.warn { background: rgba(249,115,22,0.15); color: var(--warm); }
    .badge.muted { background: rgba(148,163,184,0.1); color: var(--ink-muted); }
    .badge.live { background: var(--accent); color: #000; }

    /* ── Sliders ──────────────────────────────────────── */
    .slider-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px 16px;
    }
    .slider-label {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .slider-label span {
      font-size: 11px;
      font-weight: 600;
      color: var(--ink-muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .slider-value { color: var(--ink); font-weight: 700; }
    input[type="range"] { width: 100%; accent-color: var(--accent); }
    input[type="text"], input[type="number"] {
      width: 100%;
      background: rgba(15,23,42,0.6);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 6px 8px;
      color: var(--ink);
      font: inherit;
      font-size: 12px;
    }
    label {
      display: flex;
      flex-direction: column;
      gap: 4px;
      font-size: 11px;
      font-weight: 600;
      color: var(--ink-muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }

    /* ── Stat cards ───────────────────────────────────── */
    .stat-row {
      display: flex;
      gap: 8px;
      margin-bottom: 12px;
    }
    .stat-card {
      flex: 1;
      background: rgba(15,23,42,0.5);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 8px 10px;
    }
    .stat-label { font-size: 10px; color: var(--ink-muted); text-transform: uppercase; letter-spacing: 0.07em; }
    .stat-value { font-size: 20px; font-weight: 700; color: var(--ink); line-height: 1.2; }

    /* ── Console ──────────────────────────────────────── */
    .console {
      background: #0d1117;
      color: #c9d1d9;
      padding: 10px 12px;
      border-radius: 8px;
      overflow: auto;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font-size: 11px;
      font-family: "JetBrains Mono", "Fira Mono", monospace;
      flex: 1;
    }

    /* ── Mini bar ─────────────────────────────────────── */
    .mini-bar-wrap { display: flex; align-items: center; gap: 6px; }
    .mini-bar { width: 60px; height: 6px; background: rgba(148,163,184,0.15); border-radius: 3px; overflow: hidden; }
    .mini-bar-fill { height: 100%; background: var(--accent); border-radius: 3px; }

    /* ── Quality badges ───────────────────────────────── */
    .quality-badge {
      display: inline-flex;
      align-items: center;
      border-radius: 6px;
      padding: 3px 8px;
      font-size: 11px;
      font-weight: 600;
      background: rgba(148,163,184,0.1);
      color: var(--ink-muted);
      margin: 2px;
    }
    .quality-badge.ok { background: rgba(34,197,94,0.15); color: var(--accent); }
    .quality-badge.warn { background: rgba(249,115,22,0.15); color: var(--warm); }
    .quality-badge.error { background: rgba(239,68,68,0.15); color: var(--danger); }

    /* ── Tracking viewer ──────────────────────────────── */
    .tracking-viewer-inner {
      position: relative;
      border-radius: 10px;
      border: 1px solid var(--border);
      overflow: hidden;
      background: #060d19;
      flex: 1;
    }
    .tracking-viewer-inner canvas { width: 100%; height: 100%; display: block; }
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
      background: rgba(6,13,25,0.85);
      pointer-events: none;
      font-size: 14px;
    }

    /* ── Tracking cards ───────────────────────────────── */
    .tracking-camera-card {
      background: rgba(15,23,42,0.6);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 8px 10px;
      margin-bottom: 6px;
    }
    .tracking-card-title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 12px;
      font-weight: 700;
      margin-bottom: 4px;
    }
    .tracking-card-badge {
      font-size: 10px;
      font-weight: 700;
      padding: 2px 7px;
      border-radius: 999px;
      background: rgba(148,163,184,0.15);
      color: var(--ink-muted);
    }
    .tracking-card-badge.ok { background: rgba(34,197,94,0.15); color: var(--accent); }
    .tracking-card-badge.warn { background: rgba(249,115,22,0.15); color: var(--warm); }
    .tracking-card-metrics { display: flex; gap: 10px; font-size: 11px; color: var(--ink-muted); flex-wrap: wrap; }
    .tracking-rigid-row {
      background: rgba(15,23,42,0.6);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 8px 10px;
      margin-bottom: 6px;
    }
    .tracking-rigid-row.invalid { border-color: rgba(239,68,68,0.3); }
    .tracking-rigid-row header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 12px;
      font-weight: 700;
      margin-bottom: 4px;
    }
    .tracking-state-badge {
      padding: 3px 10px;
      border-radius: 999px;
      font-size: 11px;
      font-weight: 700;
      background: rgba(148,163,184,0.12);
      color: var(--ink-muted);
    }
    .tracking-state-badge[data-state="running"] { background: var(--accent); color: #000; }
    .tracking-state-badge[data-state="error"] { background: var(--danger); color: #fff; }
    .tracking-empty-sm {
      padding: 8px 10px;
      border-radius: 8px;
      border: 1px dashed var(--border);
      text-align: center;
      color: var(--ink-muted);
      font-size: 12px;
    }

    /* ── Intrinsics ───────────────────────────────────── */
    .intrinsics-viewer {
      aspect-ratio: 16/9;
      background: #0d1117;
      border-radius: 8px;
      border: 1px solid var(--border);
      overflow: hidden;
      display: flex;
      align-items: center;
      justify-content: center;
      margin-bottom: 8px;
    }
    .intrinsics-viewer img { width: 100%; height: 100%; object-fit: contain; }
    .int-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 3px;
      margin-top: 6px;
    }
    .int-cell {
      aspect-ratio: 1;
      border-radius: 6px;
      background: rgba(148,163,184,0.06);
      border: 1px solid var(--border);
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 50px;
      font-weight: 700;
      color: var(--ink-muted);
    }
    .int-cell.covered { background: rgba(34,197,94,0.18); color: var(--accent); }
    .int-stat { font-size: 12px; color: var(--ink-muted); margin-bottom: 4px; }
    .int-stat strong { color: var(--ink); }
    .int-camera-list { display: flex; flex-wrap: wrap; gap: 5px; margin-top: 8px; }
    .int-slider-title { display: inline-flex; align-items: baseline; gap: 4px; white-space: nowrap; }

    /* ── Floor / countdown ────────────────────────────── */
    .floor-card {
      width: min(100%, 640px);
      max-width: 100%;
      margin: 0;
      text-align: center;
    }
    .floor-countdown {
      font-size: 48px;
      font-weight: 800;
      color: var(--accent);
      margin: 16px 0;
      min-height: 60px;
    }

    /* ── Sidebar section headers ─────────────────────── */
    .nav-section-header {
      font-size: 10px;
      font-weight: 700;
      color: var(--ink-muted);
      text-transform: uppercase;
      letter-spacing: 0.1em;
      padding: 8px 10px 3px;
      margin-top: 2px;
    }

    /* ── MJPEG preview grid ───────────────────────────── */
    .mjpeg-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 6px;
      margin-bottom: 10px;
      flex-shrink: 0;
    }
    .mjpeg-cell {
      background: #0d1117;
      border: 1px solid var(--border);
      border-radius: 8px;
      overflow: hidden;
    }
    .mjpeg-cell img {
      width: 100%;
      aspect-ratio: 16/9;
      object-fit: contain;
      display: block;
    }
    .mjpeg-label {
      font-size: 10px;
      color: var(--ink-muted);
      text-align: center;
      padding: 2px 4px;
      background: rgba(15,23,42,0.7);
    }

    /* ── Op-status badge ──────────────────────────────── */
    .op-status {
      display: inline-block;
      font-size: 11px;
      font-weight: 700;
      padding: 3px 9px;
      border-radius: 6px;
      margin-left: 8px;
      vertical-align: middle;
      background: rgba(148,163,184,0.12);
      color: var(--ink-muted);
      transition: background 0.15s, color 0.15s;
    }
    .op-status.waiting { background: rgba(249,115,22,0.15); color: var(--warm); }
    .op-status.done    { background: rgba(34,197,94,0.15);  color: var(--accent); }
    .op-status.error   { background: rgba(239,68,68,0.15);  color: var(--danger); }

    /* ── Hidden compat elements ───────────────────────── */
    .hidden-compat { display: none !important; }

    /* Step pills (hidden compat, kept for JS references) */
    #stepBlob, #stepMask, #stepWand, #stepFloor, #stepExtrinsics { display: none; }
    #blobSummary, #maskSummary, #wandSummary, #floorSummary, #extrinsicsSummary { display: none; }
    #cameraSummary, #workflowRail, #selectionMetric, #selectionSummary,
    #blobMetric, #blobMetricSummary, #maskMetric, #maskMetricSummary,
    #previewMetric, #previewMetricSummary { display: none; }
    #tabIntrinsics, #tabCalibration, #tabTracking { display: none; }
    #pageCalibration { display: none; }
    #trackingStatus, #trackingScene { display: none; }
  </style>
</head>
<body>

  <!-- Hidden compat elements (JS reads/writes to these) -->
  <div style="display:none">
    <div id="workflowRail"></div>
    <div id="cameraSummary"></div>
    <strong id="selectionMetric"></strong><p id="selectionSummary"></p>
    <strong id="blobMetric"></strong><p id="blobMetricSummary"></p>
    <strong id="maskMetric"></strong><p id="maskMetricSummary"></p>
    <strong id="previewMetric"></strong><p id="previewMetricSummary"></p>
    <div id="stepBlob" data-step="blob"><span id="stepBlobStatus"></span><div id="blobSummary"></div></div>
    <div id="stepMask" data-step="mask"><span id="stepMaskStatus"></span><div id="maskSummary"></div></div>
    <div id="stepWand" data-step="wand"><span id="stepWandStatus"></span><div id="wandSummary"></div></div>
    <div id="stepFloor" data-step="floor"><span id="stepFloorStatus"></span><div id="floorSummary"></div></div>
    <div id="stepExtrinsics" data-step="extrinsics"><span id="stepExtrinsicsStatus"></span><div id="extrinsicsSummary"></div></div>
    <button id="tabIntrinsics" type="button"></button>
    <button id="tabCalibration" type="button"></button>
    <button id="tabTracking" type="button"></button>
    <div id="pageCalibration"></div>
    <pre id="trackingStatus"></pre>
    <pre id="trackingScene"></pre>
  </div>

  <div class="app-shell">

    <!-- ── Sidebar ─────────────────────────────────────── -->
    <nav class="sidebar">
      <div class="sidebar-logo">LT<span>2</span></div>
      <div class="sidebar-nav" id="sidebarNav">
        <!-- populated by renderSidebar() -->
        <div class="nav-section-header">Status</div>
        <button class="nav-item active" data-view="cameras"><span class="step-icon">⊞</span>Camera Status</button>
        <div class="sidebar-divider"></div>
        <div class="nav-section-header">Camera Settings</div>
        <button class="nav-item" data-view="blob"><span class="step-icon">○</span>Blob Detection</button>
        <button class="nav-item" data-view="mask"><span class="step-icon">○</span>Mask Build</button>
        <div class="sidebar-divider"></div>
        <div class="nav-section-header">Camera Calibration</div>
        <button class="nav-item" data-view="intrinsics"><span class="step-icon">◎</span>Intrinsics</button>
        <button class="nav-item" data-view="pose"><span class="step-icon">○</span>Camera Pose Capture</button>
        <button class="nav-item" data-view="floor"><span class="step-icon">○</span>Floor / Metric</button>
        <button class="nav-item" data-view="extrinsics"><span class="step-icon">○</span>Extrinsics</button>
        <div class="sidebar-divider"></div>
        <div class="nav-section-header">Tracking</div>
        <button class="nav-item" data-view="tracking"><span class="step-icon">○</span>Tracking</button>
      </div>
      <div class="sidebar-bottom">
        <button class="btn-ghost btn-sm" data-command="refresh" style="flex:1">Refresh</button>
        <button class="btn-ghost btn-sm" data-command="ping" style="flex:1">Ping</button>
      </div>
    </nav>

    <!-- ── Main area ───────────────────────────────────── -->
    <div class="main-area">

      <!-- VIEW: Camera Status (default) -->
      <div class="view active" id="viewCameras">
        <div class="view-title">Camera Status</div>
        <div class="mjpeg-grid" id="mjpegGridCameras"></div>
        <div class="stat-row" id="camerasSummaryBar">
          <div class="stat-card"><div class="stat-label">Selected</div><div class="stat-value">0</div></div>
          <div class="stat-card"><div class="stat-label">Healthy</div><div class="stat-value">0</div></div>
          <div class="stat-card"><div class="stat-label">Running</div><div class="stat-value">0</div></div>
          <div class="stat-card"><div class="stat-label">Mask Ready</div><div class="stat-value">0</div></div>
        </div>
        <div class="split" style="flex:1;min-height:0">
          <div class="split-left" style="flex:2;display:flex;flex-direction:column;min-height:0">
            <div class="panel" style="flex:1;display:flex;flex-direction:column;min-height:0">
              <div class="tbl-wrap">
                <table>
                  <thead>
                    <tr>
                      <th>☑</th><th>ID</th><th>IP</th><th>State</th><th>Blobs</th><th>Mask%</th><th>Health</th><th>Clock</th>
                    </tr>
                  </thead>
                  <tbody id="cameraRows"></tbody>
                </table>
              </div>
            </div>
          </div>
          <div class="split-right" style="width:300px;min-width:200px;display:flex;flex-direction:column;min-height:0">
            <div class="panel" style="flex:1;display:flex;flex-direction:column;min-height:0">
              <div class="panel-title">Console</div>
              <pre class="console" id="status"></pre>
            </div>
          </div>
        </div>
      </div>

      <!-- VIEW: Blob Detection -->
      <div class="view" id="viewBlob">
        <div class="view-title">Blob Detection Adjustment</div>
        <div class="mjpeg-grid" id="mjpegGridBlob"></div>
        <div class="split" style="flex:1;min-height:0">
          <div class="split-left calib-split-left">
            <div class="panel" style="flex:1;display:flex;flex-direction:column;min-height:0">
              <div class="panel-title">Camera Status</div>
              <div class="tbl-wrap">
                <table>
                  <thead>
                    <tr><th>☑</th><th>ID</th><th>State</th><th>Blobs</th><th>Reject</th><th>Mask%</th><th>Health</th></tr>
                  </thead>
                  <tbody id="blobCameraRows"></tbody>
                </table>
              </div>
            </div>
          </div>
          <div class="split-right calib-split-right">
            <div class="panel">
              <div class="panel-title">Detection Settings</div>
              <div class="slider-grid">
                <div class="slider-label">
                  <span>Exposure <span class="slider-value" id="exposureValue"></span> us</span>
                  <input id="exposure" type="range" min="100" max="30000" step="100" value="12000">
                </div>
                <div class="slider-label">
                  <span>Gain <span class="slider-value" id="gainValue"></span></span>
                  <input id="gain" type="range" min="1" max="16" step="0.1" value="8">
                </div>
                <div class="slider-label">
                  <span>FPS <span class="slider-value" id="fpsValue"></span></span>
                  <input id="fps" type="range" min="15" max="120" step="1" value="56">
                </div>
                <div class="slider-label">
                  <span>Focus <span class="slider-value" id="focusValue"></span></span>
                  <input id="focus" type="range" min="0.0" max="10.0" step="0.001" value="5.215">
                </div>
                <div class="slider-label">
                  <span>Threshold <span class="slider-value" id="thresholdValue"></span></span>
                  <input id="threshold" type="range" min="0" max="255" step="1" value="200">
                </div>
                <div class="slider-label">
                  <span>Circularity <span class="slider-value" id="circularityValue"></span></span>
                  <input id="circularity" type="range" min="0.0" max="1.0" step="0.01" value="0.0">
                </div>
                <div class="slider-label">
                  <span>Blob Min Dia <span class="slider-value" id="blobMinValue"></span> px</span>
                  <input id="blobMin" type="range" min="0" max="100" step="0.5" value="0">
                </div>
                <div class="slider-label">
                  <span>Blob Max Dia <span class="slider-value" id="blobMaxValue"></span> px</span>
                  <input id="blobMax" type="range" min="0" max="100" step="0.5" value="0">
                </div>
              </div>
              <div class="btn-row">
                <button id="applyConfig">Apply Blob Settings</button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- VIEW: Mask Build -->
      <div class="view" id="viewMask">
        <div class="view-title">Mask Build</div>
        <div class="mjpeg-grid" id="mjpegGridMask"></div>
        <div class="split" style="flex:1;min-height:0">
          <div class="split-left calib-split-left">
            <div class="panel" style="flex:1;display:flex;flex-direction:column;min-height:0">
              <div class="panel-title">Camera Mask Status</div>
              <div class="tbl-wrap">
                <table>
                  <thead>
                    <tr><th>☑</th><th>ID</th><th>State</th><th>Mask%</th><th>Blobs</th><th>Health</th></tr>
                  </thead>
                  <tbody id="maskCameraRows"></tbody>
                </table>
              </div>
            </div>
          </div>
          <div class="split-right calib-split-right">
            <div class="panel">
              <div class="panel-title">Mask Settings</div>
              <div class="slider-grid" style="grid-template-columns:1fr">
                <div class="slider-label">
                  <span>Mask Threshold <span class="slider-value" id="maskThresholdValue"></span></span>
                  <input id="maskThreshold" type="range" min="0" max="255" step="1" value="200">
                </div>
                <div class="slider-label">
                  <span>Mask Seconds <span class="slider-value" id="maskSecondsValue"></span> s</span>
                  <input id="maskSeconds" type="range" min="0.1" max="5.0" step="0.1" value="0.5">
                </div>
              </div>
              <div class="btn-row">
                <button class="btn-teal" data-command="mask_start">Build Mask</button>
                <button class="btn-ghost" data-command="mask_stop">Clear Mask</button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- VIEW: Camera Pose Capture -->
      <div class="view" id="viewPose">
        <div class="view-title">Camera Pose Capture</div>
        <div class="split" style="flex:1;min-height:0">
          <div class="split-left calib-split-left">
            <div class="panel" style="flex:1;display:flex;flex-direction:column;min-height:0">
              <div class="panel-title">Camera Capture Status</div>
              <div class="tbl-wrap">
                <table>
                  <thead>
                    <tr><th>☑</th><th>ID</th><th>State</th><th>Blobs</th><th>Mask%</th><th>Health</th></tr>
                  </thead>
                  <tbody id="poseCameraRows"></tbody>
                </table>
              </div>
            </div>
          </div>
          <div class="split-right calib-split-right">
            <div class="panel">
              <div class="panel-title">Capture Control</div>
              <div class="stat-card" style="margin-bottom:10px">
                <div class="stat-label">Running Cameras</div>
                <div class="stat-value" id="poseRunningCount">0</div>
              </div>
              <div class="stat-card" style="margin-bottom:10px">
                <div class="stat-label">State</div>
                <div class="stat-value" style="font-size:14px" id="poseCaptureState">Idle</div>
              </div>
              <div class="btn-row">
                <button class="btn-teal" data-command="start">Start Pose Capture</button>
                <button class="btn-ghost" data-command="stop">Stop</button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- VIEW: Floor / Metric -->
      <div class="view" id="viewFloor">
        <div class="view-title">Floor / Metric</div>
        <div class="mjpeg-grid" id="mjpegGridFloor"></div>
        <div style="flex:1;display:flex;align-items:flex-start;justify-content:flex-start;min-height:0">
        <div class="panel floor-card">
          <div class="panel-title" style="text-align:center">Floor / Metric Capture</div>
          <p style="color:var(--ink-muted);font-size:12px;text-align:center;margin-top:6px">
            Place the wand flat on the floor, then start capture. It will auto-stop.
          </p>
          <div class="floor-countdown" id="floorCountdown"></div>
          <div style="margin-bottom:10px">
            <label>Capture Seconds
              <input id="wandMetricSeconds" type="number" min="0.5" max="10" step="0.5" value="3.0">
            </label>
          </div>
          <div style="margin-bottom:10px">
            <label>Wand Metric Log Path
              <input id="wandMetricLogPath" type="text" value="logs/extrinsics_wand_metric.jsonl">
            </label>
          </div>
          <div class="btn-row" style="justify-content:center">
            <button id="captureWandMetric" class="btn-teal">Capture Floor / Metric</button>
            <button class="btn-ghost" data-command="stop_wand_metric_capture">Stop</button>
          </div>
        </div>
        </div><!-- /.center wrapper -->
      </div>

      <!-- VIEW: Extrinsics -->
      <div class="view" id="viewExtrinsics">
        <div class="view-title">Extrinsics Generation</div>
        <div class="split">
          <div class="split-left" style="flex:1.1;overflow-y:auto">
            <div class="panel">
              <div class="panel-title">Generation Parameters</div>
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px 14px">
                <label>Intrinsics Dir<input id="intrinsicsPath" type="text" value="calibration"></label>
                <label>Pose Log Path<input id="logPath" type="text" value="logs/extrinsics_pose_capture.jsonl"></label>
                <label>Wand Metric Log<input id="generateWandMetricLogPath" type="text" value="logs/extrinsics_wand_metric.jsonl"></label>
                <label>Output Path<input id="outputPath" type="text" value="calibration/extrinsics_pose_v2.json"></label>
                <label>Pair Window (us)<input id="pairWindowUs" type="number" min="1" step="100" value="2000"></label>
                <label>Min Pairs<input id="minPairs" type="number" min="1" step="1" value="8"></label>
                <label>Wand Pair Window (us)<input id="wandPairWindowUs" type="number" min="1" step="100" value="8000"></label>
              </div>
              <div class="btn-row">
                <button id="generateExtrinsics">Generate Extrinsics</button>
              </div>
            </div>
          </div>
          <div class="split-right" style="flex:1;display:flex;flex-direction:column;min-height:0">
            <div class="panel" style="margin-bottom:10px">
              <div class="panel-title">Quality</div>
              <div id="extrinsicsQualityBadges"><span class="quality-badge">No extrinsics yet</span></div>
              <div id="extrinsicsDetailSummary" style="font-size:11px;color:var(--ink-muted);margin-top:6px"></div>
            </div>
          </div>
        </div>
      </div>

      <!-- VIEW: Tracking (keeps legacy ID pageTracking) -->
      <div class="view" id="pageTracking">
        <div class="view-title" style="display:flex;align-items:center;gap:10px">
          3D Tracking
          <span class="tracking-state-badge" id="trackingStatusBadge">Idle</span>
        </div>
        <div class="split" style="flex:1;min-height:0">
          <div class="split-left" style="flex:1.8;display:flex;flex-direction:column;min-height:0">
            <div class="tracking-viewer-inner" style="flex:1;min-height:0">
              <canvas id="trackingViewerCanvas" aria-label="Tracking scene viewport"></canvas>
              <div class="tracking-viewer-empty" id="trackingViewerEmpty">Waiting for scene data</div>
            </div>
            <div style="display:flex;justify-content:space-between;font-size:11px;color:var(--ink-muted);margin-top:6px;padding:0 2px">
              <span id="trackingSceneTimestamp">Last scene: --</span>
              <span id="trackingSceneHint"></span>
            </div>
          </div>
          <div class="split-right" style="width:300px;min-width:240px;display:flex;flex-direction:column;gap:8px;overflow-y:auto">
            <div class="panel">
              <div class="panel-title">Tracking Control</div>
              <div style="font-size:11px;color:var(--ink-muted);margin-bottom:8px" id="trackingExtrinsicsPath"></div>
              <div class="btn-row" style="margin-top:0">
                <button id="trackingStart" class="btn-teal btn-sm" type="button">Start</button>
                <button id="trackingStop" class="btn-ghost btn-sm" type="button">Stop</button>
              </div>
              <div style="font-size:11px;color:var(--ink-muted);margin-top:8px" id="trackingLastUpdate">waiting for data</div>
            </div>
            <div class="panel">
              <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
                <div class="panel-title" style="margin-bottom:0">Camera Health</div>
                <span style="font-size:11px;color:var(--ink-muted)" id="trackingSyncCoverage"></span>
              </div>
              <div id="trackingCameraHealth"></div>
            </div>
            <div class="panel">
              <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
                <div class="panel-title" style="margin-bottom:0">Rigid Bodies</div>
                <span style="font-size:11px;color:var(--ink-muted)" id="trackingRigidCount">0 tracked</span>
              </div>
              <div id="trackingRigidList"></div>
            </div>
          </div>
        </div>
      </div>

      <!-- VIEW: Intrinsics (keeps legacy ID pageIntrinsics) -->
      <div class="view" id="pageIntrinsics">
        <div class="view-title">Intrinsics Calibration</div>
        <div class="split" style="flex:1;min-height:0">
          <div class="split-left" style="flex:1;display:flex;flex-direction:column;gap:8px;overflow-y:auto">
            <div class="panel">
              <div class="panel-title">Camera &amp; Board Setup <span class="badge" id="intPhaseStatus">Idle</span></div>
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px 14px">
                <label>Camera ID<input id="intCameraId" type="text" value="pi-cam-01"></label>
                <label>MJPEG URL<input id="intMjpegUrl" type="text" value="http://192.168.1.10:8555/mjpeg" placeholder="http://PI_IP:8555/mjpeg"></label>
                <label>Square Length (mm)<input id="intSquareMm" type="number" min="1" max="500" step="1" value="30"></label>
                <label>Marker Length (mm)<input id="intMarkerMm" type="number" min="1" max="500" step="0.5" placeholder="auto"></label>
                <label>Squares X<input id="intSquaresX" type="number" min="2" max="20" step="1" value="6"></label>
                <label>Squares Y<input id="intSquaresY" type="number" min="2" max="20" step="1" value="8"></label>
                <label>Min Frames<input id="intMinFrames" type="number" min="5" max="200" step="1" value="25"></label>
                <label>Cooldown (s)<input id="intCooldown" type="number" min="0.1" max="10" step="0.1" value="1.5"></label>
              </div>
              <div id="intCameraList" class="int-camera-list"></div>
              <div class="btn-row">
                <button id="intStart" class="btn-teal btn-sm">Start Capture</button>
                <button id="intStop" class="btn-ghost btn-sm">Stop</button>
                <button id="intClear" class="btn-ghost btn-sm">Clear Frames</button>
                <button id="intDiscard" class="btn-ghost btn-sm">Discard Session</button>
              </div>
            </div>
            <div class="panel" style="flex:1">
              <div class="panel-title">Live Feed</div>
              <div class="intrinsics-viewer">
                <img id="intFrame" alt="Charuco preview" style="display:none">
                <span id="intFrameEmpty" style="color:var(--ink-muted);font-size:13px">Select a camera to preview</span>
              </div>
            </div>
          </div>
          <div class="split-right" style="flex:1;display:flex;flex-direction:column;gap:8px;overflow-y:auto">
            <div class="panel">
              <div class="panel-title">Camera Controls</div>
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:5px 12px;margin-top:6px">
                <label><span class="int-slider-title">Focus <span class="hint" id="intFocusValue">5.215</span></span>
                  <input id="intFocus" type="range" min="0" max="10" step="0.001" value="5.215">
                </label>
                <label><span class="int-slider-title">Exposure <span class="hint" id="intExposureValue">6400</span> µs</span>
                  <input id="intExposure" type="range" min="100" max="30000" step="100" value="6400">
                </label>
                <label><span class="int-slider-title">Gain <span class="hint" id="intGainValue">6.0</span></span>
                  <input id="intGain" type="range" min="1" max="16" step="0.1" value="6.0">
                </label>
                <label><span class="int-slider-title">FPS <span class="hint" id="intFpsValue">60</span></span>
                  <input id="intFps" type="range" min="15" max="120" step="1" value="60">
                </label>
              </div>
              <div class="btn-row" style="margin-top:8px">
                <button id="intApplyCamera" class="btn-teal btn-sm">Apply to Camera</button>
              </div>
            </div>
            <div class="panel">
              <div class="panel-title">Frame Collection</div>
              <div class="int-stat">Captured: <strong id="intCaptured">0</strong> / <strong id="intNeeded">25</strong></div>
              <div class="int-stat">Rejected (cooldown): <strong id="intRejCool">0</strong></div>
              <div class="int-stat">Rejected (spatial): <strong id="intRejSpat">0</strong></div>
              <div class="int-stat">Rejected (no detect): <strong id="intRejDet">0</strong></div>
              <p style="font-size:11px;color:var(--ink-muted);margin-top:8px;text-transform:uppercase;letter-spacing:.07em">3x3 Coverage Grid</p>
              <div class="int-grid" id="intCoverageGrid"></div>
              <div class="btn-row">
                <button id="intCalibrate" class="btn-sm">Calibrate</button>
              </div>
            </div>
            <div class="panel" style="flex:1;display:flex;flex-direction:column;min-height:0">
              <div class="panel-title">Calibration Output</div>
              <pre class="console" id="intResult">Waiting...</pre>
            </div>
          </div>
        </div>
      </div>

    </div><!-- /.main-area -->
  </div><!-- /.app-shell -->

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
    const stepOrder = ["blob", "mask", "wand", "floor", "extrinsics"];
    const stepLabels = {
      blob: "Blob",
      mask: "Mask",
      wand: "Pose Capture",
      floor: "Floor / Metric",
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
      generateWandMetricLogPath: document.getElementById("generateWandMetricLogPath"),
      outputPath: document.getElementById("outputPath"),
      pairWindowUs: document.getElementById("pairWindowUs"),
      wandPairWindowUs: document.getElementById("wandPairWindowUs"),
      minPairs: document.getElementById("minPairs"),
      wandMetricSeconds: document.getElementById("wandMetricSeconds"),
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
      floorSummary: document.getElementById("floorSummary"),
      extrinsicsSummary: document.getElementById("extrinsicsSummary"),
      stepBlob: document.getElementById("stepBlob"),
      stepMask: document.getElementById("stepMask"),
      stepWand: document.getElementById("stepWand"),
      stepFloor: document.getElementById("stepFloor"),
      stepExtrinsics: document.getElementById("stepExtrinsics"),
      stepBlobStatus: document.getElementById("stepBlobStatus"),
      stepMaskStatus: document.getElementById("stepMaskStatus"),
      stepWandStatus: document.getElementById("stepWandStatus"),
      stepFloorStatus: document.getElementById("stepFloorStatus"),
      stepExtrinsicsStatus: document.getElementById("stepExtrinsicsStatus"),
      tabIntrinsics: document.getElementById("tabIntrinsics"),
      tabCalibration: document.getElementById("tabCalibration"),
      tabTracking: document.getElementById("tabTracking"),
      pageIntrinsics: document.getElementById("pageIntrinsics"),
      pageCalibration: document.getElementById("pageCalibration"),
      pageTracking: document.getElementById("pageTracking"),
      intCameraId: document.getElementById("intCameraId"),
      intMjpegUrl: document.getElementById("intMjpegUrl"),
      intSquareMm: document.getElementById("intSquareMm"),
      intMarkerMm: document.getElementById("intMarkerMm"),
      intSquaresX: document.getElementById("intSquaresX"),
      intSquaresY: document.getElementById("intSquaresY"),
      intMinFrames: document.getElementById("intMinFrames"),
      intCooldown: document.getElementById("intCooldown"),
      intStart: document.getElementById("intStart"),
      intStop: document.getElementById("intStop"),
      intClear: document.getElementById("intClear"),
      intDiscard: document.getElementById("intDiscard"),
      intCalibrate: document.getElementById("intCalibrate"),
      intPhaseStatus: document.getElementById("intPhaseStatus"),
      intCaptured: document.getElementById("intCaptured"),
      intNeeded: document.getElementById("intNeeded"),
      intRejCool: document.getElementById("intRejCool"),
      intRejSpat: document.getElementById("intRejSpat"),
      intRejDet: document.getElementById("intRejDet"),
      intCameraList: document.getElementById("intCameraList"),
      intCoverageGrid: document.getElementById("intCoverageGrid"),
      intResult: document.getElementById("intResult"),
      intFrame: document.getElementById("intFrame"),
      intFrameEmpty: document.getElementById("intFrameEmpty"),
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
      if (elements.status) elements.status.textContent = `Error\n${context}: ${message}`;
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
        wand_metric_seconds: Number(elements.wandMetricSeconds.value),
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
        if (step === "wand" && workflow.pose_capture_complete) {
          status = "done";
        }
        if (step === "floor" && workflow.wand_metric_complete) {
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
      const extrinsicsQuality = workflow.latest_extrinsics_quality || {};
      const activeCaptureKind = workflow.active_capture_kind || null;

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
      elements.wandSummary.textContent = activeCaptureKind === "pose_capture"
        ? `${workflow.running_count} 台が pose capture 中です。Pi デスクトップ preview で単一点の追従と誤検出の有無を見ます。`
        : workflow.pose_capture_complete
        ? `Pose capture は完了しています（${workflow.pose_capture_log_path || "logs/extrinsics_pose_capture.jsonl"}）。続けて Generate Extrinsics を実行します。`
        : `mask 完了後に Start Pose Capture を実行します。収録中は Pi preview で単一点ターゲット追従を監視します。`;
      elements.floorSummary.textContent = activeCaptureKind === "wand_metric_capture"
        ? `floor / metric capture 実行中です。wand を床に静置したまま自動停止を待ちます。`
        : workflow.wand_metric_complete
        ? `Floor / metric capture は完了しています（${workflow.wand_metric_log_path || "logs/extrinsics_wand_metric.jsonl"}）。Generate Extrinsics で scale / floor を反映できます。`
        : `Pose capture の後、wand を床に置いて Capture Floor / Metric を実行します。静止したまま数秒だけ収録します。`;
      elements.extrinsicsSummary.textContent = workflow.extrinsics_ready
        ? `Extrinsics は生成済みです。usable=${extrinsicsQuality.usable_rows ?? "-"}, median=${extrinsicsQuality.median_reproj_error_px ?? "-"}px, p90=${extrinsicsQuality.p90_reproj_error_px ?? "-"}px, match p90=${extrinsicsQuality.matched_delta_us_p90 ?? "-"}us。metric/world の詳細は console の result を確認します。`
        : `pose capture と floor / metric capture の後に Generate Extrinsics を実行します。wand log があれば metric / world まで解きます。`;

      const stepMap = {
        blob: elements.stepBlob,
        mask: elements.stepMask,
        wand: elements.stepWand,
        floor: elements.stepFloor,
        extrinsics: elements.stepExtrinsics,
      };
      const stepStatusMap = {
        blob: elements.stepBlobStatus,
        mask: elements.stepMaskStatus,
        wand: elements.stepWandStatus,
        floor: elements.stepFloorStatus,
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
      const locked = selected.filter((camera) => camera.diagnostics?.clock_sync?.status === "locked").length;
      elements.cameraSummary.innerHTML = [
        `<div class="metric"><span class="eyebrow">Selected</span><strong>${selected.length}</strong><p>現在の操作対象</p></div>`,
        `<div class="metric"><span class="eyebrow">Healthy</span><strong>${healthy}</strong><p>ping ack=true</p></div>`,
        `<div class="metric"><span class="eyebrow">Running</span><strong>${running}</strong><p>capture 実行中</p></div>`,
        `<div class="metric"><span class="eyebrow">PTP Locked</span><strong>${locked}</strong><p>offset <= 500us</p></div>`,
        `<div class="metric"><span class="eyebrow">Preview Enabled</span><strong>${preview}</strong><p>Pi desktop OpenCV</p></div>`,
      ].join("");
    }

    function renderCameraRows(cameras) {
      renderMjpegGrids(cameras);
      // Main cameras view table (compact)
      elements.rows.innerHTML = cameras.map((camera) => {
        const diagnostics = camera.diagnostics || {};
        const blob = diagnostics.blob_diagnostics || {};
        const maskRatio = diagnostics.mask_ratio != null ? `${(Number(diagnostics.mask_ratio) * 100).toFixed(1)}%` : "-";
        const clockSync = diagnostics.clock_sync || {};
        const offsetUs = Number(clockSync.offset_us);
        const offsetLabel = Number.isFinite(offsetUs) ? `${Math.round(offsetUs)}us` : "n/a";
        const clockVariant = clockSync.status === "locked" ? "ok" : (clockSync.status === "degraded" ? "warn" : "muted");
        const clockLabel = clockSync.status || "?";
        return `
          <tr>
            <td><input type="checkbox" data-camera="${escapeHtml(camera.camera_id)}" ${camera.selected ? "checked" : ""}></td>
            <td>${escapeHtml(camera.camera_id)}</td>
            <td>${escapeHtml(camera.ip)}</td>
            <td>${escapeHtml(diagnostics.state || "")}</td>
            <td>${escapeHtml(blob.last_blob_count ?? "")}</td>
            <td>${escapeHtml(maskRatio)}</td>
            <td>${camera.healthy ? badge("OK", "ok") : badge("No Ack", "warn")}</td>
            <td>${badge(clockLabel, clockVariant)} ${escapeHtml(offsetLabel)}</td>
          </tr>
        `;
      }).join("");

      // Blob view table
      const blobRows = document.getElementById("blobCameraRows");
      if (blobRows) {
        blobRows.innerHTML = cameras.map(camera => {
          const d = camera.diagnostics || {};
          const blob = d.blob_diagnostics || {};
          const maskRatio = d.mask_ratio != null ? `${(Number(d.mask_ratio)*100).toFixed(1)}%` : "-";
          const rejectCount = Number(blob.rejected_by_diameter||0) + Number(blob.rejected_by_circularity||0);
          return `<tr>
            <td><input type="checkbox" data-camera="${escapeHtml(camera.camera_id)}" ${camera.selected?"checked":""}></td>
            <td>${escapeHtml(camera.camera_id)}</td>
            <td>${escapeHtml(d.state||"")}</td>
            <td>${escapeHtml(blob.last_blob_count??"")}</td>
            <td>${escapeHtml(rejectCount)}</td>
            <td>${escapeHtml(maskRatio)}</td>
            <td>${camera.healthy ? badge("OK","ok") : badge("No Ack","warn")}</td>
          </tr>`;
        }).join("");
      }

      // Mask view table
      const maskRows = document.getElementById("maskCameraRows");
      if (maskRows) {
        maskRows.innerHTML = cameras.map(camera => {
          const d = camera.diagnostics || {};
          const blob = d.blob_diagnostics || {};
          const maskRatio = d.mask_ratio != null ? Number(d.mask_ratio) : null;
          const maskPct = maskRatio != null ? `${(maskRatio*100).toFixed(1)}%` : "-";
          const barWidth = maskRatio != null ? Math.min(100, maskRatio*100).toFixed(0) : 0;
          return `<tr>
            <td><input type="checkbox" data-camera="${escapeHtml(camera.camera_id)}" ${camera.selected?"checked":""}></td>
            <td>${escapeHtml(camera.camera_id)}</td>
            <td>${escapeHtml(d.state||"")}</td>
            <td><div class="mini-bar-wrap"><div class="mini-bar"><div class="mini-bar-fill" style="width:${barWidth}%"></div></div>${escapeHtml(maskPct)}</div></td>
            <td>${escapeHtml(blob.last_blob_count??"")}</td>
            <td>${camera.healthy ? badge("OK","ok") : badge("No Ack","warn")}</td>
          </tr>`;
        }).join("");
      }

      // Pose view table
      const poseRows = document.getElementById("poseCameraRows");
      if (poseRows) {
        poseRows.innerHTML = cameras.map(camera => {
          const d = camera.diagnostics || {};
          const blob = d.blob_diagnostics || {};
          const maskRatio = d.mask_ratio != null ? `${(Number(d.mask_ratio)*100).toFixed(1)}%` : "-";
          const isRunning = d.state === "RUNNING";
          return `<tr>
            <td><input type="checkbox" data-camera="${escapeHtml(camera.camera_id)}" ${camera.selected?"checked":""}></td>
            <td>${escapeHtml(camera.camera_id)}</td>
            <td>${isRunning ? badge("RUNNING","ok") : escapeHtml(d.state||"")}</td>
            <td>${escapeHtml(blob.last_blob_count??"")}</td>
            <td>${escapeHtml(maskRatio)}</td>
            <td>${camera.healthy ? badge("OK","ok") : badge("No Ack","warn")}</td>
          </tr>`;
        }).join("");
      }
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

    let activeView = "cameras";

    // Views that require Pi preview to be ON
    const PREVIEW_ON_VIEWS = new Set(["cameras", "blob", "floor"]);
    // Views that explicitly turn Pi preview OFF
    const PREVIEW_OFF_VIEWS = new Set(["pose", "tracking"]);

    function showStatus(msg, variant) {
      if (!elements.status) return;
      elements.status.textContent = msg;
    }

    function renderMjpegGrids(cameras) {
      const gridIds = ["mjpegGridCameras", "mjpegGridBlob", "mjpegGridMask", "mjpegGridFloor"];
      const html = cameras
        .filter(c => c.ip)
        .map(c => `<div class="mjpeg-cell">
          <img src="http://${escapeHtml(c.ip)}:8555/mjpeg" alt="${escapeHtml(c.camera_id)}">
          <div class="mjpeg-label">${escapeHtml(c.camera_id)}</div>
        </div>`).join("");
      gridIds.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.innerHTML = html;
      });
    }

    function setActiveView(viewName) {
      document.querySelectorAll(".view").forEach(v => v.classList.remove("active"));
      const target = document.getElementById(
        viewName === "tracking" ? "pageTracking" :
        viewName === "intrinsics" ? "pageIntrinsics" :
        "view" + viewName.charAt(0).toUpperCase() + viewName.slice(1)
      );
      if (target) target.classList.add("active");
      activeView = viewName;
      document.querySelectorAll(".nav-item[data-view]").forEach(item => {
        item.classList.toggle("active", item.dataset.view === viewName);
      });
      if (viewName === "intrinsics") {
        intStartFramePoller();
        intStartStatusPoller();
      } else {
        intStopPollers();
      }
      if (viewName === "tracking") {
        trackingViewer.resize();
      }
      // Toggle Pi MJPEG preview based on page
      if (PREVIEW_ON_VIEWS.has(viewName) || PREVIEW_OFF_VIEWS.has(viewName)) {
        const renderEnabled = PREVIEW_ON_VIEWS.has(viewName);
        postJson("/api/command", {
          command: "set_preview",
          render_enabled: renderEnabled,
          camera_ids: selectedCameraIds(),
        }).catch(() => {});
      }
    }

    function setActivePage(pageName) {
      if (pageName === "tracking") setActiveView("tracking");
      else if (pageName === "intrinsics") setActiveView("intrinsics");
      else setActiveView("cameras");
      activePage = pageName === "tracking" ? "tracking" : pageName === "intrinsics" ? "intrinsics" : "calibration";
      elements.pageCalibration.classList.toggle("active", activePage === "calibration");
      elements.pageTracking.classList.toggle("active", activePage === "tracking");
      elements.pageIntrinsics.classList.toggle("active", activePage === "intrinsics");
      elements.tabCalibration.classList.toggle("active", activePage === "calibration");
      elements.tabTracking.classList.toggle("active", activePage === "tracking");
      elements.tabIntrinsics.classList.toggle("active", activePage === "intrinsics");
    }

    function renderSidebar(state) {
      const wf = state.workflow || {};
      const statuses = segmentStatuses(wf);
      const activeCaptureKind = wf.active_capture_kind || null;
      const selectedCount = wf.selected_count || 0;
      const maskReadyCount = wf.mask_ready_count || 0;

      function navItem(viewName, icon, label, extraClass, badge) {
        const isActive = activeView === viewName ? " active" : "";
        const cls = (extraClass ? " " + extraClass : "");
        const badgeHtml = badge ? `<span class="nav-badge">${badge}</span>` : "";
        return `<button class="nav-item${isActive}${cls}" data-view="${viewName}">
          <span class="step-icon">${icon}</span>${label}${badgeHtml}
        </button>`;
      }

      function stepIcon(step) {
        const isPose = step === "wand";
        const isFloor = step === "floor";
        if (isPose && activeCaptureKind === "pose_capture") return "⟳";
        if (isFloor && activeCaptureKind === "wand_metric_capture") return "⟳";
        if (statuses[step] === "done") return "✓";
        if (statuses[step] === "current") return "●";
        return "○";
      }

      function stepClass(step) {
        const isPose = step === "wand" && activeCaptureKind === "pose_capture";
        const isFloor = step === "floor" && activeCaptureKind === "wand_metric_capture";
        if (isPose || isFloor) return "inprogress";
        if (statuses[step] === "done") return "done";
        return "";
      }

      const maskBadge = selectedCount > 0 ? `${maskReadyCount}/${selectedCount}` : null;
      const trackingClass = !wf.extrinsics_ready ? "disabled" : "";
      const trackingBadge = state.tracking?.running ? "LIVE" : null;

      const navHtml = [
        `<div class="nav-section-header">Status</div>`,
        navItem("cameras", "⊞", "Camera Status", "", null),
        `<div class="sidebar-divider"></div>`,
        `<div class="nav-section-header">Camera Settings</div>`,
        navItem("blob", stepIcon("blob"), "Blob Detection", stepClass("blob"), null),
        navItem("mask", stepIcon("mask"), "Mask Build", stepClass("mask"), maskBadge),
        `<div class="sidebar-divider"></div>`,
        `<div class="nav-section-header">Camera Calibration</div>`,
        navItem("intrinsics", "◎", "Intrinsics", "", null),
        navItem("pose", stepIcon("wand"), "Camera Pose Capture", stepClass("wand"), null),
        navItem("floor", stepIcon("floor"), "Floor / Metric", stepClass("floor"), null),
        navItem("extrinsics", stepIcon("extrinsics"), "Extrinsics", stepClass("extrinsics"), null),
        `<div class="sidebar-divider"></div>`,
        `<div class="nav-section-header">Tracking</div>`,
        navItem("tracking", state.tracking?.running ? "▶" : "○", "Tracking", trackingClass, trackingBadge),
      ].join("");

      document.getElementById("sidebarNav").innerHTML = navHtml;
      document.querySelectorAll(".nav-item[data-view]").forEach(item => {
        item.addEventListener("click", () => {
          if (!item.classList.contains("disabled")) setActiveView(item.dataset.view);
        });
      });
    }

    function renderExtrinsicsQuality(workflow) {
      const q = workflow.latest_extrinsics_quality || {};
      const el = document.getElementById("extrinsicsQualityBadges");
      if (!el) return;
      if (!workflow.extrinsics_ready) {
        el.innerHTML = '<span class="quality-badge">No extrinsics yet</span>';
        return;
      }
      const usable = q.usable_rows;
      const median = q.median_reproj_error_px;
      const p90 = q.p90_reproj_error_px;
      const matchP90 = q.matched_delta_us_p90;
      const medianClass = median != null ? (median < 1.5 ? "ok" : median < 3 ? "warn" : "error") : "";
      el.innerHTML = [
        usable != null ? `<span class="quality-badge ok">✓ usable=${usable}</span>` : "",
        median != null ? `<span class="quality-badge ${medianClass}">median ${Number(median).toFixed(2)}px</span>` : "",
        p90 != null ? `<span class="quality-badge">p90 ${Number(p90).toFixed(2)}px</span>` : "",
        matchP90 != null ? `<span class="quality-badge">match p90 ${matchP90}µs</span>` : "",
      ].join("");
      const detailEl = document.getElementById("extrinsicsDetailSummary");
      if (detailEl) {
        detailEl.textContent = elements.extrinsicsSummary.textContent || "—";
      }
    }

    function renderCamerasSummaryBar(state) {
      const el = document.getElementById("camerasSummaryBar");
      if (!el) return;
      const wf = state.workflow || {};
      el.innerHTML = [
        `<div class="stat-card"><div class="stat-label">Selected</div><div class="stat-value">${wf.selected_count??0}</div></div>`,
        `<div class="stat-card"><div class="stat-label">Healthy</div><div class="stat-value">${wf.healthy_count??0}</div></div>`,
        `<div class="stat-card"><div class="stat-label">Running</div><div class="stat-value">${wf.running_count??0}</div></div>`,
        `<div class="stat-card"><div class="stat-label">Mask Ready</div><div class="stat-value">${wf.mask_ready_count??0}</div></div>`,
      ].join("");
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
        wandMetricSeconds: state.config.wand_metric_seconds ?? 3.0,
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
      if (document.activeElement !== elements.wandMetricSeconds) {
        elements.wandMetricSeconds.value = String(state.config.wand_metric_seconds ?? 3.0);
      }
      if (state.workflow?.pose_capture_log_path) {
        elements.logPath.value = state.workflow.pose_capture_log_path;
      }
      if (state.workflow?.wand_metric_log_path) {
        elements.wandMetricLogPath.value = state.workflow.wand_metric_log_path;
        elements.generateWandMetricLogPath.value = state.workflow.wand_metric_log_path;
      }
      renderCameraRows(state.cameras || []);
      renderCameraSummary(state.cameras || []);
      renderWorkflow(state);
      elements.status.textContent = JSON.stringify(state.last_result, null, 2);
      // Update pose view counters
      const workflow = state.workflow || {};
      const poseRunEl = document.getElementById("poseRunningCount");
      if (poseRunEl) poseRunEl.textContent = workflow.running_count ?? 0;
      const poseCaptureStateEl = document.getElementById("poseCaptureState");
      if (poseCaptureStateEl) {
        poseCaptureStateEl.textContent = workflow.active_capture_kind === "pose_capture" ? "Capturing" :
          workflow.pose_capture_complete ? "Done ✓" : "Idle";
      }
      renderSidebar(state);
      renderExtrinsicsQuality(state.workflow || {});
      renderCamerasSummaryBar(state);
      await loadTracking();
      // Restore intrinsics form fields from saved settings.
      // Keep camera selection local to the current UI session so quick camera
      // switching is not overwritten by periodic state refreshes.
      const is = state.intrinsics_settings;
      if (is && Date.now() > intFieldsUserEdited) {
        const fields = [
          ["intMjpegUrl", is.mjpeg_url],
          ["intSquareMm", is.square_length_mm],
          ["intMarkerMm", is.marker_length_mm != null ? is.marker_length_mm : ""],
          ["intSquaresX", is.squares_x],
          ["intSquaresY", is.squares_y],
          ["intMinFrames", is.min_frames],
          ["intCooldown", is.cooldown_s],
        ];
        for (const [id, val] of fields) {
          const el = elements[id];
          if (el && document.activeElement !== el && val != null && val !== "") {
            el.value = val;
          }
        }
      }
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
      showStatus("Waiting...");
      try {
        await postJson("/api/config", configPayload());
        await safeLoadState();
        showStatus("Done!");
      } catch (error) {
        reportUiError("apply_config", error);
      }
    });

    document.querySelectorAll("button[data-command]").forEach((button) => {
      button.addEventListener("click", async () => {
        showStatus("Waiting...");
        try {
          await postJson("/api/command", {
            command: button.dataset.command,
            camera_ids: selectedCameraIds(),
          });
          await safeLoadState();
          showStatus("Done!");
        } catch (error) {
          reportUiError(`command_${button.dataset.command || "unknown"}`, error);
        }
      });
    });

    document.getElementById("captureWandMetric").addEventListener("click", async () => {
      showStatus("Waiting...");
      try {
        await postJson("/api/command", {
          command: "start_wand_metric_capture",
          camera_ids: selectedCameraIds(),
          duration_s: Number(elements.wandMetricSeconds.value),
          wand_metric_log_path: elements.wandMetricLogPath.value,
        });
        await safeLoadState();
        showStatus("Done!");
      } catch (error) {
        reportUiError("start_wand_metric_capture", error);
      }
    });

    document.getElementById("generateExtrinsics").addEventListener("click", async () => {
      showStatus("Waiting...");
      try {
        await postJson("/api/generate_extrinsics", {
          intrinsics_path: elements.intrinsicsPath.value,
          pose_log_path: elements.logPath.value,
          wand_metric_log_path: elements.generateWandMetricLogPath.value,
          output_path: elements.outputPath.value,
          pair_window_us: Number(elements.pairWindowUs.value),
          wand_pair_window_us: Number(elements.wandPairWindowUs.value),
          min_pairs: Number(elements.minPairs.value),
        });
        await safeLoadState();
        showStatus("Done!");
      } catch (error) {
        reportUiError("generate_extrinsics", error);
      }
    });

    elements.tabCalibration.addEventListener("click", () => setActivePage("calibration"));
    elements.tabTracking.addEventListener("click", () => setActivePage("tracking"));
    elements.tabIntrinsics.addEventListener("click", () => setActivePage("intrinsics"));

    elements.trackingStart.addEventListener("click", async () => {
      showStatus("Waiting...");
      try {
        await postJson("/api/tracking/start", {
          patterns: ["waist"],
        });
        setActivePage("tracking");
        await safeLoadState();
        showStatus("Done!");
      } catch (error) {
        reportUiError("tracking_start", error);
      }
    });

    elements.trackingStop.addEventListener("click", async () => {
      showStatus("Waiting...");
      try {
        await postJson("/api/tracking/stop", {});
        setActivePage("tracking");
        await safeLoadState();
        showStatus("Done!");
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

    // ── Intrinsics tab ────────────────────────────────────────────────
    let intFrameIntervalId = null;
    let intStatusIntervalId = null;
    let intCurrentBlobUrl = null;
    // Guard: timestamp (ms) until which loadState must NOT overwrite intrinsics fields
    let intFieldsUserEdited = 0;

    function intStopPollers() {
      if (intFrameIntervalId) { clearInterval(intFrameIntervalId); intFrameIntervalId = null; }
      if (intStatusIntervalId) { clearInterval(intStatusIntervalId); intStatusIntervalId = null; }
    }

    function intStartFramePoller() {
      if (intFrameIntervalId) return;
      intFrameIntervalId = setInterval(async () => {
        try {
          const resp = await fetch("/api/intrinsics/frame.jpg", { cache: "no-store" });
          if (resp.status === 204) return;
          if (!resp.ok) return;
          const blob = await resp.blob();
          const url = URL.createObjectURL(blob);
          if (intCurrentBlobUrl) URL.revokeObjectURL(intCurrentBlobUrl);
          intCurrentBlobUrl = url;
          elements.intFrame.src = url;
          elements.intFrame.style.display = "block";
          elements.intFrameEmpty.style.display = "none";
        } catch (_) {}
      }, 500);
    }

    function intStartStatusPoller() {
      if (intStatusIntervalId) return;
      intStatusIntervalId = setInterval(async () => {
        try {
          const resp = await fetch("/api/intrinsics/status");
          if (!resp.ok) return;
          const s = await resp.json();
          elements.intPhaseStatus.textContent = s.phase ?? "idle";
          elements.intCaptured.textContent = s.frames_captured ?? 0;
          elements.intNeeded.textContent = s.frames_needed ?? 25;
          elements.intRejCool.textContent = s.frames_rejected_cooldown ?? 0;
          elements.intRejSpat.textContent = s.frames_rejected_spatial ?? 0;
          elements.intRejDet.textContent = s.frames_rejected_detection ?? 0;
          // 3x3 coverage grid
          const grid = s.grid_coverage || [[0,0,0],[0,0,0],[0,0,0]];
          if (elements.intCoverageGrid) {
            elements.intCoverageGrid.innerHTML = grid.flat().map(v =>
              `<div class="int-cell${v > 0 ? ' covered' : ''}">${v}</div>`
            ).join("");
          }
          if (s.phase === "done" && s.calibration_result) {
            elements.intResult.textContent = JSON.stringify(s.calibration_result, null, 2);
          } else if (s.last_error) {
            elements.intResult.textContent = JSON.stringify({ error: s.last_error }, null, 2);
          }
          // Render discovered cameras as quick-fill buttons for MJPEG URL
          if (elements.intCameraList) {
            const cams = Array.isArray(s.cameras) ? s.cameras : [];
            if (cams.length > 0) {
              elements.intCameraList.innerHTML = cams.map(c => {
                const url = `http://${c.ip}:8555/mjpeg`;
                return `<button type="button" class="ghost" style="font-size:12px;padding:4px 10px" data-cam="${escapeHtml(c.camera_id)}" data-url="${escapeHtml(url)}">${escapeHtml(c.camera_id)} <span style="color:var(--muted)">${escapeHtml(c.ip)}</span></button>`;
              }).join("");
              elements.intCameraList.querySelectorAll("button[data-cam]").forEach(btn => {
                btn.onclick = () => {
                  if (elements.intCameraId) elements.intCameraId.value = btn.dataset.cam;
                  if (elements.intMjpegUrl) elements.intMjpegUrl.value = btn.dataset.url;
                  const url = btn.dataset.url || "";
                  if (url && elements.intFrame) {
                    if (intFrameIntervalId) { clearInterval(intFrameIntervalId); intFrameIntervalId = null; }
                    if (intCurrentBlobUrl) { URL.revokeObjectURL(intCurrentBlobUrl); intCurrentBlobUrl = null; }
                    elements.intFrame.src = url;
                    elements.intFrame.style.display = "block";
                    if (elements.intFrameEmpty) elements.intFrameEmpty.style.display = "none";
                    intStartStatusPoller();
                  }
                  intFieldsUserEdited = Date.now() + 60000; // block server restore for 60s
                };
              });
            } else {
              elements.intCameraList.innerHTML = "";
            }
          }
        } catch (_) {}
      }, 1000);
    }

    function intPayload() {
      const markerVal = elements.intMarkerMm ? elements.intMarkerMm.value.trim() : "";
      return {
        camera_id: elements.intCameraId?.value.trim() || "pi-cam-01",
        mjpeg_url: elements.intMjpegUrl?.value.trim() || "",
        square_length_mm: Number(elements.intSquareMm?.value ?? 30),
        marker_length_mm: markerVal !== "" ? Number(markerVal) : null,
        squares_x: Number(elements.intSquaresX?.value ?? 6),
        squares_y: Number(elements.intSquaresY?.value ?? 8),
        min_frames: Number(elements.intMinFrames?.value ?? 25),
        cooldown_s: Number(elements.intCooldown?.value ?? 1.5),
      };
    }

    // Lock fields when user types directly into Camera ID or MJPEG URL
    elements.intCameraId?.addEventListener("input", () => { intFieldsUserEdited = Date.now() + 60000; });
    elements.intMjpegUrl?.addEventListener("input", () => { intFieldsUserEdited = Date.now() + 60000; });

    elements.intStart?.addEventListener("click", async () => {
      intFieldsUserEdited = Date.now() + 60000; // keep user-entered values after start
      try { await postJson("/api/intrinsics/start", intPayload()); } catch (e) { reportUiError("int_start", e); }
    });
    elements.intStop?.addEventListener("click", async () => {
      // Stop server-side capture
      try { await postJson("/api/intrinsics/stop", {}); } catch (e) { reportUiError("int_stop", e); }
      // Stop all pollers
      intStopPollers();
      // Stop MJPEG feed and hide frame
      if (elements.intFrame) {
        elements.intFrame.src = "";
        elements.intFrame.style.display = "none";
      }
      if (elements.intFrameEmpty) elements.intFrameEmpty.style.display = "";
      if (intCurrentBlobUrl) { URL.revokeObjectURL(intCurrentBlobUrl); intCurrentBlobUrl = null; }
    });
    elements.intClear?.addEventListener("click", async () => {
      try { await postJson("/api/intrinsics/clear", {}); } catch (e) { reportUiError("int_clear", e); }
    });
    elements.intDiscard?.addEventListener("click", async () => {
      try {
        await postJson("/api/intrinsics/discard", {});
        elements.intResult.textContent = "Waiting...";
        elements.intFrame.style.display = "none";
        elements.intFrameEmpty.style.display = "";
      } catch (e) { reportUiError("int_discard", e); }
    });
    elements.intCalibrate?.addEventListener("click", async () => {
      try { await postJson("/api/intrinsics/calibrate", {}); } catch (e) { reportUiError("int_calibrate", e); }
    });

    // ── end Intrinsics tab ────────────────────────────────────────────

    // ── Intrinsics Camera Controls ────────────────────────────────────
    const intCamSliders = [
      { id: "intFocus",    valId: "intFocusValue" },
      { id: "intExposure", valId: "intExposureValue" },
      { id: "intGain",     valId: "intGainValue" },
      { id: "intFps",      valId: "intFpsValue" },
    ];
    const intCamTimers = {};

    function intCamConfigPayload() {
      const blobRange = normalizedBlobRange();
      return {
        camera_ids: [elements.intCameraId?.value?.trim() || "pi-cam-01"],
        focus:            Number(document.getElementById("intFocus")?.value ?? 5.215),
        exposure_us:      Number(document.getElementById("intExposure")?.value ?? 6400),
        gain:             Number(document.getElementById("intGain")?.value ?? 6.0),
        fps:              Number(document.getElementById("intFps")?.value ?? 60),
        circularity_min:  Number(elements.circularity.value),
        blob_min_diameter_px: blobRange.min,
        blob_max_diameter_px: blobRange.max,
        mask_threshold:   Number(elements.maskThreshold.value),
        mask_seconds:     Number(elements.maskSeconds.value),
        wand_metric_seconds: Number(elements.wandMetricSeconds.value),
      };
    }

    intCamSliders.forEach(({ id, valId }) => {
      const input = document.getElementById(id);
      const span  = document.getElementById(valId);
      if (!input) return;
      if (span) span.textContent = input.value;
      input.addEventListener("input", () => {
        if (span) span.textContent = input.value;
        if (intCamTimers[id]) clearTimeout(intCamTimers[id]);
        intCamTimers[id] = setTimeout(async () => {
          try { await postJson("/api/config", intCamConfigPayload()); }
          catch (e) { reportUiError("int_cam_slider_" + id, e); }
          finally { intCamTimers[id] = null; }
        }, 200);
      });
    });

    document.getElementById("intApplyCamera")?.addEventListener("click", async () => {
      try { await postJson("/api/config", intCamConfigPayload()); }
      catch (e) { reportUiError("int_apply_camera", e); }
    });
    // ── end Intrinsics Camera Controls ───────────────────────────────

    setActiveView("cameras");
    initTrackingViewer();
    safeLoadState();
    setInterval(safeLoadState, 3000);
  </script>
</body>
</html>
"""


class LoutrackGuiState:
    def __init__(
        self,
        session: CalibrationSession,
        receiver: UDPReceiver,
        settings_path: Path | None = None,
        tracking_runtime: TrackingRuntime | None = None,
    ) -> None:
        self.session = session
        self.receiver = receiver
        self.lock = threading.Lock()
        self._uses_default_settings_path = settings_path is None
        self.settings_path = settings_path or DEFAULT_SETTINGS_PATH
        self._migrate_settings_once()
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
        self._capture_auto_stop_timer: threading.Timer | None = None
        self._receiver_frame_callback = getattr(receiver, "_frame_callback", None)
        if hasattr(self.receiver, "set_frame_callback"):
            self.receiver.set_frame_callback(self._on_frame_received)
        self._generate_extrinsics_registry: ExtrinsicsMethodRegistry = build_default_extrinsics_registry(
            blob_pose_solver=_load_extrinsics_solver(),
        )
        # Kept for backward-compatibility with existing tests and callers.
        self._generate_extrinsics_solver = self._generate_extrinsics_registry.get("blob_pose_v2").solve
        self.tracking_runtime = tracking_runtime or TrackingRuntime()
        self.latest_extrinsics_path: Path | None = None
        self.latest_extrinsics_quality: Dict[str, Any] | None = None
        self._restore_latest_extrinsics(DEFAULT_EXTRINSICS_OUTPUT_PATH)
        self._intrinsics: Optional[IntrinsicsCapture] = None
        self._intrinsics_lock = threading.Lock()

    @staticmethod
    def _default_settings_payload() -> Dict[str, Any]:
        return {
            "exposure_us": 12000,
            "gain": 8.0,
            "fps": 56,
            "focus": 5.215,
            "threshold": 200,
            "blob_min_diameter_px": None,
            "blob_max_diameter_px": None,
            "circularity_min": 0.0,
            "mask_threshold": 200,
            "mask_seconds": 0.5,
            "wand_metric_seconds": DEFAULT_WAND_METRIC_DURATION_S,
        }

    def _migrate_settings_once(self) -> None:
        if not self._uses_default_settings_path:
            return
        old_path = OLD_SETTINGS_PATH
        if not old_path.exists() or old_path == self.settings_path:
            return
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        if self.settings_path.exists():
            old_path.unlink(missing_ok=True)
            return
        try:
            payload = json.loads(old_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("legacy settings is not an object")
            self.settings_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            old_path.unlink(missing_ok=True)
            return
        except Exception:
            backup_path = old_path.with_suffix(old_path.suffix + ".invalid.bak")
            try:
                shutil.move(str(old_path), str(backup_path))
            except Exception:
                old_path.unlink(missing_ok=True)
            self.settings_path.write_text(
                json.dumps(self._default_settings_payload(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

    def _default_config(self) -> CalibrationSessionConfig:
        return CalibrationSessionConfig(exposure_us=12000, gain=8.0, fps=56, duration_s=DEFAULT_WAND_METRIC_DURATION_S)

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

    def _load_initial_config(self) -> CalibrationSessionConfig:
        config = self._default_config()
        payload = self._read_settings_payload()
        if not isinstance(payload, dict):
            return config
        return self._build_session_config(payload, base_config=config)

    def _persist_config(self) -> None:
        payload = self._config_payload()
        path = self.settings_path
        try:
            existing = self._read_settings_payload()
            existing.update(payload)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(existing, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            return

    def _mask_params(self, config: CalibrationSessionConfig | None = None) -> Dict[str, Any]:
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
            "wand_metric_seconds": self.config.duration_s,
        }

    def _build_session_config(
        self,
        payload: Dict[str, Any],
        base_config: CalibrationSessionConfig | None = None,
    ) -> CalibrationSessionConfig:
        source = base_config or self.config
        mask = self._mask_params(source)
        mask["threshold"] = int(payload.get("mask_threshold", mask["threshold"]))
        mask["seconds"] = float(payload.get("mask_seconds", mask["seconds"]))
        return CalibrationSessionConfig(
            exposure_us=int(payload.get("exposure_us", source.exposure_us)),
            gain=float(payload.get("gain", source.gain)),
            fps=int(payload.get("fps", source.fps)),
            focus=float(payload.get("focus", source.focus)),
            threshold=int(payload.get("threshold", source.threshold)),
            blob_min_diameter_px=payload.get("blob_min_diameter_px", source.blob_min_diameter_px),
            blob_max_diameter_px=payload.get("blob_max_diameter_px", source.blob_max_diameter_px),
            circularity_min=float(payload.get("circularity_min", source.circularity_min)),
            duration_s=float(payload.get("wand_metric_seconds", source.duration_s)),
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
        with self.lock:
            active_capture_kind = self._active_capture_kind

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
        pose_capture_complete = pose_capture_exists or bool(self._capture_completed.get("pose_capture"))
        wand_metric_exists = self.wand_metric_log_path.exists() and self.wand_metric_log_path.is_file()
        wand_metric_complete = wand_metric_exists or bool(self._capture_completed.get("wand_metric_capture"))
        extrinsics_ready = self.latest_extrinsics_path is not None and self.latest_extrinsics_path.exists() and self.latest_extrinsics_path.is_file()

        active_segment = "blob"
        if running_count > 0:
            active_segment = "wand"
        elif pose_capture_complete and not wand_metric_complete:
            active_segment = "floor"
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
            "pose_capture_exists": pose_capture_exists,
            "pose_capture_log_path": str(self.pose_capture_log_path),
            "wand_metric_complete": wand_metric_complete,
            "wand_metric_exists": wand_metric_exists,
            "wand_metric_log_path": str(self.wand_metric_log_path),
            "extrinsics_ready": extrinsics_ready,
            "latest_extrinsics_path": str(self.latest_extrinsics_path) if self.latest_extrinsics_path else None,
            "latest_extrinsics_quality": self.latest_extrinsics_quality,
            "active_segment": active_segment,
            "active_capture_kind": active_capture_kind,
        }

    def _tracking_extrinsics_ready(self) -> bool:
        path = self.latest_extrinsics_path
        return bool(path is not None and path.exists() and path.is_file())

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
        solve_summary = payload.get("pose", {}).get("solve_summary", {}) if isinstance(payload.get("pose"), dict) else {}
        if isinstance(solve_summary, dict):
            self.latest_extrinsics_quality = {
                key: solve_summary.get(key)
                for key in (
                    "usable_rows",
                    "complete_rows",
                    "median_reproj_error_px",
                    "p90_reproj_error_px",
                    "matched_delta_us_p50",
                    "matched_delta_us_p90",
                    "matched_delta_us_max",
                )
                if key in solve_summary
            }
        else:
            self.latest_extrinsics_quality = {}

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
        if resolved.is_file() and resolved.name.startswith("extrinsics_pose_v2"):
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

    # ── Intrinsics capture ─────────────────────────────────────────────

    def start_intrinsics_capture(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        camera_id = str(payload.get("camera_id", "pi-cam-01")).strip()
        mjpeg_url = str(payload.get("mjpeg_url", "")).strip()
        square_length_mm = float(payload.get("square_length_mm", 30.0))
        marker_length_mm_raw = payload.get("marker_length_mm")
        marker_length_mm = float(marker_length_mm_raw) if marker_length_mm_raw is not None else None
        squares_x = int(payload.get("squares_x", 6))
        squares_y = int(payload.get("squares_y", 8))
        min_frames = int(payload.get("min_frames", 25))
        cooldown_s = float(payload.get("cooldown_s", 1.5))

        if not mjpeg_url:
            raise ValueError("mjpeg_url is required")
        if square_length_mm <= 0:
            raise ValueError("square_length_mm must be > 0")

        config = IntrinsicsConfig(
            camera_id=camera_id,
            mjpeg_url=mjpeg_url,
            square_length_mm=square_length_mm,
            marker_length_mm=marker_length_mm,
            squares_x=squares_x,
            squares_y=squares_y,
            min_frames=min_frames,
            cooldown_s=cooldown_s,
            output_dir=PROJECT_ROOT / "calibration",
        )
        with self._intrinsics_lock:
            if self._intrinsics is not None:
                self._intrinsics.stop()
            self._intrinsics = IntrinsicsCapture(config)
            self._intrinsics.start()
        self._persist_intrinsics_settings(payload)
        return {"ok": True, "camera_id": camera_id}

    def stop_intrinsics_capture(self) -> Dict[str, Any]:
        with self._intrinsics_lock:
            cap = self._intrinsics
        if cap is not None:
            cap.stop()
        return {"ok": True}

    def clear_intrinsics_frames(self) -> Dict[str, Any]:
        with self._intrinsics_lock:
            cap = self._intrinsics
        if cap is not None:
            cap.clear()
        return {"ok": True}

    def trigger_intrinsics_calibration(self) -> Dict[str, Any]:
        with self._intrinsics_lock:
            cap = self._intrinsics
        if cap is None:
            raise ValueError("No active intrinsics capture session")
        cap.trigger_calibration()
        return {"ok": True}

    def discard_intrinsics_capture(self) -> Dict[str, Any]:
        with self._intrinsics_lock:
            if self._intrinsics is not None:
                self._intrinsics.stop()
                self._intrinsics = None
        return {"ok": True}

    def get_intrinsics_status(self) -> Dict[str, Any]:
        with self._intrinsics_lock:
            cap = self._intrinsics
        if cap is None:
            targets = self.session.discover_targets(None)
            camera_list = [{"camera_id": t.camera_id, "ip": t.ip} for t in targets]
            return {
                "phase": "idle",
                "camera_id": None,
                "frames_captured": 0,
                "frames_needed": 25,
                "frames_target": 50,
                "frames_rejected_cooldown": 0,
                "frames_rejected_spatial": 0,
                "frames_rejected_detection": 0,
                "grid_coverage": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                "last_error": None,
                "calibration_result": None,
                "output_path": None,
                "cameras": camera_list,
            }
        status = cap.get_status()
        targets = self.session.discover_targets(None)
        status["cameras"] = [{"camera_id": t.camera_id, "ip": t.ip} for t in targets]
        return status

    def get_intrinsics_jpeg(self) -> Optional[bytes]:
        with self._intrinsics_lock:
            cap = self._intrinsics
        if cap is None:
            return None
        return cap.get_latest_jpeg()

    def _persist_intrinsics_settings(self, payload: Dict[str, Any]) -> None:
        path = self.settings_path
        try:
            existing = self._read_settings_payload()
            marker_mm_raw = payload.get("marker_length_mm")
            existing["intrinsics"] = {
                "camera_id": str(payload.get("camera_id", "pi-cam-01")),
                "mjpeg_url": str(payload.get("mjpeg_url", "")),
                "square_length_mm": float(payload.get("square_length_mm", 30.0)),
                "marker_length_mm": float(marker_mm_raw) if marker_mm_raw is not None else None,
                "squares_x": int(payload.get("squares_x", 6)),
                "squares_y": int(payload.get("squares_y", 8)),
                "min_frames": int(payload.get("min_frames", 25)),
                "cooldown_s": float(payload.get("cooldown_s", 1.5)),
            }
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(existing, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _load_intrinsics_settings(self) -> Dict[str, Any]:
        defaults: Dict[str, Any] = {
            "camera_id": "pi-cam-01",
            "mjpeg_url": "",
            "square_length_mm": 30.0,
            "marker_length_mm": None,
            "squares_x": 6,
            "squares_y": 8,
            "min_frames": 25,
            "cooldown_s": 1.5,
        }
        try:
            payload = self._read_settings_payload()
            saved = payload.get("intrinsics", {}) if isinstance(payload, dict) else {}
            if isinstance(saved, dict):
                for k in defaults:
                    if k in saved:
                        defaults[k] = saved[k]
        except Exception:
            pass
        return defaults

    # ── end Intrinsics capture ─────────────────────────────────────────

    def get_state(self) -> Dict[str, Any]:
        cameras = self.refresh_targets()
        return {
            "config": self._config_payload(),
            "cameras": cameras,
            "workflow": self._workflow_summary(cameras),
            "extrinsics_methods": self._generate_extrinsics_registry.to_payload(),
            "last_result": self.last_result,
            "receiver": self.receiver.stats,
            "tracking": self.get_tracking_status(),
            "intrinsics_settings": self._load_intrinsics_settings(),
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
        if command == "set_preview":
            kwargs: Dict[str, Any] = {}
            render_enabled = payload.get("render_enabled")
            if render_enabled is not None:
                kwargs["render_enabled"] = bool(render_enabled)
            result = self.session._broadcast(targets, "set_preview", **kwargs)
            self._update_camera_status({command: result})
            self.last_result = {command: result}
            return self.last_result
        if command in ("start", "start_pose_capture"):
            log_path = self._start_capture_log("pose_capture")
            result = self.session._broadcast(targets, "start", mode="pose_capture")
            payload_out: Dict[str, Any] = {command: result, "capture_log": {"path": str(log_path)}}
            if not self._all_acked(result):
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"].update(stop_meta)
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if command == "start_wand_metric_capture":
            duration_s = float(payload.get("duration_s", self.config.duration_s))
            if duration_s <= 0.0:
                raise ValueError("duration_s must be > 0")
            wand_log_raw = str(payload.get("wand_metric_log_path", "")).strip()
            if wand_log_raw:
                self.wand_metric_log_path = self._resolve_project_path(wand_log_raw, DEFAULT_WAND_METRIC_LOG_PATH)
            log_path = self._start_capture_log("wand_metric_capture")
            result = self.session._broadcast(targets, "start", mode="wand_metric_capture")
            payload_out = {
                command: result,
                "capture_log": {"path": str(log_path)},
                "duration_s": duration_s,
            }
            if not self._all_acked(result):
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"].update(stop_meta)
            else:
                self._schedule_auto_stop([target.camera_id for target in targets], duration_s, "wand_metric_capture")
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if command in ("stop", "stop_pose_capture", "stop_wand_metric_capture"):
            self._cancel_capture_timer()
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
        extrinsics_method = str(payload.get("extrinsics_method", "blob_pose_v2")).strip() or "blob_pose_v2"
        intrinsics_raw = str(payload.get("intrinsics_path", "calibration")).strip()
        log_raw = str(payload.get("pose_log_path", payload.get("log_path", str(DEFAULT_POSE_LOG_PATH)))).strip()
        output_raw = str(payload.get("output_path", str(DEFAULT_EXTRINSICS_OUTPUT_PATH))).strip()
        intrinsics_path = self._resolve_project_path(intrinsics_raw, Path("calibration"))
        resolved_log_path = self._resolve_project_path(log_raw, DEFAULT_POSE_LOG_PATH)
        wand_log_raw = str(payload.get("wand_metric_log_path", str(self.wand_metric_log_path))).strip()
        resolved_output = self._resolve_project_path(output_raw, DEFAULT_EXTRINSICS_OUTPUT_PATH)
        resolved_wand_log_path = self._resolve_project_path(wand_log_raw, DEFAULT_WAND_METRIC_LOG_PATH)
        pair_window_us = int(payload.get("pair_window_us", 2000))
        min_pairs = int(payload.get("min_pairs", 8))
        wand_pair_window_us = int(payload.get("wand_pair_window_us", 8000))
        if pair_window_us < 1:
            raise ValueError("pair_window_us must be >= 1")
        if min_pairs < 1:
            raise ValueError("min_pairs must be >= 1")
        if wand_pair_window_us < 1:
            raise ValueError("wand_pair_window_us must be >= 1")

        if not resolved_log_path.exists():
            summary = self._build_generate_extrinsics_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=f"log_path does not exist: {resolved_log_path}",
                resolved_wand_log_path=resolved_wand_log_path,
            )
            self.last_result = {"generate_extrinsics": summary}
            return self.last_result
        if not resolved_log_path.is_file():
            summary = self._build_generate_extrinsics_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=f"log_path is not a file: {resolved_log_path}",
                resolved_wand_log_path=resolved_wand_log_path,
            )
            self.last_result = {"generate_extrinsics": summary}
            return self.last_result

        try:
            if extrinsics_method == "blob_pose_v2" and callable(self._generate_extrinsics_solver):
                solve_fn = self._generate_extrinsics_solver
            else:
                solve_fn = self._generate_extrinsics_registry.get(extrinsics_method).solve
            raw_result = solve_fn(
                intrinsics_path=str(intrinsics_path),
                pose_log_path=str(resolved_log_path),
                output_path=str(resolved_output),
                pair_window_us=pair_window_us,
                min_pairs=min_pairs,
                wand_metric_log_path=str(resolved_wand_log_path) if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file() else None,
                wand_pair_window_us=wand_pair_window_us,
            )
        except (FileNotFoundError, ValueError) as exc:
            reason = str(exc)
            if isinstance(exc, FileNotFoundError):
                missing_path = Path(getattr(exc, "filename", "") or str(resolved_log_path))
                reason = f"log_path does not exist: {missing_path}"
            summary = self._build_generate_extrinsics_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=reason,
                resolved_wand_log_path=resolved_wand_log_path,
            )
            self.last_result = {"generate_extrinsics": summary}
            return self.last_result
        if not isinstance(raw_result, dict):
            raise ValueError("extrinsics solver returned invalid response")

        pose_section = raw_result.get("pose", {})
        camera_rows = pose_section.get("camera_poses", []) if isinstance(pose_section, dict) else []
        camera_count = len(camera_rows) if isinstance(camera_rows, list) else 0
        solve_summary = pose_section.get("solve_summary", {}) if isinstance(pose_section, dict) else {}
        if not isinstance(solve_summary, dict):
            solve_summary = {}
        quality_summary = {
            key: solve_summary.get(key)
            for key in (
                "usable_rows",
                "complete_rows",
                "median_reproj_error_px",
                "p90_reproj_error_px",
                "matched_delta_us_p50",
                "matched_delta_us_p90",
                "matched_delta_us_max",
            )
            if key in solve_summary
        }

        summary = {
            "ok": True,
            "extrinsics_method": extrinsics_method,
            "camera_order": raw_result.get("camera_order", []),
            "camera_count": camera_count,
            "output_path": str(resolved_output),
            "quality": quality_summary,
            "metric_status": raw_result.get("metric", {}).get("status"),
            "world_status": raw_result.get("world", {}).get("status"),
            "wand_metric_log_path": str(resolved_wand_log_path) if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file() else None,
        }
        self.latest_extrinsics_path = resolved_output
        self.latest_extrinsics_quality = quality_summary
        self.last_result = {"generate_extrinsics": summary}
        return self.last_result

    @staticmethod
    def _extract_pose_log_payload(line: str) -> Dict[str, Any] | None:
        line = line.strip()
        if not line:
            return None
        payload = json.loads(line)
        if isinstance(payload, dict) and payload.get("_type") == "frame":
            frame = payload.get("data")
            if isinstance(frame, dict):
                return frame
        return payload if isinstance(payload, dict) else None

    def _summarize_pose_log(self, log_path: Path) -> Dict[str, Any]:
        total_by_camera: Counter[str] = Counter()
        single_blob_by_camera: Counter[str] = Counter()
        invalid_rows = 0
        try:
            with log_path.open("r", encoding="utf-8") as handle:
                for raw_line in handle:
                    try:
                        payload = self._extract_pose_log_payload(raw_line)
                    except json.JSONDecodeError:
                        invalid_rows += 1
                        continue
                    if not payload:
                        continue
                    camera_id = str(payload.get("camera_id", "")).strip()
                    if not camera_id:
                        continue
                    blobs = payload.get("blobs", [])
                    if not isinstance(blobs, list):
                        blobs = []
                    blob_count = payload.get("blob_count", len(blobs))
                    try:
                        blob_count = int(blob_count)
                    except (TypeError, ValueError):
                        blob_count = len(blobs)
                    total_by_camera[camera_id] += 1
                    if blob_count == 1 and len(blobs) == 1:
                        single_blob_by_camera[camera_id] += 1
        except OSError:
            return {
                "exists": False,
                "is_file": False,
                "rows_by_camera": {},
                "single_blob_rows_by_camera": {},
                "usable_camera_count": 0,
                "invalid_rows": invalid_rows,
            }
        return {
            "exists": log_path.exists(),
            "is_file": log_path.is_file(),
            "rows_by_camera": dict(total_by_camera),
            "single_blob_rows_by_camera": dict(single_blob_by_camera),
            "usable_camera_count": sum(1 for count in single_blob_by_camera.values() if count > 0),
            "invalid_rows": invalid_rows,
        }

    def _build_generate_extrinsics_failure(
        self,
        *,
        resolved_log_path: Path,
        resolved_output: Path,
        reason: str,
        resolved_wand_log_path: Path,
    ) -> Dict[str, Any]:
        pose_log_summary = self._summarize_pose_log(resolved_log_path) if resolved_log_path.exists() and resolved_log_path.is_file() else {
            "exists": resolved_log_path.exists(),
            "is_file": resolved_log_path.is_file(),
            "rows_by_camera": {},
            "single_blob_rows_by_camera": {},
            "usable_camera_count": 0,
            "invalid_rows": 0,
        }
        return {
            "ok": False,
            "output_path": str(resolved_output),
            "error": reason,
            "pose_log_path": str(resolved_log_path),
            "pose_log_summary": pose_log_summary,
            "wand_metric_log_path": str(resolved_wand_log_path) if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file() else None,
        }

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
                if self._active_capture_kind != capture_kind:
                    raise ValueError(f"capture already active: {self._active_capture_kind}")
                default_path = (
                    self.pose_capture_log_path
                    if capture_kind == "pose_capture"
                    else self.wand_metric_log_path
                )
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
            else:
                self.wand_metric_log_path = Path(log_file)
            return Path(log_file)

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

    def _cancel_capture_timer(self) -> None:
        with self.lock:
            timer = self._capture_auto_stop_timer
            self._capture_auto_stop_timer = None
        if timer is not None:
            timer.cancel()

    def _schedule_auto_stop(self, camera_ids: List[str], duration_s: float, capture_kind: str) -> None:
        def _auto_stop() -> None:
            try:
                targets = self.session.discover_targets(camera_ids or None)
                result = self.session._broadcast(targets, "stop")
                payload_out: Dict[str, Any] = {f"stop_{capture_kind}": result}
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"] = stop_meta
                self._update_camera_status({"stop": result})
                self.last_result = payload_out
            finally:
                with self.lock:
                    self._capture_auto_stop_timer = None

        timer = threading.Timer(duration_s, _auto_stop)
        timer.daemon = True
        with self.lock:
            if self._capture_auto_stop_timer is not None:
                self._capture_auto_stop_timer.cancel()
            self._capture_auto_stop_timer = timer
        timer.start()

    @staticmethod
    def _all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
        return all(bool(resp.get("ack")) for resp in responses.values())


class LoutrackGuiHandler(BaseHTTPRequestHandler):
    state: LoutrackGuiState

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
        if path == "/api/intrinsics/status":
            self._send_json(self.state.get_intrinsics_status())
            return
        if path == "/api/intrinsics/frame.jpg":
            jpeg = self.state.get_intrinsics_jpeg()
            if jpeg is None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
            else:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
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
            if self.path == "/api/tracking/start":
                self._debug_log("POST /api/tracking/start")
                self._send_json(self.state.start_tracking(payload))
                return
            if self.path == "/api/tracking/stop":
                self._debug_log("POST /api/tracking/stop")
                self._send_json(self.state.stop_tracking())
                return
            if self.path == "/api/intrinsics/start":
                self._send_json(self.state.start_intrinsics_capture(payload))
                return
            if self.path == "/api/intrinsics/stop":
                self._send_json(self.state.stop_intrinsics_capture())
                return
            if self.path == "/api/intrinsics/clear":
                self._send_json(self.state.clear_intrinsics_frames())
                return
            if self.path == "/api/intrinsics/calibrate":
                self._send_json(self.state.trigger_intrinsics_calibration())
                return
            if self.path == "/api/intrinsics/discard":
                self._send_json(self.state.discard_intrinsics_capture())
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
        print(f"[loutrack_gui] {message}", flush=True)

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
    parser = argparse.ArgumentParser(description="Loutrack GUI for calibration and tracking control")
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
    solve_extrinsics = getattr(module, "solve_extrinsics", None)
    if not callable(solve_extrinsics):
        raise RuntimeError("solve_extrinsics is missing in calibrate_extrinsics.py")
    return solve_extrinsics


def main() -> None:
    args = parse_args()
    receiver = UDPReceiver(port=args.udp_port)
    receiver.start()
    session = CalibrationSession(
        inventory_path=Path(args.inventory) if args.inventory else None,
        receiver=receiver,
    )
    state = LoutrackGuiState(session=session, receiver=receiver)
    LoutrackGuiHandler.state = state
    server = ThreadingHTTPServer((args.host, args.port), LoutrackGuiHandler)
    try:
        print(f"Loutrack GUI listening on http://{args.host}:{args.port}")
        server.serve_forever()
    finally:
        server.server_close()
        receiver.stop()


if __name__ == "__main__":
    main()
