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
    }
    @media (max-width: 720px) {
      main {
        padding: 20px 14px 32px;
      }
      .controls-grid {
        grid-template-columns: 1fr;
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
            <label for="exposure">Exposure <span class="hint"><span id="exposureValue"></span> us</span><input id="exposure" type="range" min="100" max="30000" step="100" value="1200"></label>
            <label for="gain">Gain <span class="hint"><span id="gainValue"></span></span><input id="gain" type="range" min="1" max="16" step="0.1" value="4"></label>
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
            <button class="secondary" data-command="start">Start Wand Capture</button>
            <button class="ghost" data-command="stop">Stop Capture</button>
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
            <label for="logPath">Log Path<input id="logPath" type="text" value="logs/wand_capture.jsonl"></label>
            <label for="outputPath">Output Path<input id="outputPath" type="text" value="calibration/calibration_extrinsics_v1.json"></label>
          </div>
          <div class="button-row">
            <button id="generateExtrinsics">Generate Extrinsics</button>
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
  </main>
  <script>
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
      outputPath: document.getElementById("outputPath"),
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
        if (step === "extrinsics" && workflow.extrinsics_ready) {
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
        : `mask 完了後に Start Wand Capture を実行します。収録中は Pi preview で wand 軌跡と blob 数の崩れを監視します。`;
      elements.extrinsicsSummary.textContent = workflow.extrinsics_ready
        ? `Extrinsics は生成済みです。出力先と reference camera を console で確認します。`
        : `wand 収録ログのパスを確認してから Extrinsics を生成します。`;

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
      sliders.forEach((name) => {
        values[name].textContent = elements[name].value;
      });
      renderCameraRows(state.cameras || []);
      renderCameraSummary(state.cameras || []);
      renderWorkflow(state);
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
        self.config = SessionConfig(exposure_us=1200, gain=4.0, fps=56, duration_s=60.0)
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
        extrinsics_ready = bool(
            isinstance(self.last_result.get("generate_extrinsics"), dict)
            and self.last_result["generate_extrinsics"].get("ok")
        )

        active_segment = "blob"
        if extrinsics_ready:
            active_segment = "extrinsics"
        elif running_count > 0 or (selected_count > 0 and mask_ready_count >= selected_count):
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
            "extrinsics_ready": extrinsics_ready,
            "active_segment": active_segment,
        }

    def get_state(self) -> Dict[str, Any]:
        cameras = self.refresh_targets()
        return {
            "config": self._config_payload(),
            "cameras": cameras,
            "workflow": self._workflow_summary(cameras),
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
            "mask_stop": lambda: self.session._broadcast(targets, "mask_stop"),
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
