# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Loutrack2** is a distributed motion capture system: multiple Raspberry Pi cameras detect 2D blobs of reflective markers, stream UDP frames to a Host machine, which triangulates 3D positions and tracks rigid bodies in real time.

**Workflow stages**: Intrinsics calibration → Extrinsics calibration (wand GUI) → Live tracking (GUI)

## Commands

### Environment Setup
```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
```

### Run Tests
```bash
# All tests
.venv/bin/pytest tests/

# Single test file
.venv/bin/pytest tests/test_extrinsics_pose_v2.py -v

# Single test
.venv/bin/pytest tests/test_extrinsics_pose_v2.py::test_solver_converges -v
```

### Key Runtime Commands
```bash
# Start GUI (extrinsics calibration + tracking)
python src/host/loutrack_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000

# Start Pi capture (on Raspberry Pi)
.venv/bin/python src/pi/service/capture_runtime.py --camera-id pi-cam-01 --udp-dest <HOST_IP>:5000

# Pi capture with dummy backend (no camera hardware)
.venv/bin/python src/pi/service/capture_runtime.py --backend dummy --camera-id pi-cam-01 --udp-dest localhost:5000

# Extrinsics from CLI
.venv/bin/python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --wand-metric-log logs/extrinsics_wand_metric.jsonl \
  --output calibration/extrinsics_pose_v2.json
```

## Architecture

### System Topology
```
Raspberry Pi (x N)          Host Machine
  capture_runtime.py          loutrack_gui.py (HTTP server)
  - blob detection             receiver.py (UDP frame pairing)
  - PTP sync                   geo.py (DLT triangulation)
  - UDP → port 5000  ──────→   rigid.py (Kabsch pose estimation)
  - TCP ← port 8554  ←──────   pipeline.py (integrated tracker)
```

### Key Source Locations
- `src/pi/service/capture_runtime.py` — Pi-side state machine (IDLE→MASK_INIT→READY→RUNNING), blob detection, UDP emission, TCP control server
- `src/host/loutrack_gui.py` — Web GUI; HTTP API; orchestrates all calibration steps and tracking
- `src/host/receiver.py` — UDP reception, timestamp-based frame pairing across cameras
- `src/host/geo.py` — DLT triangulation, camera projection utilities
- `src/host/rigid.py` — DBSCAN clustering, Kabsch/SVD rigid body estimation, tracker
- `src/camera-calibration/calibrate_extrinsics.py` — Full extrinsics solver pipeline (init → BA → metric scale)
- `src/camera-calibration/calibrate.py` — Intrinsics from Charuco board
- `schema/` — JSON schemas for UDP frame format, control commands, calibration outputs

### Pi State Machine
`IDLE → MASK_INIT (30 frames) → READY ↔ RUNNING`

Static mask must be built (READY state) before any capture mode works. All modes (capture, pose_capture, wand_metric_capture) use the same single pipeline.

### Calibration Output Format (`extrinsics_pose_v2.json`)
Three sections: `pose` (camera transforms), `metric` (scale factor from wand geometry), `world` (final world-frame extrinsics). `world` is what the tracker consumes.

### Control Protocol (TCP 8554)
NDJSON over TCP. Host sends `{request_id, camera_id, cmd, params}`, Pi responds `{request_id, camera_id, ack, error_code}`. Key commands: `ping`, `start`/`stop`, `set_exposure`/`set_gain`/`set_fps`, `mask_start`/`mask_stop`.

### Timestamps
Unix epoch microseconds. Pi prioritizes sensor metadata timestamp; falls back to dequeue time. `timestamp_source` field in UDP payload identifies which. PTP synchronizes clocks across Pis (pi-cam-01 is Grandmaster master).

## Conventions

### Camera IDs
`pi-cam-01` (PTP master/Grandmaster), `pi-cam-02`, `pi-cam-03`, … Must match `--camera-id` flag and intrinsics filename: `calibration_intrinsics_v1_<camera_id>.json`.

### File Locations
- Calibration outputs: `calibration/`
- Runtime logs: `logs/` (JSONL format)
- GUI state persistence: `logs/loutrack_gui_settings.json`

### Testing
- Tests use pytest; no mocking of core geometry or solver logic — test with synthetic data
- When adding/updating tests, consolidate similar tests and remove stale duplicates in the same change (per AGENTS.md)

## Always Update README.md
Per AGENTS.md: update `README.md` whenever making progress or implementation changes. Keep file/directory references in README accurate.
