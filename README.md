# Loutrack2

Loutrack2 is an open-source optical motion tracking project built around Raspberry Pi camera nodes, host-side calibration tools, and a GUI-based workflow.

日本語はこっち→: [README_ja.md](README_ja.md)

![Loutrack Pi device](readme_imgs/pi_device.png)

## Overview

Loutrack2 combines:

- Raspberry Pi camera capture nodes
- host-side control and calibration
- multi-camera 3D reconstruction
- a GUI workflow for setup, tuning, capture, and inspection

The repository is intended as a practical base for building and evaluating a custom camera-based tracking system. The current implementation is most mature in capture, calibration, and 3D reconstruction.

## System Overview

```mermaid
flowchart LR
  subgraph PiCameras["Raspberry Pi Camera Nodes"]
    A["pi-cam-01<br/>capture_runtime.py"]
    B["pi-cam-02+<br/>capture_runtime.py"]
    C["Blob detection<br/>Mask / Preview / UDP streaming"]
  end

  subgraph Host["Host Workstation"]
    D["loutrack_gui.py<br/>Web GUI"]
    E["UDPReceiver"]
    F["Calibration session"]
    G["Intrinsics / Extrinsics tools"]
    H["Tracking runtime"]
    I["3D reconstruction"]
  end

  subgraph Outputs["Outputs"]
    J["Intrinsics JSON"]
    K["Extrinsics JSON"]
    L["3D marker positions"]
    M["Tracking scene snapshot"]
  end

  A -->|UDP frames| E
  B -->|UDP frames| E
  D -->|TCP control| A
  D -->|TCP control| B
  E --> F
  F --> G
  G --> J
  G --> K
  E --> H
  H --> I
  I --> L
  H --> M
```

## Current Status

Loutrack2 currently provides a working multi-camera optical tracking foundation.

Available now:

- Raspberry Pi capture nodes can detect reflective blobs and stream observations to the host
- the host GUI can drive blob tuning, mask building, pose capture, floor or metric capture, and extrinsics generation
- the calibration flow can produce intrinsics and extrinsics JSON outputs
- synchronized multi-camera observations can be reconstructed into 3D marker positions
- the host can inspect tracking state, scene snapshots, and calibration-related metrics

Current scope:

- useful for building and validating a custom multi-camera tracking setup
- not yet a finished full-body tracking or IK output system

## Roadmap

The project direction after the current baseline includes:

- more robust clustering and identity tracking for multiple rigid bodies
- body-part level tracking for head, chest, waist, and feet
- more stable rigid-body association through the full pipeline
- IK-friendly pose output
- SteamVR tracker output
- further improvements to setup, deployment, and hardware documentation

## GUI Workflow

The GUI already covers a connected workflow from camera bring-up to tracking inspection.

![Loutrack GUI](readme_imgs/wip_gui.png)

Typical flow:

1. Start the Pi capture nodes
2. Open the host GUI
3. Adjust blob detection
4. Build masks
5. Capture pose data
6. Capture floor or metric data
7. Generate extrinsics
8. Inspect tracking and scene snapshots

## Hardware Direction

Loutrack2 also includes a hardware direction centered around DIY tracking camera nodes and tracked targets.

![Tracked HMD prototype](readme_imgs/HMD_tracker.png)

Current hardware direction:

- Raspberry Pi-based camera nodes built from commonly available parts
- Raspberry Pi Camera Module 3 Wide NoIR as the current camera configuration
- a PoE HAT so the node can be powered and networked over a single LAN cable
- a custom board mounted above the Pi that combines camera holding and IR LED illumination
- open hardware files in the repository, including PCB design data and 3D-printable parts
- reflective tracked markers built from 3D-printed spheres and retroreflective tape

What this enables:

- active IR illumination from the device side
- camera capture through an IR-pass configuration for higher-SNR reflective blob detection
- ceiling or room installation with simplified wiring through PoE
- a hardware stack that can be reproduced with DIY fabrication and off-the-shelf components

Relevant repository areas:

- [`hardware`](hardware) for printable parts and board-related assets
- [`hardware/LED board`](hardware/LED%20board) for the custom LED board design files
- [`hardware/pi mount`](hardware/pi%20mount) for Pi mount printable parts

The longer-term goal is a tracking platform that can be assembled, modified, and extended by the community.

## Open Source

Loutrack2 is being developed in the open.

- pull requests are welcome
- forks and experiments are welcome
- documentation, setup improvements, hardware refinements, and calibration workflow improvements are particularly useful

## License

This project is intended to be released under `GPL-3.0-or-later`.

## Repository Map

- [`src/pi`](src/pi) for Raspberry Pi capture services
- [`src/host`](src/host) for the host GUI, receiver, runtime, and tracking pipeline
- [`src/camera-calibration`](src/camera-calibration) for intrinsics and extrinsics tooling
- [`src/calibration`](src/calibration) for calibration domain types and targets
- [`calibration`](calibration) for generated calibration artifacts
- [`schema`](schema) for message and control contracts
- [`tests`](tests) for regression coverage
