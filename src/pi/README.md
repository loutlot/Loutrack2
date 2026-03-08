# Pi Capture Service

`src/pi/capture.py` is the Raspberry Pi-side capture service.

- TCP control (NDJSON): `0.0.0.0:8554`
- UDP frames (one JSON per datagram): default `255.255.255.255:5000`
- Default capture resolution: `2304x1296`
- Startup logs print listen address, backend, preview state, and capture start/stop status

## Run locally (dummy backend)

Use this on macOS/Linux for development; it generates synthetic dot frames.

```bash
.venv/bin/python src/pi/capture.py \
  --backend dummy \
  --camera-id pi-cam-01 \
  --udp-dest localhost:5000
```

## Run on Raspberry Pi (picamera2 backend)

On Raspberry Pi OS, use the real camera backend.

```bash
.venv/bin/python src/pi/capture.py \
  --backend picamera2 \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000
```

To show a local OpenCV debug preview on the Pi during idle, mask setup, and wand capture, add `--debug-preview`.

```bash
.venv/bin/python src/pi/capture.py \
  --backend picamera2 \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000 \
  --debug-preview
```

If you start capture over SSH, make sure the desktop X display is exported first.

```bash
export DISPLAY=:0
export XAUTHORITY=/home/pi/.Xauthority
```

The preview is debug-only. It stays active while the service is idle, while `mask_start` is building the mask, and while frames are streaming. It overlays accepted blobs, the active mask, and current detection parameters. If `DISPLAY` is not set, the service now disables the preview before touching OpenCV HighGUI and keeps running without the window.

If `picamera2` is not available, the service stays alive (e.g. `ping` still works) and `start` returns `ack:false` with `error_code=6`.

### Required Pi packages (high level)

- `picamera2` (Python)
- `libcamera` stack (system)
- `numpy`, `opencv-python` (Python, matching this repo's requirements)
