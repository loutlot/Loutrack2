# Pi Capture Service

`src/pi/capture.py` is the Raspberry Pi-side capture service.

- TCP control (NDJSON): `0.0.0.0:8554`
- UDP frames (one JSON per datagram): default `255.255.255.255:5000`

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

To show a local OpenCV debug preview on the Pi while streaming, add `--debug-preview`.

```bash
.venv/bin/python src/pi/capture.py \
  --backend picamera2 \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000 \
  --debug-preview
```

The preview is debug-only. It overlays accepted blobs, the active mask, and current detection parameters. If OpenCV window creation fails, the service keeps running and the preview disables itself.

If `picamera2` is not available, the service stays alive (e.g. `ping` still works) and `start` returns `ack:false` with `error_code=6`.

### Required Pi packages (high level)

- `picamera2` (Python)
- `libcamera` stack (system)
- `numpy`, `opencv-python` (Python, matching this repo's requirements)
