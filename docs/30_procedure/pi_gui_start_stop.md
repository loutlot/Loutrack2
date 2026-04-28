# Pi / GUI start-stop procedure

This runbook covers the day-to-day commands for bringing up and shutting down the Raspberry Pi capture services and the host GUI.

## Targets

Current Pi inventory is defined in `src/deploy/hosts.ini`.

| camera_id | IP | Role |
| --- | --- | --- |
| `pi-cam-01` | `192.168.8.223` | PTP grandmaster / capture node |
| `pi-cam-02` | `192.168.8.100` | PTP client / capture node |

Run host-side commands from the repository root:

```bash
cd /Users/loutlot/Documents/cursor/MOCAP/Loutrack2
```

## Start Pi Capture Services

Preferred path, when the Pis have been deployed with `src/deploy/deploy.sh`, is to run the systemd service on each Pi:

```bash
ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.223 \
  'sudo systemctl restart loutrack.service'

ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.100 \
  'sudo systemctl restart loutrack.service'
```

The service entrypoint is:

```bash
/usr/bin/python3 /opt/loutrack/current/src/pi/service/capture_runtime.py \
  --camera-id <camera_id> \
  --udp-dest 255.255.255.255:5000
```

Use this manual fallback only when systemd is unavailable:

```bash
ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.223 \
  'cd /opt/loutrack/current && nohup /usr/bin/python3 src/pi/service/capture_runtime.py --camera-id pi-cam-01 --udp-dest 255.255.255.255:5000 >/tmp/loutrack_capture_pi-cam-01.log 2>&1 </dev/null &'

ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.100 \
  'cd /opt/loutrack/current && nohup /usr/bin/python3 src/pi/service/capture_runtime.py --camera-id pi-cam-02 --udp-dest 255.255.255.255:5000 >/tmp/loutrack_capture_pi-cam-02.log 2>&1 </dev/null &'
```

## Verify Pi Services

Check the control server and PTP/runtime diagnostics:

```bash
PYTHONPATH=src .venv/bin/python -m host.control \
  --ip 192.168.8.223 \
  --port 8554 \
  --camera-id pi-cam-01 \
  ping

PYTHONPATH=src .venv/bin/python -m host.control \
  --ip 192.168.8.100 \
  --port 8554 \
  --camera-id pi-cam-02 \
  ping
```

Healthy baseline:

- response has `"ack": true`
- `result.state` is usually `IDLE` until the GUI starts capture/tracking
- `result.clock_sync.status` is `locked`
- `result.runtime.backend_active` is `true`

Pi logs:

```bash
ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.223 \
  'journalctl -u loutrack.service -n 120 --no-pager'

ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.100 \
  'journalctl -u loutrack.service -n 120 --no-pager'
```

## Start Host GUI

Foreground mode is best while debugging because logs stay in the terminal:

```bash
.venv/bin/python src/host/loutrack_gui.py \
  --host 127.0.0.1 \
  --port 8765 \
  --udp-port 5000
```

Open:

```text
http://127.0.0.1:8765/
```

Background mode:

```bash
nohup .venv/bin/python src/host/loutrack_gui.py \
  --host 127.0.0.1 \
  --port 8765 \
  --udp-port 5000 \
  >/tmp/loutrack_gui.log 2>&1 </dev/null &
```

Verify the GUI API:

```bash
curl -sS --max-time 3 http://127.0.0.1:8765/api/state
```

GUI log:

```bash
tail -f /tmp/loutrack_gui.log
```

## Stop Streaming But Keep Pi Runtime Up

Use this when capture/tracking is active and you want the Pi service to stay reachable:

```bash
PYTHONPATH=src .venv/bin/python -m host.control \
  --ip 192.168.8.223 \
  --port 8554 \
  --camera-id pi-cam-01 \
  stop

PYTHONPATH=src .venv/bin/python -m host.control \
  --ip 192.168.8.100 \
  --port 8554 \
  --camera-id pi-cam-02 \
  stop
```

After `stop`, `ping` should show `result.state` returning to `IDLE`.

## Stop Host GUI

Foreground mode:

```text
Press Ctrl-C in the terminal running loutrack_gui.py.
```

Background mode:

```bash
pkill -f 'src/host/loutrack_gui.py'
```

Confirm the HTTP server is down:

```bash
curl -sS --max-time 2 http://127.0.0.1:8765/api/state
```

Expected result is a connection failure.

## Stop Pi Capture Services

Use this when you want the Pi control server itself down:

```bash
ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.223 \
  'sudo systemctl stop loutrack.service'

ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.100 \
  'sudo systemctl stop loutrack.service'
```

Confirm the control port is closed or no longer acknowledges:

```bash
PYTHONPATH=src .venv/bin/python -m host.control \
  --ip 192.168.8.223 \
  --port 8554 \
  --camera-id pi-cam-01 \
  ping

PYTHONPATH=src .venv/bin/python -m host.control \
  --ip 192.168.8.100 \
  --port 8554 \
  --camera-id pi-cam-02 \
  ping
```

Expected result is a timeout or connection failure.

## Manage Pi Services From The GUI

The Camera Status page includes Pi Admin actions for selected cameras after each Pi has been installed and can be reached over SSH.

The GUI uses the default deployment key named `loutrack_deploy_key`, SSH user `pi`, and the inventory/discovery camera targets. It runs service management over SSH without requiring host-side shell scripts, which keeps the path usable from Windows packages.

Available actions:

- `Refresh Admin Status`: checks SSH reachability, `loutrack.service`, PTP service state, uptime, CPU temperature, disk usage, current release, and recent service logs.
- `Start Service`: refreshes the `loutrack.service` unit with TCP `8554`, UDP `5000`, and MJPEG `8555` settings, then restarts it.
- `Stop Service`: stops `loutrack.service`.
- `Update`: uploads `src/pi` and `src/camera-calibration`, switches the Loutrack remote `current` release, updates the service file, and restarts the service.
- `Rollback`: switches to the previous Loutrack remote release and restarts the service.
- `Reboot`: reboots the selected Pi cameras.
- `Shutdown`: powers off the selected Pi cameras.

The Pi user must be able to run the required commands with non-interactive sudo. A typical sudoers entry is:

Allow `systemctl`, `shutdown`, `reboot`, `tee`, `mkdir`, and `chown` for the Pi management user.

## Troubleshooting

- If GUI startup fails with UDP bind errors on port `5000`, stop the old GUI/tracking process first, then start the GUI again.
- If a Pi `ping` works but tracking does not receive frames, make sure the GUI host is on the same LAN/broadcast segment as the Pis and that the GUI is listening on UDP `5000`.
- If `clock_sync.status` is not `locked`, inspect PTP before capture:

```bash
ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.223 \
  'systemctl status loutrack-ptp4l.service --no-pager'

ssh -i ~/.ssh/loutrack_deploy_key -o BatchMode=yes -o ConnectTimeout=5 pi@192.168.8.100 \
  'systemctl status loutrack-ptp4l.service --no-pager'
```
