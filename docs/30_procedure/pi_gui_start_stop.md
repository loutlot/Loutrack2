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

