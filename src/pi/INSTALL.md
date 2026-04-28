# Loutrack Pi first install

This guide is for a user who has never set up Loutrack before. It explains how to prepare a fresh Raspberry Pi OS install, choose names, install the Loutrack camera service, and make the Pi appear in the Loutrack GUI.

## What You Need

- Raspberry Pi camera node hardware with the camera connected
- Raspberry Pi OS Lite 64-bit
- Wired Ethernet is strongly recommended for PTP sync
- A host PC on the same LAN running the Loutrack GUI
- This repository copied or cloned onto each Pi

## Naming Rules

Use the same value for the Pi hostname and Loutrack camera id whenever possible.

Recommended names:

| Role | Hostname | Camera ID | PTP role |
| --- | --- | --- | --- |
| First / clock source Pi | `pi-cam-01` | `pi-cam-01` | `master` |
| Second Pi | `pi-cam-02` | `pi-cam-02` | `slave` |
| Third Pi | `pi-cam-03` | `pi-cam-03` | `slave` |

For now, use the Linux username `pi`. The GUI Pi Admin path defaults to SSH user `pi`, so choosing this username keeps first setup simple.

## Raspberry Pi Imager Settings

In Raspberry Pi Imager, choose Raspberry Pi OS Lite 64-bit and open OS customization.

Set:

- Hostname: `pi-cam-01`, `pi-cam-02`, etc.
- Username: `pi`
- Password: choose a local admin password
- SSH: enabled
- SSH authentication: public key is preferred
- Locale/timezone: your local settings
- Network: wired Ethernet preferred; set Wi-Fi only if needed

If you use SSH keys, paste the public key for the host PC that will run the Loutrack GUI. This allows GUI Pi Admin actions to connect later.

Write the image, insert it into the Pi, connect camera and network, then boot.

## Get The Repository Onto The Pi

Log into the Pi with SSH:

```bash
ssh pi@pi-cam-01.local
```

Then clone or copy this repository onto the Pi. Enter the repository root before running the installer:

```bash
cd Loutrack2
```

If you copied a zip instead of using git, make sure the directory still contains `src/pi` and `src/camera-calibration`.

## Install The Master Pi

Run this on `pi-cam-01`:

```bash
sudo ./src/pi/install_loutrack_node.sh \
  --camera-id pi-cam-01 \
  --ptp-role master
```

The master Pi briefly uses NTP to set wall-clock time, then becomes the Loutrack PTP grandmaster.

## Install Slave Pis

Run this on every other Pi, changing the camera id:

```bash
sudo ./src/pi/install_loutrack_node.sh \
  --camera-id pi-cam-02 \
  --ptp-role slave
```

For a third Pi:

```bash
sudo ./src/pi/install_loutrack_node.sh \
  --camera-id pi-cam-03 \
  --ptp-role slave
```

## Optional Installer Flags

Common options:

```bash
sudo ./src/pi/install_loutrack_node.sh \
  --camera-id pi-cam-02 \
  --ptp-role slave \
  --interface eth0 \
  --timestamping software \
  --udp-dest 255.255.255.255:5000
```

Use `--user <name>` only if you intentionally did not create the `pi` user. If you do that, GUI Pi Admin must also be configured to use the same SSH user before management actions will work.

Use `--authorized-key-file <path>` or `--authorized-key "<public key>"` if you did not add the GUI host key in Raspberry Pi Imager.

Use `--no-start` when you want to install the service but start it manually later.

## What The Installer Does

The installer:

- Installs Pi runtime packages: `linuxptp`, `python3-picamera2`, `python3-opencv`, `python3-numpy`, `python3-scipy`, `rsync`, and related basics
- Copies `src/pi` and `src/camera-calibration` into the Loutrack release layout
- Calls `src/pi/setup_ptp.sh` with the selected `master` or `slave` role
- Creates and enables `loutrack.service`
- Adds non-interactive sudo permissions needed by GUI Pi Admin
- Starts the service unless `--no-start` was used

## Verify On The Pi

Check the capture service:

```bash
systemctl status loutrack.service
journalctl -u loutrack.service -n 80 --no-pager
```

Check PTP:

```bash
systemctl status loutrack-ptp4l.service
```

Check the camera Python stack:

```bash
python3 -c "from picamera2 import Picamera2; print('picamera2 ok')"
```

## Make The Pi Appear In The GUI

Start the Loutrack GUI on the host PC, then open Camera Status.

If the Pi service is running and UDP broadcast reaches the host, cameras should appear automatically. If not, add or update `src/deploy/hosts.ini` on the host PC:

```ini
pi-cam-01 192.168.8.223 pi-cam-01
pi-cam-02 192.168.8.100 pi-cam-02
```

Then press Refresh in the GUI.

The GUI Pi Admin buttons use SSH. They expect:

- The Pi is reachable on the LAN
- SSH is enabled
- The GUI host public key is authorized for user `pi`
- The installer has completed sudoers setup

## Troubleshooting

If the GUI shows `No Ack`, check:

```bash
systemctl status loutrack.service
journalctl -u loutrack.service -n 80 --no-pager
```

If the camera backend is unavailable, check the ribbon cable and run:

```bash
python3 -c "from picamera2 import Picamera2; p=Picamera2(); print('created')"
```

If PTP does not lock, make sure:

- `pi-cam-01` was installed as `master`
- all other Pi cameras were installed as `slave`
- all Pi cameras are on the same wired Ethernet segment
- no other time sync service is fighting Loutrack PTP

If SSH admin actions fail from the GUI, check that the host PC key is present for the `pi` user and that the installer was run successfully.

