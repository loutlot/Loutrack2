#!/usr/bin/env bash
set -eu

PTP4L_UNIT="loutrack-ptp4l.service"
PHC2SYS_UNIT="loutrack-phc2sys.service"
TIMESYNCD_UNIT="systemd-timesyncd.service"
NTP_BOOTSTRAP_UNIT="loutrack-ntp-bootstrap.service"
NTP_BOOTSTRAP_SCRIPT="/usr/local/bin/loutrack-ntp-bootstrap.sh"

if [ "$(id -u)" -ne 0 ]; then
  echo "error: run as root, e.g. sudo ./src/pi/revert_ptp.sh" >&2
  exit 2
fi

if ! command -v systemctl >/dev/null 2>&1; then
  echo "error: systemctl is required" >&2
  exit 1
fi

if ! command -v timedatectl >/dev/null 2>&1; then
  echo "error: timedatectl is required" >&2
  exit 1
fi

echo "[revert_ptp] stopping Loutrack PTP services"
systemctl stop "${PHC2SYS_UNIT}" >/dev/null 2>&1 || true
systemctl stop "${PTP4L_UNIT}" >/dev/null 2>&1 || true

echo "[revert_ptp] disabling Loutrack PTP services"
systemctl disable "${NTP_BOOTSTRAP_UNIT}" >/dev/null 2>&1 || true
systemctl disable "${PHC2SYS_UNIT}" >/dev/null 2>&1 || true
systemctl disable "${PTP4L_UNIT}" >/dev/null 2>&1 || true

echo "[revert_ptp] removing Loutrack systemd units"
rm -f \
  "/etc/systemd/system/${NTP_BOOTSTRAP_UNIT}" \
  "/etc/systemd/system/${PTP4L_UNIT}" \
  "/etc/systemd/system/${PHC2SYS_UNIT}"

echo "[revert_ptp] removing Loutrack linuxptp config files"
rm -f \
  /etc/linuxptp/loutrack-ptp.conf \
  /etc/linuxptp/loutrack-role \
  /etc/linuxptp/loutrack-timestamping-mode
rm -f "${NTP_BOOTSTRAP_SCRIPT}"

systemctl daemon-reload
systemctl reset-failed "${NTP_BOOTSTRAP_UNIT}" >/dev/null 2>&1 || true
systemctl reset-failed "${PTP4L_UNIT}" >/dev/null 2>&1 || true
systemctl reset-failed "${PHC2SYS_UNIT}" >/dev/null 2>&1 || true

if systemctl list-unit-files "${TIMESYNCD_UNIT}" >/dev/null 2>&1; then
  echo "[revert_ptp] re-enabling ${TIMESYNCD_UNIT}"
  systemctl unmask "${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true
  systemctl enable --now "${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true
  timedatectl set-ntp true >/dev/null 2>&1 || true
fi

echo "[revert_ptp] done"
echo "verify with:"
echo "  systemctl status ${TIMESYNCD_UNIT}"
echo "  timedatectl status"
