#!/usr/bin/env bash
set -eu

PTP4L_UNIT="loutrack-ptp4l.service"
PHC2SYS_UNIT="loutrack-phc2sys.service"
TIMESYNCD_UNIT="systemd-timesyncd.service"

if [ "$(id -u)" -ne 0 ]; then
  echo "error: run as root, e.g. sudo ./src/pi/manual_ntp_sync.sh" >&2
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

echo "[manual_ntp_sync] stopping Loutrack PTP services"
systemctl stop "${PHC2SYS_UNIT}" >/dev/null 2>&1 || true
systemctl stop "${PTP4L_UNIT}" >/dev/null 2>&1 || true

if ! systemctl list-unit-files "${TIMESYNCD_UNIT}" >/dev/null 2>&1; then
  echo "error: ${TIMESYNCD_UNIT} is not available on this system" >&2
  exit 1
fi

echo "[manual_ntp_sync] enabling temporary NTP sync via ${TIMESYNCD_UNIT}"
systemctl unmask "${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true
systemctl enable --now "${TIMESYNCD_UNIT}" >/dev/null 2>&1
timedatectl set-ntp true

attempt=0
synced="no"
while [ "${attempt}" -lt 30 ]; do
  attempt=$((attempt + 1))
  if timedatectl show --property=NTPSynchronized --value | grep -qi '^yes$'; then
    synced="yes"
    break
  fi
  sleep 1
done

echo "[manual_ntp_sync] current time: $(date -Is)"
echo "[manual_ntp_sync] ntp synchronized: ${synced}"

echo "[manual_ntp_sync] disabling temporary NTP sync"
timedatectl set-ntp false || true
systemctl disable --now "${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true

echo "[manual_ntp_sync] restarting Loutrack PTP services"
if systemctl list-unit-files "${PTP4L_UNIT}" >/dev/null 2>&1; then
  systemctl restart "${PTP4L_UNIT}"
fi
if systemctl list-unit-files "${PHC2SYS_UNIT}" >/dev/null 2>&1; then
  systemctl restart "${PHC2SYS_UNIT}"
fi

echo "[manual_ntp_sync] done"
echo "verify with:"
echo "  systemctl status ${PTP4L_UNIT}"
if systemctl list-unit-files "${PHC2SYS_UNIT}" >/dev/null 2>&1; then
  echo "  systemctl status ${PHC2SYS_UNIT}"
fi
echo "  /usr/sbin/pmc -u -b 0 -s /var/run/ptp4lro -i /tmp/pmc.manual_ntp_sync \"GET TIME_STATUS_NP\""
