#!/usr/bin/env bash
set -eu

ROLE="${1:-}"
INTERFACE="${2:-eth0}"
TIMESTAMPING_MODE="${3:-software}"
PTP4L_UNIT="loutrack-ptp4l.service"
PHC2SYS_UNIT="loutrack-phc2sys.service"
TIMESYNCD_UNIT="systemd-timesyncd.service"

if [ -z "${ROLE}" ]; then
  echo "usage: sudo ./src/pi/setup_ptp.sh <master|slave> [interface] [software|hardware]" >&2
  exit 2
fi

if [ "${ROLE}" != "master" ] && [ "${ROLE}" != "slave" ]; then
  echo "error: role must be 'master' or 'slave'" >&2
  exit 2
fi

if [ "${TIMESTAMPING_MODE}" != "software" ] && [ "${TIMESTAMPING_MODE}" != "hardware" ]; then
  echo "error: timestamping mode must be 'software' or 'hardware'" >&2
  exit 2
fi

if [ "$(id -u)" -ne 0 ]; then
  echo "error: run as root, e.g. sudo ./src/pi/setup_ptp.sh ${ROLE} ${INTERFACE} ${TIMESTAMPING_MODE}" >&2
  exit 2
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "error: apt-get is required" >&2
  exit 1
fi

if ! command -v systemctl >/dev/null 2>&1; then
  echo "error: systemctl is required" >&2
  exit 1
fi

if ! command -v timedatectl >/dev/null 2>&1; then
  echo "error: timedatectl is required" >&2
  exit 1
fi

install -d /etc/linuxptp

apt-get update
apt-get install -y linuxptp

if systemctl list-unit-files "${TIMESYNCD_UNIT}" >/dev/null 2>&1; then
  timedatectl set-ntp false >/dev/null 2>&1 || true
  systemctl disable --now "${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true
fi

cat >/etc/linuxptp/loutrack-ptp.conf <<EOF
[global]
time_stamping ${TIMESTAMPING_MODE}
uds_ro_address /var/run/ptp4lro
uds_ro_file_mode 0666
EOF

if [ "${ROLE}" = "master" ]; then
  cat >>/etc/linuxptp/loutrack-ptp.conf <<EOF
masterOnly 1
priority1 128
EOF
fi

cat >/etc/linuxptp/loutrack-role <<EOF
${ROLE}
EOF

cat >/etc/linuxptp/loutrack-timestamping-mode <<EOF
${TIMESTAMPING_MODE}
EOF

if [ "${ROLE}" = "master" ]; then
  PTP4L_EXEC="/usr/sbin/ptp4l -f /etc/linuxptp/loutrack-ptp.conf -i ${INTERFACE} -m"
else
  PTP4L_EXEC="/usr/sbin/ptp4l -f /etc/linuxptp/loutrack-ptp.conf -i ${INTERFACE} -s -m --summary_interval 1"
fi

if [ "${TIMESTAMPING_MODE}" = "hardware" ]; then
  if [ "${ROLE}" = "master" ]; then
    PHC2SYS_EXEC="/usr/sbin/phc2sys -s CLOCK_REALTIME -c ${INTERFACE} -O 0 --step_threshold 0.5 -w -m"
  else
    PHC2SYS_EXEC="/usr/sbin/phc2sys -s ${INTERFACE} -c CLOCK_REALTIME -O 0 --step_threshold 0.5 -w -m"
  fi
fi

cat >/etc/systemd/system/${PTP4L_UNIT} <<EOF
[Unit]
Description=Loutrack PTP4L (${ROLE})
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStartPre=/bin/sleep 5
ExecStart=${PTP4L_EXEC}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

if [ "${TIMESTAMPING_MODE}" = "hardware" ]; then
  cat >/etc/systemd/system/${PHC2SYS_UNIT} <<EOF
[Unit]
Description=Loutrack PHC2SYS (${ROLE})
After=${PTP4L_UNIT}
Requires=${PTP4L_UNIT}

[Service]
Type=simple
ExecStart=${PHC2SYS_EXEC}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
else
  systemctl stop "${PHC2SYS_UNIT}" >/dev/null 2>&1 || true
  systemctl disable "${PHC2SYS_UNIT}" >/dev/null 2>&1 || true
  rm -f "/etc/systemd/system/${PHC2SYS_UNIT}"
fi

systemctl daemon-reload
systemctl enable "${PTP4L_UNIT}"
systemctl restart "${PTP4L_UNIT}"
if [ "${TIMESTAMPING_MODE}" = "hardware" ]; then
  systemctl enable "${PHC2SYS_UNIT}"
  systemctl restart "${PHC2SYS_UNIT}"
fi

echo "Configured Loutrack PTP role=${ROLE} interface=${INTERFACE} timestamping=${TIMESTAMPING_MODE}"
echo "Check status with:"
echo "  systemctl status ${PTP4L_UNIT}"
if [ "${TIMESTAMPING_MODE}" = "hardware" ]; then
  echo "  systemctl status ${PHC2SYS_UNIT}"
fi
echo "  pmc -u -b 0 \"GET TIME_STATUS_NP\""
