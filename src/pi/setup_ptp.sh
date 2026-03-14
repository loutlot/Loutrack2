#!/usr/bin/env bash
set -eu

ROLE="${1:-}"
INTERFACE="${2:-eth0}"

if [ -z "${ROLE}" ]; then
  echo "usage: sudo ./src/pi/setup_ptp.sh <master|slave> [interface]" >&2
  exit 2
fi

if [ "${ROLE}" != "master" ] && [ "${ROLE}" != "slave" ]; then
  echo "error: role must be 'master' or 'slave'" >&2
  exit 2
fi

if [ "$(id -u)" -ne 0 ]; then
  echo "error: run as root, e.g. sudo ./src/pi/setup_ptp.sh ${ROLE} ${INTERFACE}" >&2
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

install -d /etc/linuxptp

apt-get update
apt-get install -y linuxptp

if systemctl list-unit-files systemd-timesyncd.service >/dev/null 2>&1; then
  systemctl disable --now systemd-timesyncd.service >/dev/null 2>&1 || true
fi

cat >/etc/linuxptp/loutrack-ptp.conf <<EOF
[global]
time_stamping hardware
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

if [ "${ROLE}" = "master" ]; then
  PTP4L_EXEC="/usr/sbin/ptp4l -f /etc/linuxptp/loutrack-ptp.conf -i ${INTERFACE} -m"
  PHC2SYS_EXEC="/usr/sbin/phc2sys -s CLOCK_REALTIME -c ${INTERFACE} -O 0 --step_threshold 0.5 -w -m"
else
  PTP4L_EXEC="/usr/sbin/ptp4l -f /etc/linuxptp/loutrack-ptp.conf -i ${INTERFACE} -s -m --summary_interval 1"
  PHC2SYS_EXEC="/usr/sbin/phc2sys -s ${INTERFACE} -c CLOCK_REALTIME -O 0 --step_threshold 0.5 -w -m"
fi

cat >/etc/systemd/system/loutrack-ptp4l.service <<EOF
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

cat >/etc/systemd/system/loutrack-phc2sys.service <<EOF
[Unit]
Description=Loutrack PHC2SYS (${ROLE})
After=loutrack-ptp4l.service
Requires=loutrack-ptp4l.service

[Service]
Type=simple
ExecStart=${PHC2SYS_EXEC}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable loutrack-ptp4l.service loutrack-phc2sys.service
systemctl restart loutrack-ptp4l.service
systemctl restart loutrack-phc2sys.service

echo "Configured Loutrack PTP role=${ROLE} interface=${INTERFACE}"
echo "Check status with:"
echo "  systemctl status loutrack-ptp4l.service"
echo "  systemctl status loutrack-phc2sys.service"
echo "  pmc -u -b 0 \"GET TIME_STATUS_NP\""
