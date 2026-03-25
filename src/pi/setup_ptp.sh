#!/usr/bin/env bash
set -eu

ROLE="${1:-}"
INTERFACE="${2:-eth0}"
TIMESTAMPING_MODE="${3:-software}"
PTP4L_UNIT="loutrack-ptp4l.service"
PHC2SYS_UNIT="loutrack-phc2sys.service"
TIMESYNCD_UNIT="systemd-timesyncd.service"
NTP_BOOTSTRAP_UNIT="loutrack-ntp-bootstrap.service"
NTP_BOOTSTRAP_SCRIPT="/usr/local/bin/loutrack-ntp-bootstrap.sh"

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

cat >"${NTP_BOOTSTRAP_SCRIPT}" <<EOF
#!/usr/bin/env bash
set -eu

TIMESYNCD_UNIT="${TIMESYNCD_UNIT}"
ROLE_PATH="/etc/linuxptp/loutrack-role"

if [ ! -f "\${ROLE_PATH}" ] || ! grep -qi '^master$' "\${ROLE_PATH}"; then
  exit 0
fi

if ! command -v systemctl >/dev/null 2>&1; then
  exit 0
fi

if ! command -v timedatectl >/dev/null 2>&1; then
  exit 0
fi

if ! systemctl list-unit-files "\${TIMESYNCD_UNIT}" >/dev/null 2>&1; then
  exit 0
fi

if timedatectl show --property=NTPSynchronized --value | grep -qi '^yes$'; then
  echo "[loutrack-ntp-bootstrap] ntp already synchronized"
  exit 0
fi

echo "[loutrack-ntp-bootstrap] enabling temporary NTP sync via \${TIMESYNCD_UNIT}"
systemctl unmask "\${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true
systemctl enable --now "\${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true
timedatectl set-ntp true >/dev/null 2>&1 || true

attempt=0
synced="no"
while [ "\${attempt}" -lt 30 ]; do
  attempt=\$((attempt + 1))
  if timedatectl show --property=NTPSynchronized --value | grep -qi '^yes$'; then
    synced="yes"
    break
  fi
  sleep 1
done

echo "[loutrack-ntp-bootstrap] current time: \$(date -Is)"
echo "[loutrack-ntp-bootstrap] ntp synchronized: \${synced}"

echo "[loutrack-ntp-bootstrap] disabling temporary NTP sync"
timedatectl set-ntp false >/dev/null 2>&1 || true
systemctl disable --now "\${TIMESYNCD_UNIT}" >/dev/null 2>&1 || true

exit 0
EOF
chmod 0755 "${NTP_BOOTSTRAP_SCRIPT}"

if [ "${ROLE}" = "master" ]; then
  "${NTP_BOOTSTRAP_SCRIPT}" >/dev/null 2>&1 || true
fi

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
  PTP4L_AFTER="${NTP_BOOTSTRAP_UNIT} network-online.target"
  PTP4L_WANTS="${NTP_BOOTSTRAP_UNIT} network-online.target"
  PTP4L_EXEC="/usr/sbin/ptp4l -f /etc/linuxptp/loutrack-ptp.conf -i ${INTERFACE} -m"
else
  PTP4L_AFTER="network-online.target"
  PTP4L_WANTS="network-online.target"
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
After=${PTP4L_AFTER}
Wants=${PTP4L_WANTS}

[Service]
Type=simple
ExecStartPre=/bin/sleep 5
ExecStart=${PTP4L_EXEC}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

if [ "${ROLE}" = "master" ]; then
  cat >/etc/systemd/system/${NTP_BOOTSTRAP_UNIT} <<EOF
[Unit]
Description=Loutrack master NTP bootstrap
After=network-online.target
Wants=network-online.target
Before=${PTP4L_UNIT}

[Service]
Type=oneshot
ExecStart=${NTP_BOOTSTRAP_SCRIPT}

[Install]
WantedBy=multi-user.target
EOF
else
  systemctl stop "${NTP_BOOTSTRAP_UNIT}" >/dev/null 2>&1 || true
  systemctl disable "${NTP_BOOTSTRAP_UNIT}" >/dev/null 2>&1 || true
  rm -f "/etc/systemd/system/${NTP_BOOTSTRAP_UNIT}"
fi

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
if [ "${ROLE}" = "master" ]; then
  systemctl enable "${NTP_BOOTSTRAP_UNIT}"
  systemctl start "${NTP_BOOTSTRAP_UNIT}" >/dev/null 2>&1 || true
fi
systemctl enable "${PTP4L_UNIT}"
systemctl restart "${PTP4L_UNIT}"
if [ "${TIMESTAMPING_MODE}" = "hardware" ]; then
  systemctl enable "${PHC2SYS_UNIT}"
  systemctl restart "${PHC2SYS_UNIT}"
fi

echo "Configured Loutrack PTP role=${ROLE} interface=${INTERFACE} timestamping=${TIMESTAMPING_MODE}"
echo "Check status with:"
if [ "${ROLE}" = "master" ]; then
  echo "  systemctl status ${NTP_BOOTSTRAP_UNIT}"
fi
echo "  systemctl status ${PTP4L_UNIT}"
if [ "${TIMESTAMPING_MODE}" = "hardware" ]; then
  echo "  systemctl status ${PHC2SYS_UNIT}"
fi
echo "  /usr/sbin/pmc -u -b 0 -s /var/run/ptp4lro -i /tmp/pmc.setup_ptp \"GET TIME_STATUS_NP\""
