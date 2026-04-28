#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_ROOT="${PROJECT_ROOT}/src"

SERVICE_NAME="loutrack.service"
INSTALL_ROOT="/opt/loutrack"
RELEASE_KEEP=3

RUN_USER="${SUDO_USER:-pi}"
if [ "${RUN_USER}" = "root" ] || [ -z "${RUN_USER}" ]; then
  RUN_USER="pi"
fi

CAMERA_ID=""
PTP_ROLE=""
INTERFACE="eth0"
TIMESTAMPING_MODE="software"
UDP_DEST="255.255.255.255:5000"
TCP_PORT="8554"
MJPEG_PORT="8555"
AUTHORIZED_KEY=""
AUTHORIZED_KEY_FILE=""
START_SERVICE=true
CONFIGURE_PTP=true
DEBUG_PREVIEW=false

usage() {
  cat <<EOF
Usage:
  sudo ./src/pi/install_loutrack_node.sh --camera-id <id> --ptp-role <master|slave> [options]

Required:
  --camera-id <id>          Loutrack camera id, e.g. pi-cam-01
  --ptp-role <master|slave> PTP role. Use master for the fixed grandmaster Pi.

Options:
  --user <name>             Runtime/deploy user (default: sudo user, or pi)
  --interface <name>        Network interface for PTP (default: eth0)
  --timestamping <mode>     software|hardware (default: software)
  --udp-dest <host:port>    Host UDP destination (default: 255.255.255.255:5000)
  --tcp-port <port>         Control TCP port (default: 8554)
  --mjpeg-port <port>       MJPEG port, 0 disables it (default: 8555)
  --authorized-key <key>    SSH public key to append for the runtime user
  --authorized-key-file <p> File containing SSH public key(s) to append
  --no-ptp                  Skip Loutrack PTP setup
  --no-start                Install service but do not start it
  --debug-preview           Enable local OpenCV debug preview in the service
  -h, --help                Show this help
EOF
}

log() {
  printf '[loutrack-install] %s\n' "$*"
}

fail() {
  printf '[loutrack-install] error: %s\n' "$*" >&2
  exit 1
}

require_root() {
  if [ "$(id -u)" -ne 0 ]; then
    fail "run as root, e.g. sudo ./src/pi/install_loutrack_node.sh --camera-id pi-cam-01 --ptp-role master"
  fi
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --camera-id)
      CAMERA_ID="${2:-}"; shift 2;;
    --ptp-role)
      PTP_ROLE="${2:-}"; shift 2;;
    --user)
      RUN_USER="${2:-}"; shift 2;;
    --interface)
      INTERFACE="${2:-}"; shift 2;;
    --timestamping)
      TIMESTAMPING_MODE="${2:-}"; shift 2;;
    --udp-dest)
      UDP_DEST="${2:-}"; shift 2;;
    --tcp-port)
      TCP_PORT="${2:-}"; shift 2;;
    --mjpeg-port)
      MJPEG_PORT="${2:-}"; shift 2;;
    --authorized-key)
      AUTHORIZED_KEY="${2:-}"; shift 2;;
    --authorized-key-file)
      AUTHORIZED_KEY_FILE="${2:-}"; shift 2;;
    --no-ptp)
      CONFIGURE_PTP=false; shift;;
    --no-start)
      START_SERVICE=false; shift;;
    --debug-preview)
      DEBUG_PREVIEW=true; shift;;
    -h|--help)
      usage; exit 0;;
    *)
      fail "unknown option: $1";;
  esac
done

require_root

[ -n "${CAMERA_ID}" ] || fail "--camera-id is required"
[ -n "${RUN_USER}" ] || fail "--user must not be empty"
if "${CONFIGURE_PTP}"; then
  [ -n "${PTP_ROLE}" ] || fail "--ptp-role is required unless --no-ptp is used"
  [ "${PTP_ROLE}" = "master" ] || [ "${PTP_ROLE}" = "slave" ] || fail "--ptp-role must be master or slave"
fi
[ "${TIMESTAMPING_MODE}" = "software" ] || [ "${TIMESTAMPING_MODE}" = "hardware" ] || fail "--timestamping must be software or hardware"
id "${RUN_USER}" >/dev/null 2>&1 || fail "user does not exist: ${RUN_USER}"
[ -d "${SRC_ROOT}/pi" ] || fail "missing source directory: src/pi"
[ -d "${SRC_ROOT}/camera-calibration" ] || fail "missing source directory: src/camera-calibration"

log "installing apt dependencies"
apt-get update
apt-get install -y \
  git \
  linuxptp \
  python3-numpy \
  python3-opencv \
  python3-picamera2 \
  python3-scipy \
  rsync \
  sudo

log "checking camera stack"
if ! python3 -c "from picamera2 import Picamera2; print('picamera2 ok')" >/dev/null 2>&1; then
  log "warning: picamera2 import failed; check camera package and OS image"
fi
if ! command -v rpicam-hello >/dev/null 2>&1 && ! command -v libcamera-hello >/dev/null 2>&1; then
  log "warning: rpicam-hello/libcamera-hello not found"
fi

log "creating Loutrack release layout"
RELEASE_NAME="bootstrap-$(date +%Y%m%d%H%M%S)"
RELEASE_DIR="${INSTALL_ROOT}/releases/${RELEASE_NAME}"
install -d "${RELEASE_DIR}/src/pi" "${RELEASE_DIR}/src/camera-calibration"
chown -R "${RUN_USER}:${RUN_USER}" "${INSTALL_ROOT}"

rsync -a --delete \
  --exclude "__pycache__/" \
  --exclude "*.pyc" \
  "${SRC_ROOT}/pi/" \
  "${RELEASE_DIR}/src/pi/"
rsync -a --delete \
  --exclude "__pycache__/" \
  --exclude "*.pyc" \
  "${SRC_ROOT}/camera-calibration/" \
  "${RELEASE_DIR}/src/camera-calibration/"
chown -R "${RUN_USER}:${RUN_USER}" "${RELEASE_DIR}"
ln -sfn "${RELEASE_DIR}" "${INSTALL_ROOT}/current"

if [ -n "${AUTHORIZED_KEY}" ] || [ -n "${AUTHORIZED_KEY_FILE}" ]; then
  log "installing SSH authorized key for ${RUN_USER}"
  USER_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
  install -d -m 0700 -o "${RUN_USER}" -g "${RUN_USER}" "${USER_HOME}/.ssh"
  AUTH_KEYS="${USER_HOME}/.ssh/authorized_keys"
  touch "${AUTH_KEYS}"
  chmod 0600 "${AUTH_KEYS}"
  chown "${RUN_USER}:${RUN_USER}" "${AUTH_KEYS}"
  if [ -n "${AUTHORIZED_KEY}" ] && ! grep -qxF "${AUTHORIZED_KEY}" "${AUTH_KEYS}"; then
    printf '%s\n' "${AUTHORIZED_KEY}" >>"${AUTH_KEYS}"
  fi
  if [ -n "${AUTHORIZED_KEY_FILE}" ]; then
    [ -f "${AUTHORIZED_KEY_FILE}" ] || fail "authorized key file not found: ${AUTHORIZED_KEY_FILE}"
    while IFS= read -r key_line; do
      [ -n "${key_line}" ] || continue
      grep -qxF "${key_line}" "${AUTH_KEYS}" || printf '%s\n' "${key_line}" >>"${AUTH_KEYS}"
    done <"${AUTHORIZED_KEY_FILE}"
  fi
fi

if "${CONFIGURE_PTP}"; then
  log "configuring PTP role=${PTP_ROLE} interface=${INTERFACE} timestamping=${TIMESTAMPING_MODE}"
  "${SCRIPT_DIR}/setup_ptp.sh" "${PTP_ROLE}" "${INTERFACE}" "${TIMESTAMPING_MODE}"
fi

log "installing ${SERVICE_NAME}"
SERVICE_ARGS=(
  "/usr/bin/python3"
  "${INSTALL_ROOT}/current/src/pi/service/capture_runtime.py"
  "--camera-id" "${CAMERA_ID}"
  "--tcp-host" "0.0.0.0"
  "--tcp-port" "${TCP_PORT}"
  "--udp-dest" "${UDP_DEST}"
  "--mjpeg-port" "${MJPEG_PORT}"
  "--sync-role" "auto"
)
if "${DEBUG_PREVIEW}"; then
  SERVICE_ARGS+=("--debug-preview")
fi

EXEC_START=""
for arg in "${SERVICE_ARGS[@]}"; do
  EXEC_START+=" $(printf '%q' "${arg}")"
done
EXEC_START="${EXEC_START# }"

cat >"/etc/systemd/system/${SERVICE_NAME}" <<EOF
[Unit]
Description=Loutrack camera capture
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${INSTALL_ROOT}/current
Environment=PYTHONUNBUFFERED=1
Environment=PYTHONDONTWRITEBYTECODE=1
ExecStart=${EXEC_START}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

log "installing sudoers for GUI Pi Admin"
SUDOERS_TMP="$(mktemp)"
cat >"${SUDOERS_TMP}" <<EOF
Cmnd_Alias LOUTRACK_ADMIN = /usr/bin/systemctl *, /usr/sbin/shutdown *, /usr/sbin/reboot, /usr/sbin/reboot *, /usr/bin/tee /etc/systemd/system/${SERVICE_NAME}, /usr/bin/mkdir *, /usr/bin/chown *
${RUN_USER} ALL=(ALL) NOPASSWD: LOUTRACK_ADMIN
EOF
visudo -cf "${SUDOERS_TMP}" >/dev/null
install -m 0440 "${SUDOERS_TMP}" /etc/sudoers.d/loutrack
rm -f "${SUDOERS_TMP}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}" >/dev/null
if "${START_SERVICE}"; then
  log "starting ${SERVICE_NAME}"
  systemctl restart "${SERVICE_NAME}"
fi

log "cleaning old bootstrap releases"
if [ -d "${INSTALL_ROOT}/releases" ]; then
  (cd "${INSTALL_ROOT}/releases" && ls -1dt */ 2>/dev/null | tail -n +"$((RELEASE_KEEP + 1))" | xargs -r rm -rf)
fi

log "installation complete"
echo "camera_id=${CAMERA_ID}"
echo "release=${RELEASE_NAME}"
echo "service=${SERVICE_NAME}"
echo "ptp_role=${PTP_ROLE:-skipped}"
echo
echo "Next checks:"
echo "  systemctl status ${SERVICE_NAME}"
if "${CONFIGURE_PTP}"; then
  echo "  systemctl status loutrack-ptp4l.service"
fi
echo "  journalctl -u ${SERVICE_NAME} -n 80 --no-pager"
