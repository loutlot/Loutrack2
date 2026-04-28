#!/usr/bin/env bash

# loutrack deployment script
# Uses rsync over SSH to copy ./pi/ to Raspberry Pi devices,
# creates atomic release via symlink, restarts loutrack.service,
# retains last 3 releases, supports dry-run and parallel execution.

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SSH_KEY="$HOME/.ssh/loutrack_deploy_key"
REMOTE_USER="pi"
REMOTE_BASE="/opt/loutrack"
RELEASES_DIR="$REMOTE_BASE/releases"
CURRENT_LINK="$REMOTE_BASE/current"
SERVICE_NAME="loutrack.service"
TCP_HOST="0.0.0.0"
TCP_PORT="8554"
UDP_DEST="255.255.255.255:5000"
MJPEG_PORT="8555"
LOCAL_SRC="$SRC_ROOT/pi/"
LOCAL_CALIB_SRC="$SRC_ROOT/camera-calibration/"
HOSTS_FILE="$SCRIPT_DIR/hosts.ini"
LOG_FILE="$SCRIPT_DIR/deploy.log"
REMOTE_SRC_DIR="$CURRENT_LINK/src"

# Default options
DRY_RUN=false
PARALLEL=true
VERSION="$(date +%Y%m%d%H%M%S)"

usage() {
  cat <<EOF
Usage: $0 [options]
Options:
  --dry-run           Show what would be done without making changes.
  --no-parallel       Run hosts sequentially.
  --version <tag>     Release version tag (default: timestamp).
  -h, --help          Show this help.
EOF
  exit 1
}

log() {
  local msg="$(date '+%Y-%m-%d %H:%M:%S') $1"
  echo "$msg" | tee -a "$LOG_FILE"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --dry-run) DRY_RUN=true; shift;;
    --no-parallel) PARALLEL=false; shift;;
    --version) VERSION="$2"; shift 2;;
    -h|--help) usage;;
    *) echo "Unknown option: $1"; usage;;
  esac
done

SSH_OPTS=(-n -i "$SSH_KEY" -o StrictHostKeyChecking=no)
RSYNC_OPTS=(
  -az
  --delete
  --exclude "__pycache__/"
  --exclude "*.pyc"
  -e "ssh -i $SSH_KEY -o StrictHostKeyChecking=no"
)
if $DRY_RUN; then
  RSYNC_OPTS+=(--dry-run)
fi

install_service() {
  local target="$1"
  local camera_id="$2"

  ssh "${SSH_OPTS[@]}" "$target" "sudo tee /etc/systemd/system/$SERVICE_NAME >/dev/null <<EOF
[Unit]
Description=Loutrack camera capture
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$REMOTE_USER
WorkingDirectory=$CURRENT_LINK
Environment=PYTHONUNBUFFERED=1
Environment=PYTHONDONTWRITEBYTECODE=1
ExecStart=/usr/bin/python3 $REMOTE_SRC_DIR/pi/service/capture_runtime.py --camera-id $camera_id --tcp-host $TCP_HOST --tcp-port $TCP_PORT --udp-dest $UDP_DEST --mjpeg-port $MJPEG_PORT --sync-role auto
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME >/dev/null"
}

# Function to deploy to a single host
deploy_host() {
  local host="$1"
  local ip="$2"
  local camera_id="$3"
  local target="${REMOTE_USER}@${ip}"
  local remote_release="$RELEASES_DIR/$VERSION"

  log "Deploying to $host ($ip) as release $VERSION"

  # Create remote release directory
  if $DRY_RUN; then
    log "[dry-run] Would create $remote_release on $host"
  else
    ssh "${SSH_OPTS[@]}" "$target" "sudo mkdir -p \"$remote_release/src/pi\" \"$remote_release/src/camera-calibration\" && sudo chown -R \"$REMOTE_USER:$REMOTE_USER\" \"$REMOTE_BASE\""
  fi

  # Rsync source to remote release directory
  rsync "${RSYNC_OPTS[@]}" "$LOCAL_SRC" "$target:$remote_release/src/pi/"
  rsync "${RSYNC_OPTS[@]}" "$LOCAL_CALIB_SRC" "$target:$remote_release/src/camera-calibration/"

  if $DRY_RUN; then
    log "[dry-run] Would switch current link, install $SERVICE_NAME, and restart on $host"
    return
  fi

  # Atomic symlink switch
  ssh "${SSH_OPTS[@]}" "$target" "ln -sfn \"$remote_release\" \"$CURRENT_LINK\""

  # Install/update systemd service with the host-specific camera_id.
  install_service "$target" "$camera_id"

  # Restart service
  ssh "${SSH_OPTS[@]}" "$target" "sudo systemctl restart $SERVICE_NAME"

  # Cleanup old releases (keep latest 3)
  ssh "${SSH_OPTS[@]}" "$target" "cd \"$RELEASES_DIR\" && ls -1dt */ | tail -n +4 | xargs -r rm -rf"

  log "Deployment to $host completed"
}

# Read hosts file and launch deployments
jobs=()
while read -r line; do
  # Skip empty lines and comments
  [[ -z "$line" || "$line" =~ ^# ]] && continue
  # Expected format: hostname ip_address camera_id
  read -r hostname ip cam <<<"$line"
  if $PARALLEL; then
    deploy_host "$hostname" "$ip" "$cam" &
    jobs+=("$!")
  else
    deploy_host "$hostname" "$ip" "$cam"
  fi
done < "$HOSTS_FILE"

# Wait for background jobs if parallel
if $PARALLEL; then
  for pid in "${jobs[@]}"; do
    wait "$pid"
  done
fi

log "All deployments finished"
