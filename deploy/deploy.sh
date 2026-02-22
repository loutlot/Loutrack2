#!/usr/bin/env bash

# loutrack deployment script
# Uses rsync over SSH to copy ./pi/ to Raspberry Pi devices,
# creates atomic release via symlink, restarts loutrack.service,
# retains last 3 releases, supports dry-run and parallel execution.

set -euo pipefail

# Configuration
SSH_KEY="$HOME/.ssh/loutrack_deploy_key"
REMOTE_USER="pi"
REMOTE_BASE="/opt/loutrack"
RELEASES_DIR="$REMOTE_BASE/releases"
CURRENT_LINK="$REMOTE_BASE/current"
SERVICE_NAME="loutrack.service"
LOCAL_SRC="$(pwd)/pi/"
HOSTS_FILE="$(pwd)/deploy/hosts.ini"
LOG_FILE="$(pwd)/deploy/deploy.log"

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

# Build rsync options
RSYNC_OPTS="-az --delete -e \"ssh -i $SSH_KEY -o StrictHostKeyChecking=no\""
if $DRY_RUN; then
  RSYNC_OPTS="$RSYNC_OPTS --dry-run"
fi

# Function to deploy to a single host
deploy_host() {
  local host="$1"
  local ip="$2"
  local camera_id="$3"
  local target="${REMOTE_USER}@${ip}"
  local remote_release="$RELEASES_DIR/$VERSION"

  log "Deploying to $host ($ip) as release $VERSION"

  # Create remote release directory
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "mkdir -p \"$remote_release\""

  # Rsync source to remote release directory
  rsync $RSYNC_OPTS "$LOCAL_SRC" "$target:$remote_release/"

  # Atomic symlink switch
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "ln -sfn \"$remote_release\" \"$CURRENT_LINK\""

  # Restart service
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "sudo systemctl restart $SERVICE_NAME"

  # Cleanup old releases (keep latest 3)
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "cd \"$RELEASES_DIR\" && ls -1dt */ | tail -n +4 | xargs -r rm -rf"

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
