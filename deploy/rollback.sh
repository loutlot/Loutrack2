#!/usr/bin/env bash

# loutrack rollback script
# Switches each Pi to the previous release (if available) and restarts the service.
# Uses the same SSH key and user as deploy.sh.

set -euo pipefail

# Configuration (must match deploy.sh)
SSH_KEY="$HOME/.ssh/loutrack_deploy_key"
REMOTE_USER="pi"
REMOTE_BASE="/opt/loutrack"
RELEASES_DIR="$REMOTE_BASE/releases"
CURRENT_LINK="$REMOTE_BASE/current"
SERVICE_NAME="loutrack.service"
HOSTS_FILE="$(pwd)/deploy/hosts.ini"
LOG_FILE="$(pwd)/deploy/rollback.log"

# Default options
DRY_RUN=false
PARALLEL=true

usage() {
  cat <<EOF
Usage: $0 [options]
Options:
  --dry-run           Show actions without making changes.
  --no-parallel       Run hosts sequentially.
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
    -h|--help) usage;;
    *) echo "Unknown option: $1"; usage;;
  esac
done

rollback_host() {
  local host="$1"
  local ip="$2"
  local target="${REMOTE_USER}@${ip}"

  log "Rolling back $host ($ip)"

  # Determine previous release (second newest)
  local releases=$(ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "ls -1dt $RELEASES_DIR/*/" 2>/dev/null || true)
  if [[ -z "$releases" ]]; then
    log "No releases found on $host; skipping"
    return
  fi
  # Convert to array
  IFS=$'\n' read -r -d '' -a rev_array <<<"$releases"
  if (( ${#rev_array[@]} < 2 )); then
    log "Only one release present on $host; cannot rollback"
    return
  fi
  local previous_release="${rev_array[1]}"

  if $DRY_RUN; then
    log "[dry-run] Would set symlink $CURRENT_LINK -> $previous_release on $host"
    log "[dry-run] Would restart $SERVICE_NAME on $host"
    return
  fi

  # Switch symlink atomically
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "ln -sfn \"$previous_release\" \"$CURRENT_LINK\""
  # Restart service
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$target" "sudo systemctl restart $SERVICE_NAME"

  log "Rollback on $host completed"
}

jobs=()
while read -r line; do
  [[ -z "$line" || "$line" =~ ^# ]] && continue
  read -r hostname ip _ <<<"$line"
  if $PARALLEL; then
    rollback_host "$hostname" "$ip" &
    jobs+=("$!")
  else
    rollback_host "$hostname" "$ip"
  fi
done < "$HOSTS_FILE"

if $PARALLEL; then
  for pid in "${jobs[@]}"; do
    wait "$pid"
  done
fi

log "All rollbacks finished"
