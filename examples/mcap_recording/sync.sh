#!/usr/bin/env bash
# Post-hoc sync of a finished MCAP recording to a remote host (SAD §10
# "本地记录 + 事后同步"). The recording is authoritative on local disk; this
# copies it off-box and verifies the copy with sha256sum so a dropped or
# truncated transfer is caught.
#
# Usage:
#   AVIATOR_REMOTE_HOST=user@host AVIATOR_REMOTE_DIR=/data/aviator \
#       ./sync.sh recording.mcap
set -euo pipefail

FILE="${1:-recording.mcap}"
REMOTE_HOST="${AVIATOR_REMOTE_HOST:-user@example.com}"
REMOTE_DIR="${AVIATOR_REMOTE_DIR:-/data/aviator}"
REMOTE_FILE="$REMOTE_DIR/$(basename "$FILE")"

local_sum="$(sha256sum "$FILE" | cut -d' ' -f1)"
rsync -av --checksum "$FILE" "$REMOTE_HOST:$REMOTE_DIR/"

remote_sum="$(ssh "$REMOTE_HOST" "sha256sum '$REMOTE_FILE'" | cut -d' ' -f1)"
echo "local  sha256 = $local_sum"
echo "remote sha256 = $remote_sum"

if [ "$local_sum" = "$remote_sum" ]; then
    echo "OK: checksums match"
else
    echo "MISMATCH: remote copy differs from local"
    exit 1
fi
