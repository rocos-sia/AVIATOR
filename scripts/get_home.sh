#!/usr/bin/env bash
# Capture the measured arm joints as home in the local posture configuration.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/runtime_env.sh"
exec "${UI_PYTHON:-python3}" "$SCRIPT_DIR/get_home.py" "$@"
