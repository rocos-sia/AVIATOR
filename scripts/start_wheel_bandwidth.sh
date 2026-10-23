#!/usr/bin/env bash
set -Eeuo pipefail
if [[ -f $(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh ]]; then
    source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
fi
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# Isolate system scientific packages from incompatible user-site installations.
exec "${BANDWIDTH_PYTHON:-/usr/bin/python3}" -s "$ROOT/tools/wheel_bandwidth/launch.py" "$@"
