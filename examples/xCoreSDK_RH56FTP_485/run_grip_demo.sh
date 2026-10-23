#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ -f $HERE/../../scripts/runtime_env.sh ]]; then
    source "$HERE/../../scripts/runtime_env.sh"
fi
exec "${UI_PYTHON:-/usr/bin/python3}" "$HERE/grip_demo.py" "$@"
