#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "${UI_PYTHON:-/usr/bin/python3}" "$HERE/grip_demo.py" "$@"
