#!/usr/bin/env bash
# Local desktop UI; opening it does not start remote devices.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec /usr/bin/python3 -B "$ROOT/tools/remote_launcher/main.py" "$@"
