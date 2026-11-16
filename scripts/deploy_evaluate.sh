#!/usr/bin/env bash
# Independent evaluation package; never builds or deploys the robot runtime.
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 -B "$ROOT/tools/evaluate/deploy.py" "$@"
