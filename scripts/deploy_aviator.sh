#!/usr/bin/env bash
# Run on the development computer; credentials are handled only by SSH/sudo.
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 "$ROOT/deploy/deploy.py" "$@"
