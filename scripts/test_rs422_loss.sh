#!/usr/bin/env bash
# Use the same installed Python environment as the other runtime tools.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PREFIX=$(cd -- "$ROOT/../.." && pwd)
if [[ -f $PREFIX/release-version.json ]]; then
    python="$PREFIX/cache/python/bin/python"
    [[ -x $python ]] || { echo "部署 Python 不存在：$python；请重新执行项目部署。" >&2; exit 1; }
else
    python=/usr/bin/python3
fi
exec "$python" -s "$ROOT/scripts/test_rs422_loss.py" "$@"
