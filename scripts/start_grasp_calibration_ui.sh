#!/usr/bin/env bash
# Standalone calibration entry point. Existing launch scripts are unaffected.
set -euo pipefail
AVIATOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$AVIATOR_ROOT"
UI_PYTHON="${UI_PYTHON:-python3}"
if ! "$UI_PYTHON" -c 'import numpy, yaml, zmq' >/dev/null; then
    echo "UI 解释器 $UI_PYTHON 缺少依赖或无法运行；需要 numpy、PyYAML、pyzmq。" >&2
    echo '请使用普通用户启动；sudo 会改变 Python 环境。可通过 UI_PYTHON 指定已安装依赖的解释器。' >&2
    exit 1
fi
exec "$UI_PYTHON" tools/grasp_tool_calibration/ui.py "$@"
