#!/usr/bin/env bash
# Launch from any directory; all options are passed to the native UI.
set -euo pipefail
if [[ -f $(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh ]]; then
    source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
fi
AVIATOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERA_PYTHON="${CAMERA_PYTHON:-${CONDA_ROOT:-$HOME/miniconda3}/envs/apriltag_realsense/bin/python}"
export CAMERA_PYTHON
UI_PYTHON="${UI_PYTHON:-python3}"
exec "$UI_PYTHON" "$AVIATOR_ROOT/tools/steering_wheel_calibration/ui.py" "$@"
