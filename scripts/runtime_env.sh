#!/usr/bin/env bash
# Shared defaults; source-tree tools retain their existing environment.
aviator_runtime_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
aviator_runtime_prefix=$(cd -- "$aviator_runtime_root/../.." && pwd)
if [[ -f $aviator_runtime_prefix/release-version.json ]]; then
    export AVIATOR_BIN=${AVIATOR_BIN:-$aviator_runtime_prefix/bin}
    export PYTHONDONTWRITEBYTECODE=1
    if [[ -x $aviator_runtime_prefix/cache/python/bin/python ]]; then
        export CAMERA_PYTHON=${CAMERA_PYTHON:-$aviator_runtime_prefix/cache/python/bin/python}
        export HAND_PYTHON=${HAND_PYTHON:-$aviator_runtime_prefix/cache/python/bin/python}
        export UI_PYTHON=${UI_PYTHON:-$aviator_runtime_prefix/cache/python/bin/python}
        export TTS_PYTHON=${TTS_PYTHON:-$aviator_runtime_prefix/cache/python/bin/python}
        export BANDWIDTH_PYTHON=${BANDWIDTH_PYTHON:-$aviator_runtime_prefix/cache/python/bin/python}
    fi
    export AVIATOR_RECORDING_DIR=${AVIATOR_RECORDING_DIR:-$aviator_runtime_root/logs}
fi
