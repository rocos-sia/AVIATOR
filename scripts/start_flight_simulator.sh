#!/usr/bin/env bash
# Run on the second IPC. Does not start Bus, Core, or robot devices.
set -euo pipefail
if [[ -f $(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh ]]; then
    source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
fi
SIM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo 'Usage: start_flight_simulator.sh [--config path]'
    echo 'Edit flight-simulator.yaml serial paths first. AVIATOR_BIN overrides the binary directory.'
    exit 0
fi
if [[ -x $SIM_ROOT/../../bin/flight_gateway ]]; then
    SIM_BIN=${AVIATOR_BIN:-$SIM_ROOT/../../bin}
else
    SIM_BIN=${AVIATOR_BIN:-$SIM_ROOT/build/communication/bin}
    [[ -x $SIM_BIN/flight_gateway ]] || SIM_BIN=${AVIATOR_BIN:-$SIM_ROOT/build/bin}
fi
if [[ $# == 0 ]]; then
    set -- --config "$SIM_ROOT/config/flight-simulator.yaml"
fi
exec "$SIM_BIN/flight_gateway" "$@"
