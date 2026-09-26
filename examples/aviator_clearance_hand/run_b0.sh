#!/usr/bin/env bash
# B0 baseline clearance along the sine profile (theta = 0.87266*sin(2*pi*f*t),
# s = 0) on the hand + wall model, using the PIN-IK port of build_manifold_phi.
#
# Outputs (into ./out):
#   online_summary_b0.csv        one row per (profile, frequency)
#   trajectory_online_b0_*.csv   per-knot d_min / coll / dq / ddq / interv
set -euo pipefail
cd "$(dirname "$0")"

BIN=../../build/aviator_clearance/build_manifold_phi
if [[ ! -x "$BIN" ]]; then
    echo "build_manifold_phi not built. Build it first:" >&2
    echo "  cmake -S tools/aviator_clearance -B build/aviator_clearance -DAVIATOR_DEPENDENCY_JOBS=\$(nproc)" >&2
    echo "  cmake --build build/aviator_clearance -j\$(nproc)" >&2
    exit 1
fi

# CONFIG OUTPUT_DIR D_SAFE Q_MARGIN TRACE_HALF TRACE_STEP TASKS
"$BIN" config.yaml out 0.005 0.03 10 0.1 online_b0
