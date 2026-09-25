#!/bin/bash
# Sweep the CSP position-loop kp (default aviator.xml, gravity-comped, kd=80).
set -uo pipefail
cd /home/rocos/sia/AVIATOR
SIM=reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim
PROBE=AviatorRobot/build/bin/aviator_joint_step_probe
CONFIG=AviatorRobot/config/aviator.yaml
LUT=hil-serl/data/aviator/manifold_phi
OUTROOT=methods/t6/results/csp_kp_sweep
STEP=0.0011
KD=80
mkdir -p "$OUTROOT"

run() {
  local kp=$1
  local out="$OUTROOT/step_kp_${kp}"
  local log="$out.log"
  echo "===== CSP kp=$kp kd=$KD (default aviator.xml) ====="
  # NOTE: do NOT set AVIATOR_STEP_MODEL -> sim uses default aviator.xml (no actuators).
  if env -u AVIATOR_STEP_MODEL \
        AVIATOR_SIM_POSITION_KP="$kp" AVIATOR_SIM_POSITION_KD="$KD" \
        python3 AviatorRobot/tests/run_joint_step_probe.py \
        "$SIM" "$PROBE" "$CONFIG" "$out" "$LUT" "$STEP" > "$log" 2>&1; then
    grep -E 'step_probe_completed|wheel_10ms_completion|wheel_settled_5pct_ms|t90_ms|settled_5pct_ms' "$log" | sed 's/^/  /'
  else
    echo "  FAILED"; tail -n 8 "$log" | sed 's/^/  /'
  fi
}

run 1000
run 2000
run 5000
run 10000
run 20000
