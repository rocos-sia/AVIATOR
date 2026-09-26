#!/bin/bash
# Sweep CSP damping kd at fixed kp (default aviator.xml, gravity-comped).
set -uo pipefail
cd /home/rocos/sia/AVIATOR
SIM=reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim
PROBE=AviatorRobot/build/bin/aviator_joint_step_probe
CONFIG=AviatorRobot/config/aviator.yaml
LUT=hil-serl/data/aviator/manifold_phi
OUTROOT=methods/t6/results/csp_kp_sweep
STEP=0.0011
mkdir -p "$OUTROOT"

run() {
  local kp=$1 kd=$2
  local out="$OUTROOT/step_kp_${kp}_kd_${kd}"
  local log="$out.log"
  echo "===== CSP kp=$kp kd=$kd ====="
  if env -u AVIATOR_STEP_MODEL \
        AVIATOR_SIM_POSITION_KP="$kp" AVIATOR_SIM_POSITION_KD="$kd" \
        python3 AviatorRobot/tests/run_joint_step_probe.py \
        "$SIM" "$PROBE" "$CONFIG" "$out" "$LUT" "$STEP" > "$log" 2>&1; then
    grep -E 'step_probe_completed|wheel_10ms_completion|wheel_settled_5pct_ms|t90_ms|settled_5pct_ms' "$log" | sed 's/^/  /'
  else
    echo "  FAILED"; tail -n 8 "$log" | sed 's/^/  /'
  fi
}

run 10000 150
run 10000 250
run 10000 350
run 10000 500
run 20000 200
run 20000 400
