#!/bin/bash
# Sweep weld solref (coupling stiffness/damping) at fixed kp/kv baseline.
set -uo pipefail
cd /home/rocos/sia/AVIATOR
SIM=reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim
PROBE=AviatorRobot/build/bin/aviator_joint_step_probe
CONFIG=AviatorRobot/config/aviator.yaml
LUT=hil-serl/data/aviator/manifold_phi
BASE=reference/rocos-mujoco/model/aviator_position_20k500.xml
MODELDIR=reference/rocos-mujoco/model
OUTROOT=methods/t6/results/position_actuator_sweep
STEP=0.0011
mkdir -p "$OUTROOT"

run() {
  local solref=$1 tag=$2
  local model="$MODELDIR/aviator_pos_weld_${tag}.xml"
  sed "s/solref=\"0.005 1\"/solref=\"${solref}\"/g" "$BASE" > "$model"
  local out="$OUTROOT/step_weld_${tag}"
  local log="$out.log"
  echo "===== weld solref=$solref ($tag) ====="
  if AVIATOR_STEP_MODEL="$model" python3 AviatorRobot/tests/run_joint_step_probe.py \
      "$SIM" "$PROBE" "$CONFIG" "$out" "$LUT" "$STEP" > "$log" 2>&1; then
    grep -E 'step_probe_completed|wheel_10ms_completion|wheel_settled_5pct_ms|t90_ms|settled_5pct_ms' "$log" | sed 's/^/  /'
  else
    echo "  FAILED"; tail -n 6 "$log" | sed 's/^/  /'
  fi
}

run "0.005 1" "base"
run "0.002 1" "stiff2x"
run "0.001 1" "stiff5x"
run "0.005 2" "overdamp2"
