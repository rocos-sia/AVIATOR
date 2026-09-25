#!/bin/bash
# CSP kp=10000/kd=80 baseline: sweep wheel-damping and weld-stiffness levers
# to see if either can push wheel settle toward 10 ms.
set -uo pipefail
cd /home/rocos/sia/AVIATOR
SIM=reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim
PROBE=AviatorRobot/build/bin/aviator_joint_step_probe
CONFIG=AviatorRobot/config/aviator.yaml
LUT=hil-serl/data/aviator/manifold_phi
BASE=reference/rocos-mujoco/model/aviator.xml
MODELDIR=reference/rocos-mujoco/model
OUTROOT=methods/t6/results/csp_lever_sweep
STEP=0.0011
KP=10000
KD=80
mkdir -p "$OUTROOT"

run() {
  local tag=$1 model=$2
  local out="$OUTROOT/step_${tag}"
  local log="$out.log"
  echo "===== $tag ====="
  if AVIATOR_STEP_MODEL="$model" \
       AVIATOR_SIM_POSITION_KP="$KP" AVIATOR_SIM_POSITION_KD="$KD" \
       python3 AviatorRobot/tests/run_joint_step_probe.py \
       "$SIM" "$PROBE" "$CONFIG" "$out" "$LUT" "$STEP" > "$log" 2>&1; then
    grep -E 'step_probe_completed|wheel_10ms_completion|wheel_settled_5pct_ms|t90_ms|settled_5pct_ms' "$log" | sed 's/^/  /'
  else
    echo "  FAILED"; tail -n 6 "$log" | sed 's/^/  /'
  fi
}

# ---- wheel damping: roll_input_joint damping (baseline 0.1) ----
for d in 0.1 1 3 5 10 20; do
  m="$MODELDIR/aviator_csp_wheeldamp_${d}.xml"
  sed "s/damping=\"0.1\"/damping=\"${d}\"/" "$BASE" > "$m"
  run "wheeldamp_${d}" "$m"
done

# ---- weld stiffness: weld solref time constant (baseline 0.005 1) ----
for s in "0.005 1" "0.002 1" "0.001 1" "0.0005 1" "0.0002 1"; do
  tag=$(echo "$s" | tr ' ' '_')
  m="$MODELDIR/aviator_csp_weld_${tag}.xml"
  sed "s/solref=\"0.005 1\"/solref=\"${s}\"/g" "$BASE" > "$m"
  run "weld_${tag}" "$m"
done
