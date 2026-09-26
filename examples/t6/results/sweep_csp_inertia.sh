#!/bin/bash
# CSP kp=10000/kd=80 baseline: reduce steering-wheel roll inertia (fullinertia Izz)
# to see if wheel settle can be pushed to ~10 ms. Realism irrelevant (per user).
set -uo pipefail
cd /home/rocos/sia/AVIATOR
SIM=reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim
PROBE=AviatorRobot/build/bin/aviator_joint_step_probe
CONFIG=AviatorRobot/config/aviator.yaml
LUT=hil-serl/data/aviator/manifold_phi
BASE=reference/rocos-mujoco/model/aviator.xml
MODELDIR=reference/rocos-mujoco/model
OUTROOT=methods/t6/results/csp_inertia_sweep
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

# steering_wheel roll inertia = fullinertia 3rd component (Izz), baseline 0.001
# sed: fullinertia="0.001 0.001 0.001 0.0 0.0 0.0" -> "0.001 0.001 <izz> 0.0 0.0 0.0"
for izz in 0.0003 0.0001 0.00003 0.00001; do
  m="$MODELDIR/aviator_csp_izz_${izz}.xml"
  sed "s/fullinertia=\"0.001 0.001 0.001 0.0 0.0 0.0\"/fullinertia=\"0.001 0.001 ${izz} 0.0 0.0 0.0\"/" "$BASE" > "$m"
  run "izz_${izz}" "$m"
done

# combined: reduced inertia + stiff weld (solref 0.001)
for izz in 0.0001 0.00003; do
  m="$MODELDIR/aviator_csp_izz_${izz}_weld0.001.xml"
  sed -e "s/fullinertia=\"0.001 0.001 0.001 0.0 0.0 0.0\"/fullinertia=\"0.001 0.001 ${izz} 0.0 0.0 0.0\"/" \
      -e "s/solref=\"0.005 1\"/solref=\"0.001 1\"/g" "$BASE" > "$m"
  run "izz_${izz}_weld0.001" "$m"
done
