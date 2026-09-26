#!/bin/bash
# Break the 21 ms floor: reduce CSP damping kd BELOW 80 (untested) and try
# kp scaling, on top of the best inertia+weld config (izz=0.0001, weld=0.001).
set -uo pipefail
cd /home/rocos/sia/AVIATOR
SIM=reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim
PROBE=AviatorRobot/build/bin/aviator_joint_step_probe
CONFIG=AviatorRobot/config/aviator.yaml
LUT=hil-serl/data/aviator/manifold_phi
BASE=reference/rocos-mujoco/model/aviator.xml
MODELDIR=reference/rocos-mujoco/model
OUTROOT=methods/t6/results/csp_finetune_sweep
STEP=0.0011
mkdir -p "$OUTROOT"

# base model: izz=0.0001 (10x lighter roll inertia) + weld solref 0.001
BASE_MODEL="$MODELDIR/aviator_csp_izz_0.0001_weld0.001.xml"
if [ ! -f "$BASE_MODEL" ]; then
  sed -e "s/fullinertia=\"0.001 0.001 0.001 0.0 0.0 0.0\"/fullinertia=\"0.001 0.001 0.0001 0.0 0.0 0.0\"/" \
      -e "s/solref=\"0.005 1\"/solref=\"0.001 1\"/g" "$BASE" > "$BASE_MODEL"
fi

run() {
  local tag=$1 kp=$2 kd=$3
  local out="$OUTROOT/step_${tag}"
  local log="$out.log"
  echo "===== kp=$kp kd=$kd (izz=0.0001, weld=0.001) ====="
  if AVIATOR_STEP_MODEL="$BASE_MODEL" \
       AVIATOR_SIM_POSITION_KP="$kp" AVIATOR_SIM_POSITION_KD="$kd" \
       python3 AviatorRobot/tests/run_joint_step_probe.py \
       "$SIM" "$PROBE" "$CONFIG" "$out" "$LUT" "$STEP" > "$log" 2>&1; then
    grep -E 'wheel_10ms_completion|wheel_settled_5pct_ms|t90_ms|settled_5pct_ms' "$log" | sed 's/^/  /'
  else
    echo "  FAILED"; tail -n 6 "$log" | sed 's/^/  /'
  fi
}

# kd sweep below 80 (previously untested)
run kd_0  10000 0
run kd_20 10000 20
run kd_40 10000 40
run kd_60 10000 60
run kd_80 10000 80
# kp scaling with lower kd
run kp20000_kd40 20000 40
run kp20000_kd80 20000 80
run kp30000_kd60 30000 60
