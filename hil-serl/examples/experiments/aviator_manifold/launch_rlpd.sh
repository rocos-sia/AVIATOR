#!/usr/bin/env bash
# Launch unshielded RLPD with a fresh SAC state and DP replay prior.
#
# Env vars mirror run_learner.sh / run_actor.sh: disable XLA preallocation so the
# learner (0.3) and actor (0.1) share the 48 GB GPU without OOM.
set -euo pipefail

cd "$(dirname "$0")"                 # .../aviator_manifold
source /home/rocos/miniconda3/etc/profile.d/conda.sh
conda activate serl_clean
export PYTHONPATH="$(cd ../../.. && pwd)${PYTHONPATH:+:$PYTHONPATH}"
learner_max_steps="${AVIATOR_MAX_LEARNER_STEPS:-500000}"
actor_max_steps="${AVIATOR_MAX_ACTOR_STEPS:-60000}"
run_dir="${AVIATOR_RUN_DIR:-route_a_lut_native_rlpd}"
if [[ -e "$run_dir" && "${AVIATOR_RESUME:-0}" != "1" ]]; then
  echo "Run directory already exists: $run_dir (set AVIATOR_RESUME=1 to resume it)" >&2
  exit 1
fi

# Diagnose the DP/LUT state chart and the physical clearance before allowing
# any prior into the critic. The following replay gate remains the hard stop.
python ../../../tools/audit_prior_states.py \
  --manifold-dir ../../../data/aviator/manifold_phi \
  --trajectory-dir ../../../data/aviator/trajectory_source \
  --demo-dir ../../../data/aviator/dp_demo \
  --output-dir ../../../data/aviator/prior_state_audit \
  --binary ../../../../AviatorRobot/build/bin/aviator_clearance_trajectory \
  --config ../../../../AviatorRobot/config/aviator.yaml

python ../../../tools/audit_unshielded_prior.py \
  --manifold-dir ../../../data/aviator/manifold_phi \
  --trajectory-dir ../../../data/aviator/trajectory_source \
  --demo-dir ../../../data/aviator/dp_demo

python ../../../tools/audit_online_starts.py \
  --manifold-dir ../../../data/aviator/manifold_phi \
  --trajectory-dir ../../../data/aviator/trajectory_source \
  --output-dir ../../../data/aviator/online_start_audit \
  --binary ../../../../AviatorRobot/build/bin/aviator_clearance_trajectory \
  --config ../../../../AviatorRobot/config/aviator.yaml

if [[ "${AVIATOR_PREFLIGHT_ONLY:-0}" == "1" ]]; then
  echo "RLPD preflight passed; learner and actor were not started."
  exit 0
fi

COMMON=(
  ../../train_rlpd.py
  --exp_name=aviator_manifold
  --checkpoint_path="$run_dir"
)

DEMO_ARGS=()
for f in ../../../data/aviator/dp_demo/traj_*.pkl; do
  DEMO_ARGS+=(--demo_path="$f")
done

# learner (TrainerServer) first; actor connects to it via TrainerClient.
AVIATOR_MAX_STEPS="$learner_max_steps" \
XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.3 \
nohup python "${COMMON[@]}" --learner "${DEMO_ARGS[@]}" \
  > unshielded_learner.log 2>&1 &
learner_pid=$!
echo "learner pid: $learner_pid"

# give the learner time to load the demo buffer and start the server
sleep 30

AVIATOR_MAX_STEPS="$actor_max_steps" \
XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.1 \
nohup python "${COMMON[@]}" --actor \
  > unshielded_actor.log 2>&1 &
actor_pid=$!
echo "actor pid: $actor_pid"

# Keep the launching shell alive so the managed command session owns both
# processes for the full experiment and can report failures through wait.
if ! wait "$actor_pid"; then
  kill "$learner_pid" 2>/dev/null || true
  wait "$learner_pid" 2>/dev/null || true
  exit 1
fi
wait "$learner_pid"
