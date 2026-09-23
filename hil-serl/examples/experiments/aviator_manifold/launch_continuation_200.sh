#!/usr/bin/env bash
# Resume the SAC state on a separately audited set of 200 online trajectories.
set -euo pipefail

cd "$(dirname "$0")"
source /home/rocos/miniconda3/etc/profile.d/conda.sh
conda activate serl_clean
export PYTHONPATH="$(cd ../../.. && pwd)${PYTHONPATH:+:$PYTHONPATH}"

run_dir="${1:?pass the continuation run directory}"
trajectory_dir="${2:?pass the audited 200-trajectory source directory}"
[[ -d "$run_dir" && -d "$trajectory_dir/trajs/rl_train" ]]
[[ -n "$(find "$run_dir" -maxdepth 1 -name 'checkpoint_*' -print -quit)" ]]

export AVIATOR_TRAJECTORY_DIR="$trajectory_dir"
export AVIATOR_MAX_ONLINE_EPISODES=200
common=(../../train_rlpd.py --exp_name=aviator_manifold --checkpoint_path="$run_dir")
demo_args=()
for f in ../../../data/aviator/dp_demo/traj_*.pkl; do
  demo_args+=(--demo_path="$f")
done

AVIATOR_MAX_STEPS=700000 XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.3 \
python "${common[@]}" --learner "${demo_args[@]}" > "$run_dir/learner.log" 2>&1 &
learner_pid=$!
sleep 30
AVIATOR_MAX_STEPS=60000 XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.1 \
python "${common[@]}" --actor > "$run_dir/actor.log" 2>&1 &
actor_pid=$!

if ! wait "$actor_pid"; then
  kill "$learner_pid" 2>/dev/null || true
  wait "$learner_pid" 2>/dev/null || true
  exit 1
fi
wait "$learner_pid"
