#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
source /home/rocos/miniconda3/etc/profile.d/conda.sh
conda activate serl_clean

export AVIATOR_MAX_STEPS="${AVIATOR_MAX_STEPS:-100}"
export AVIATOR_NUM_ENVS="${AVIATOR_NUM_ENVS:-2}"
export AVIATOR_MAX_ONLINE_EPISODES="${AVIATOR_MAX_ONLINE_EPISODES:-16}"
export AVIATOR_CHECKPOINT_PERIOD="${AVIATOR_CHECKPOINT_PERIOD:-50}"
export AVIATOR_TRAINING_STARTS="${AVIATOR_TRAINING_STARTS:-32}"
export AVIATOR_ACTOR_STEP_DELAY=0
export XLA_PYTHON_CLIENT_PREALLOCATE=false

run_dir="$(pwd)/smoke_unshielded_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
common=(../../train_rlpd.py --exp_name=aviator_manifold
        --checkpoint_path="$run_dir" --debug)
demo_args=()
for p in ../../../data/aviator/dp_demo/traj_*.pkl; do
    demo_args+=(--demo_path="$p")
done

AVIATOR_MAX_STEPS=1000 XLA_PYTHON_CLIENT_MEM_FRACTION=.3 \
    python "${common[@]}" --learner "${demo_args[@]}" \
    > "$run_dir/learner.log" 2>&1 &
learner_pid=$!
trap 'kill "$learner_pid" "${actor_pid:-}" 2>/dev/null || true' EXIT

sleep 15
XLA_PYTHON_CLIENT_MEM_FRACTION=.1 python "${common[@]}" --actor \
    > "$run_dir/actor.log" 2>&1 &
actor_pid=$!

wait "$actor_pid"
for _ in $(seq 1 60); do
    if [[ -d "$run_dir/checkpoint_50" ]]; then
        break
    fi
    if ! kill -0 "$learner_pid" 2>/dev/null; then
        break
    fi
    sleep 1
done
if [[ ! -d "$run_dir/checkpoint_50" ]]; then
    echo "learner did not reach checkpoint_50" >&2
    exit 1
fi
kill "$learner_pid" 2>/dev/null || true
wait "$learner_pid" || true
trap - EXIT
printf '%s\n' "$run_dir"
