#!/usr/bin/env bash
# Launch the overnight RLPD run (learner + 8-env actor) for aviator_manifold.
# Fresh start: BC-warmstarts the SAC actor from ../debug, then trains online.
#
# Env vars mirror run_learner.sh / run_actor.sh: disable XLA preallocation so the
# learner (0.3) and actor (0.1) share the 48 GB GPU without OOM.
set -euo pipefail

cd "$(dirname "$0")"                 # .../aviator_manifold
source /home/rocos/miniconda3/etc/profile.d/conda.sh
conda activate serl_clean

COMMON=(
  ../../train_rlpd.py
  --exp_name=aviator_manifold
  --checkpoint_path=../../experiments/aviator_manifold/debug_rlpd
  --bc_checkpoint_path=../../experiments/aviator_manifold/debug
)

DEMO_ARGS=()
for f in ../../../data/aviator/dp_demo/traj_*.pkl; do
  DEMO_ARGS+=(--demo_path="$f")
done

# learner (TrainerServer) first; actor connects to it via TrainerClient.
XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.3 \
nohup python "${COMMON[@]}" --learner "${DEMO_ARGS[@]}" \
  > rlpd_learner.log 2>&1 &
echo "learner pid: $!"

# give the learner time to load the demo buffer and start the server
sleep 30

XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.1 \
nohup python "${COMMON[@]}" --actor \
  > rlpd_actor.log 2>&1 &
echo "actor pid: $!"
