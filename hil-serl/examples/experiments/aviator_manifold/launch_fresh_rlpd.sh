#!/usr/bin/env bash
# One uninterrupted RLPD run with 99 DP prior trajectories and a selected
# online task split. Defaults are 600 online trajectories and 24 actor envs.
set -euo pipefail

cd "$(dirname "$0")"
source /home/rocos/miniconda3/etc/profile.d/conda.sh
conda activate serl_clean
hil_serl_root="$(cd ../../.. && pwd)"
export PYTHONPATH="$hil_serl_root${PYTHONPATH:+:$PYTHONPATH}"

online_episodes="${AVIATOR_MAX_ONLINE_EPISODES:-600}"
actor_envs="${AVIATOR_NUM_ENVS:-24}"
run_dir="${AVIATOR_RUN_DIR:-$PWD/route_a_lut_native_rlpd_${online_episodes}x${actor_envs}}"
if [[ "$run_dir" != /* ]]; then
  run_dir="$PWD/$run_dir"
fi
trajectory_dir="${AVIATOR_TRAJECTORY_DIR:-$hil_serl_root/data/aviator/trajectory_source_${online_episodes}}"
audit_summary="${AVIATOR_AUDIT_SUMMARY:-$hil_serl_root/data/aviator/online_start_audit_${online_episodes}/summary.json}"

if [[ -e "$run_dir" ]]; then
  echo "Fresh run directory already exists: $run_dir" >&2
  exit 1
fi
python - "$trajectory_dir" "$audit_summary" "$online_episodes" "$actor_envs" <<'PY'
import glob, json, sys
from pathlib import Path
trajectory_dir, audit_summary = map(Path, sys.argv[1:3])
expected, actor_envs = map(int, sys.argv[3:])
paths = glob.glob(str(trajectory_dir / 'trajs/rl_train/traj_*.npz'))
summary = json.loads(audit_summary.read_text())
if expected < actor_envs or actor_envs <= 0:
    raise SystemExit('Online episode count must be at least the positive actor environment count')
if len(paths) != expected or summary['online_paths'] != expected or summary['valid_starts'] != expected:
    raise SystemExit(f'Expected {expected} trajectory files with {expected} physically valid starts')
PY

demo_args=()
for f in ../../../data/aviator/dp_demo/traj_*.pkl; do
  demo_args+=(--demo_path="$f")
done
if [[ "${#demo_args[@]}" -ne 99 ]]; then
  echo "Expected exactly 99 DP prior trajectories; found ${#demo_args[@]}" >&2
  exit 1
fi

if [[ "${AVIATOR_PREFLIGHT_ONLY:-0}" == "1" ]]; then
  echo "Fresh ${online_episodes}x${actor_envs} preflight passed; no learner or actor started."
  exit 0
fi

mkdir "$run_dir"
export AVIATOR_TRAJECTORY_DIR="$trajectory_dir"
export AVIATOR_NUM_ENVS="$actor_envs"
export AVIATOR_MAX_ONLINE_EPISODES="$online_episodes"
export AVIATOR_ACTOR_STEP_DELAY=0.5
export AVIATOR_UPDATES_PER_ONLINE_TRANSITION=1

python - "$run_dir" "$trajectory_dir" "$online_episodes" "$actor_envs" <<'PY'
import json, sys
from pathlib import Path
run_dir, trajectory_dir = map(Path, sys.argv[1:3])
online_episodes, actor_envs = map(int, sys.argv[3:])
metadata = {
    'run_type': 'fresh_uninterrupted_rlpd',
    'seed': 42,
    'dp_prior_trajectories': 99,
    'online_trajectories': online_episodes,
    'actor_envs': actor_envs,
    'actor_step_delay_s': 0.5,
    'updates_per_online_transition': 1,
    'trajectory_source': str(trajectory_dir),
    'trajectory_manifest': str(trajectory_dir / 'manifest.json'),
    'checkpoint_selection': 'evaluate saved checkpoints on the fixed val split; do not assume final is best',
}
(run_dir / 'run_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
PY

common=(../../train_rlpd.py --exp_name=aviator_manifold \
        --checkpoint_path="$run_dir" --seed=42)

AVIATOR_MAX_STEPS=1200000 XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.3 \
python -u "${common[@]}" --learner "${demo_args[@]}" \
  > "$run_dir/learner.log" 2>&1 &
learner_pid=$!
echo "learner pid: $learner_pid"
sleep 30

AVIATOR_MAX_STEPS=60000 XLA_PYTHON_CLIENT_PREALLOCATE=false \
XLA_PYTHON_CLIENT_MEM_FRACTION=.1 \
python -u "${common[@]}" --actor \
  > "$run_dir/actor.log" 2>&1 &
actor_pid=$!
echo "actor pid: $actor_pid"

if ! wait "$actor_pid"; then
  kill "$learner_pid" 2>/dev/null || true
  wait "$learner_pid" 2>/dev/null || true
  exit 1
fi
wait "$learner_pid"
