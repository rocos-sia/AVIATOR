#!/usr/bin/env bash
# Train a fresh SAC model after the dataset and checked priors exist.
# Usage: bash launch_speed_reversal_rlpd.sh dp_extra|dp_static
set -euo pipefail

cd "$(dirname "$0")"
variant="${1:-}"
check_only="${2:-}"
if [[ "$variant" != "dp_extra" && "$variant" != "dp_static" ]]; then
  echo "usage: bash launch_speed_reversal_rlpd.sh dp_extra|dp_static [--check-only]" >&2
  exit 2
fi
if [[ -n "$check_only" && "$check_only" != "--check-only" ]]; then
  echo "unknown argument: $check_only" >&2
  exit 2
fi

repo_root="$(cd ../../../.. && pwd)"
hil_root="$repo_root/hil-serl"
data_root="$repo_root/outputs/speed_reversal_grid_v3_1200"
manifold="$hil_root/data/aviator/manifold_phi_stale_pitch-10deg"
dp_prior="$data_root/dp_safe_prior.pkl"
static_prior="$data_root/static_phase_prior.pkl"
run_dir="$PWD/route_a_speed_reversal_1200_64_scratch_steps1m_${variant}"
for required in "$data_root/manifest.json" "$dp_prior" "$manifold/manifest.json"; do
  if [[ ! -f "$required" ]]; then
    echo "missing input: $required" >&2
    exit 1
  fi
done
if [[ "$variant" == "dp_static" && ! -f "$static_prior" ]]; then
  echo "missing input: $static_prior" >&2
  exit 1
fi
if [[ -e "$run_dir" ]]; then
  echo "run directory already exists: $run_dir" >&2
  exit 1
fi

source /home/rocos/miniconda3/etc/profile.d/conda.sh
conda activate serl_clean
export PYTHONPATH="$hil_root${PYTHONPATH:+:$PYTHONPATH}"
export AVIATOR_TRAJECTORY_DIR="$data_root"
export AVIATOR_MANIFOLD_DIR="$manifold"
export AVIATOR_NUM_ENVS=64
export AVIATOR_MAX_ONLINE_EPISODES=1200
export AVIATOR_MAX_ONLINE_TRANSITIONS=1000000
export AVIATOR_UPDATES_PER_ONLINE_TRANSITION=1
export AVIATOR_MAX_TRAJ_LENGTH=2000
export AVIATOR_QDDOT_MAX=10
export AVIATOR_INITIAL_PHI_FRACTION_MIN=0.25
export AVIATOR_INITIAL_PHI_FRACTION_MAX=0.75
export XLA_PYTHON_CLIENT_PREALLOCATE=false
export PYTHONUNBUFFERED=1

python - "$data_root/manifest.json" "$dp_prior" "$static_prior" "$variant" <<'PY'
import hashlib, json, pickle, sys
from collections import Counter
from pathlib import Path
manifest_path = Path(sys.argv[1])
manifest = json.loads(manifest_path.read_text())
counts = Counter(row['split'] for row in manifest['trajectories'])
assert counts == {'rl_train': 1200, 'val': 120, 'test': 1200}, counts
cells = Counter((row['split'], row['theta_speed_target'], row['reversals'])
                for row in manifest['trajectories'])
for split, n in (('rl_train', 100), ('val', 10), ('test', 100)):
    for speed in (0.6, 1.0, 1.2, 1.5):
        for reversals in (2, 4, 6):
            assert cells[split, speed, reversals] == n, (split, speed, reversals)
starts = {split: {tuple(row['initial_x']) for row in manifest['trajectories']
                  if row['split'] == split} for split in counts}
for left, right in (('rl_train', 'val'), ('rl_train', 'test'), ('val', 'test')):
    assert not (starts[left] & starts[right]), (left, right)
for row in manifest['trajectories']:
    path = manifest_path.parent / row['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == row['sha256'], path
with open(sys.argv[2], 'rb') as handle:
    dp = pickle.load(handle)
assert len(dp) > 0, 'empty dynamics-safe DP prior'
if sys.argv[4] == 'dp_static':
    with open(sys.argv[3], 'rb') as handle:
        static = pickle.load(handle)
    assert len(static) > 0, 'empty static phase prior'
print(f"preflight: {dict(counts)}, DP transitions={len(dp)}")
PY
if [[ "$check_only" == "--check-only" ]]; then
  exit 0
fi

mkdir -p "$run_dir"
cp "$data_root/manifest.json" "$run_dir/trajectory_manifest.json"
demo_args=("--demo_path=$dp_prior")
if [[ "$variant" == "dp_static" ]]; then
  demo_args+=("--extra_demo_path=$static_prior")
else
  demo_args+=("--extra_demo_path=$dp_prior")
fi
common=(../../train_rlpd.py --exp_name=aviator_manifold
        "--checkpoint_path=$run_dir" --seed=42 --debug)

learner_pid=""
actor_pid=""
cleanup() {
  if [[ -n "$actor_pid" ]]; then
    kill "$actor_pid" 2>/dev/null || true
  fi
  if [[ -n "$learner_pid" ]]; then
    kill "$learner_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
echo "training: 64 environments, 1200 trajectories reshuffled, 1000000 online transitions"
echo "logs: $run_dir/{learner,actor}.log"

AVIATOR_MAX_STEPS=1500000 XLA_PYTHON_CLIENT_MEM_FRACTION=.3 \
  python "${common[@]}" --learner "${demo_args[@]}" \
  > "$run_dir/learner.log" 2>&1 &
learner_pid=$!
ready=0
for _ in $(seq 1 120); do
  if grep -Fq 'sent initial network to actor' "$run_dir/learner.log"; then
    ready=1
    break
  fi
  if ! kill -0 "$learner_pid" 2>/dev/null; then
    echo "learner stopped during startup; see $run_dir/learner.log" >&2
    exit 1
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  echo "learner startup timeout; see $run_dir/learner.log" >&2
  exit 1
fi

AVIATOR_MAX_STEPS=60000 XLA_PYTHON_CLIENT_MEM_FRACTION=.1 \
  python "${common[@]}" --actor > "$run_dir/actor.log" 2>&1 &
actor_pid=$!
if ! wait "$actor_pid"; then
  echo "actor failed; see $run_dir/actor.log" >&2
  exit 1
fi
actor_pid=""
wait "$learner_pid"
learner_pid=""
echo "finished: $run_dir"
