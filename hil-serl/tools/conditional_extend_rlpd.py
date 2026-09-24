"""Wait for the 400-rollout run, evaluate it, and train 200 more.

Run from ``hil-serl``. The continuation restores the complete SAC checkpoint
but starts a fresh online replay buffer; the first run does not persist replay.
The DP prior is loaded again in the continuation. New trajectory *reset states*
are physically audited; the policy must discover full-horizon feasibility.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / "examples" / "experiments" / "aviator_manifold"
SOURCE = ROOT / "data" / "aviator" / "trajectory_source"
RUN = EXPERIMENT / "route_a_lut_native_rlpd"
EXTENSION = EXPERIMENT / "route_a_lut_native_rlpd_plus200"
TRAJECTORIES = ROOT / "data" / "aviator" / "trajectory_source_plus200"
STATUS = EXPERIMENT / "conditional_extend_rlpd_status.json"
PYTHON = Path(sys.executable)


def record(stage: str, **fields) -> None:
    payload = {"stage": stage, "updated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"), **fields}
    tmp = STATUS.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(STATUS)
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def checkpoint_steps(directory: Path) -> list[int]:
    steps = []
    for path in directory.glob("checkpoint_*"):
        try:
            steps.append(int(path.name.removeprefix("checkpoint_")))
        except ValueError:
            pass
    return sorted(steps)


def completed_episodes(log: Path) -> int:
    # The actor prints a rolling 10-step window. Its progress bar repeats the
    # description on each step, so count each reporting step only once.
    pattern = re.compile(r"completed=(\d+)/(\d+), mean J=([^:]+):\s+\d+%\|.*?\|\s*(\d+)/60000")
    by_step = {}
    for match in pattern.finditer(log.read_text(errors="replace")):
        by_step[int(match.group(4))] = int(match.group(2))
    return sum(n for step, n in by_step.items() if step % 10 == 0)


def evaluate(step: int) -> dict:
    env = os.environ.copy()
    env["JAX_PLATFORMS"] = "cpu"
    env["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    command = [str(PYTHON), "-m", "tools.eval_rlpd_policy", "--split", "val",
               "--n-eps", "50", "--sac-checkpoint", str(RUN), "--ckpt-step", str(step)]
    result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True)
    match = re.search(
        r"^rlpd\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)"
        r"\s+([\d.]+)\s+([-\d.]+)\s+([\d.]+)\s+([-\d.]+)",
        result.stdout, re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"cannot parse fixed-val evaluation for checkpoint {step}:\n{result.stdout}")
    return {"checkpoint_step": step, "completion_rate": float(match.group(1)) / 100,
            "clearance_rate": float(match.group(2)) / 100,
            "speed_rate": float(match.group(3)) / 100,
            "mean_return": float(match.group(9)), "raw": result.stdout}


def trajectory_fingerprint(path: Path) -> str:
    """Hash trajectory samples, independent of NPZ container metadata."""
    digest = hashlib.sha256()
    with np.load(path, allow_pickle=False) as trajectory:
        for key in ("t", "x", "xdot", "xddot"):
            values = np.ascontiguousarray(trajectory[key])
            digest.update(key.encode())
            digest.update(str(values.shape).encode())
            digest.update(str(values.dtype).encode())
            digest.update(values.tobytes())
    return digest.hexdigest()


def make_trajectories() -> dict:
    candidate_root = TRAJECTORIES / "candidates"
    command = [str(PYTHON), str(SOURCE / "generate.py"), "--seed", "20260924",
               "--n-dp", "0", "--n-rl", "240", "--n-val", "0", "--n-test", "0",
               "--out", str(candidate_root)]
    subprocess.run(command, cwd=ROOT, check=True)

    audit_root = TRAJECTORIES / "candidate_start_audit"
    command = [str(PYTHON), "-m", "tools.audit_online_starts", "--trajectory-dir",
               str(candidate_root), "--expected-count", "240", "--output-dir", str(audit_root),
               "--binary", str(ROOT.parent / "AviatorRobot/build/bin/aviator_clearance_trajectory"),
               "--config", str(ROOT.parent / "AviatorRobot/config/aviator.yaml")]
    # The audit exits 1 if *any* candidate is infeasible. We select only rows
    # whose physical reset check passed, and require at least 200 of them.
    subprocess.run(command, cwd=ROOT, check=False)
    table = pd.read_csv(audit_root / "online_starts.csv")
    valid = table.loc[table["valid_start"].astype(bool)]
    existing = {trajectory_fingerprint(path)
                for path in (SOURCE / "trajs").glob("*/*.npz")}

    destination = TRAJECTORIES / "trajs" / "rl_train"
    destination.mkdir(parents=True, exist_ok=True)
    selected = []
    excluded_duplicates = 0
    seen = set(existing)
    for old_index in valid["index"].astype(int):
        source = candidate_root / "trajs" / "rl_train" / f"traj_{old_index:04d}.npz"
        fingerprint = trajectory_fingerprint(source)
        if fingerprint in seen:
            excluded_duplicates += 1
            continue
        seen.add(fingerprint)
        new_index = len(selected)
        target = destination / f"traj_{new_index:04d}.npz"
        shutil.copy2(source, target)
        selected.append({"index": new_index, "candidate_index": old_index,
                         "path": str(target.relative_to(TRAJECTORIES)),
                         "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                         "trajectory_fingerprint": fingerprint})
        if len(selected) == 200:
            break
    if len(selected) != 200:
        raise RuntimeError(f"only {len(selected)}/240 candidates had valid, non-duplicate starts")
    manifest = {"seed": 20260924, "candidate_count": 240, "valid_candidates": int(len(valid)),
                "selected_count": 200, "excluded_exact_duplicates": excluded_duplicates,
                "compared_against": "all current dp_train/rl_train/val/test trajectory samples",
                "start_audit": str(audit_root / "online_starts.csv"),
                "trajectories": selected}
    (TRAJECTORIES / "extension_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--actor-pid", type=int, required=True)
    parser.add_argument("--learner-pid", type=int, required=True)
    args = parser.parse_args()

    lock_file = EXPERIMENT / "conditional_extend_rlpd.lock"
    with lock_file.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if STATUS.exists():
            previous = json.loads(STATUS.read_text())
            if previous.get("stage") in {"extension_running", "extension_complete"}:
                raise RuntimeError(f"task already acted on this run: {previous['stage']}")
        record("waiting_for_400", actor_pid=args.actor_pid, learner_pid=args.learner_pid)
        while alive(args.actor_pid) or alive(args.learner_pid):
            time.sleep(30)
        time.sleep(5)  # allow checkpoint writes to close

        actor_log = EXPERIMENT / "unshielded_actor.log"
        learner_log = EXPERIMENT / "unshielded_learner.log"
        actor_text = actor_log.read_text(errors="replace")
        learner_text = learner_log.read_text(errors="replace")
        if ("Traceback (most recent call last)" in actor_text[-20000:]
                or "Traceback (most recent call last)" in learner_text[-20000:]
                or "actor step cap reached" in actor_text[-20000:]):
            raise RuntimeError("original training ended with an error; see actor/learner logs")
        logged_episodes = completed_episodes(actor_log)
        if logged_episodes < 390:
            raise RuntimeError(f"original run ended too early (logged at most {logged_episodes} episodes)")
        steps = checkpoint_steps(RUN)
        if not steps or steps[-1] <= 80000:
            raise RuntimeError("original run has no final checkpoint after 80k")
        # The learner writes a non-periodic checkpoint only after the actor's
        # actor-done message. A periodic checkpoint alone cannot prove that
        # all 400 episodes completed, so stop for manual review in that rare
        # ambiguous case (final step exactly divisible by 20k).
        if steps[-1] % 20000 == 0:
            raise RuntimeError("cannot prove 400 episodes completed: latest checkpoint is periodic")

        record("evaluating", logged_episodes=logged_episodes, final_checkpoint=steps[-1])
        baseline = final = None
        evaluation_error = None
        try:
            baseline = evaluate(80000)
            final = evaluate(steps[-1])
        except Exception as exc:
            # The requested extension is unconditional; an evaluation error
            # must not turn the comparison into another continuation gate.
            evaluation_error = str(exc)
        (EXPERIMENT / "conditional_extend_rlpd_evaluation.json").write_text(
            json.dumps({"baseline": baseline, "final": final,
                        "evaluation_error": evaluation_error}, indent=2) + "\n")
        record("generating_200", baseline=baseline, final=final,
               evaluation_error=evaluation_error)
        manifest = make_trajectories()
        EXTENSION.mkdir(exist_ok=True)
        if (EXTENSION / "actor.log").exists() or (EXTENSION / "learner.log").exists():
            raise RuntimeError("continuation already started; review its logs before retrying")
        source_checkpoint = RUN / f"checkpoint_{steps[-1]}"
        shutil.copytree(source_checkpoint, EXTENSION / source_checkpoint.name,
                        dirs_exist_ok=True)
        (EXTENSION / "continuation_metadata.json").write_text(json.dumps({
            "source_checkpoint": str(source_checkpoint), "trajectory_source": str(TRAJECTORIES),
            "trajectory_count": manifest["selected_count"],
            "online_replay_restored": False, "dp_prior_reloaded": True,
            "baseline": baseline, "final_400": final,
            "evaluation_error": evaluation_error}, indent=2) + "\n")
        record("extension_running", baseline=baseline, final=final,
               evaluation_error=evaluation_error, continuation_run=str(EXTENSION),
               trajectory_source=str(TRAJECTORIES))
        command = ["bash", str(EXPERIMENT / "launch_continuation_200.sh"),
                   str(EXTENSION), str(TRAJECTORIES)]
        subprocess.run(command, cwd=ROOT, check=True)
        record("extension_complete", continuation_run=str(EXTENSION),
               final_checkpoint=max(checkpoint_steps(EXTENSION)))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        record("error", error=str(exc), traceback=traceback.format_exc())
        raise
