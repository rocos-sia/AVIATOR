"""One-shot status report for the overnight RLPD run.

Reads the wandb run's logged "environment" metrics (intervention rate,
completion rate, min clearance, max |qdot|, mean episode return) and prints the
most recent values, so periodic check-ins don't need to shell into wandb.
Usage: python monitor_run.py [wandb_run_id]
"""
import sys
import glob
import os

import wandb

RUN_DIR = os.path.dirname(os.path.abspath(__file__))


def _latest_run_id():
    # parse the run id out of the learner log (wandb prints "Syncing run <id>")
    log = os.path.join(RUN_DIR, "rlpd_learner.log")
    if not os.path.exists(log):
        return None
    with open(log, "rb") as f:
        data = f.read().decode("utf-8", "ignore")
    for line in data.splitlines():
        if "Syncing run" in line:
            return line.split("Syncing run")[-1].strip()
    return None


def _latest_checkpoint():
    ckpts = glob.glob(os.path.join(RUN_DIR, "debug_rlpd", "checkpoint_*"))
    if not ckpts:
        return None
    def key(p):
        return int(os.path.basename(p).split("_")[1])
    return os.path.basename(max(ckpts, key=key))


def main():
    run_id = sys.argv[1] if len(sys.argv) > 1 else _latest_run_id()
    print(f"run_id={run_id}")
    print(f"latest_checkpoint={_latest_checkpoint()}")

    if run_id is None:
        return

    api = wandb.Api()
    # run path is <entity>/<project>/<run_id>
    run = api.run(f"sjcyxr-northeastern-university/hil-serl/{run_id}")
    hist = run.scan_history()  # lazy; iterate to the end
    env_keys = {
        "environment/intervention_rate": "intervention_rate",
        "environment/completion_rate": "completion_rate",
        "environment/min_d": "min_d",
        "environment/max_qdot": "max_qdot",
        "environment/mean_episode_return": "mean_episode_return",
    }
    latest = {}
    for row in hist:
        for k, short in env_keys.items():
            if k in row and row[k] is not None:
                latest[short] = (row.get("_step"), row[k])

    for short, (step, val) in latest.items():
        print(f"{short}: {val:.6g}  (step {step})")


if __name__ == "__main__":
    main()
