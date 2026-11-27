"""Continue the frozen remount pipeline, stopping on an unpassed prerequisite.

The first serial B/C run and geometry preparation may already be running. This
driver waits for their saved gate decisions; it never fabricates RL artifacts.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from tools.gfr1_remount_evaluate import ROOT, dump, RULES


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    state = dict(status="running", active_stage="serial_BC", stages={},
        tracking_gate=False, cycle_5ms_hard_gate=False, qualified_shared_policy_exists=False)
    def update(stage, status, **extra):
        state["active_stage"] = stage
        state["stages"][stage] = dict(status=status, **extra)
        dump(out/"pipeline_status.json", state)
    def wait_gate(directory, stage):
        update(stage, "waiting_for_gate", directory=str(directory))
        gate = directory/"gate.json"
        while not gate.exists():
            time.sleep(5)
        payload = json.loads(gate.read_text())
        update(stage, "passed" if payload["passed"] else "failed", directory=str(directory), decision=payload)
        if not payload["passed"]:
            state["status"] = "stopped_on_failed_condition"
            state["stop_stage"] = stage
            dump(out/"pipeline_status.json", state)
            return False
        return True
    def run(stage, script, arguments, directory):
        update(stage, "running", directory=str(directory))
        with (out/(stage+".log")).open("x") as log:
            result = subprocess.run([sys.executable, str(ROOT/"hil-serl/tools"/script), *map(str, arguments)],
                stdout=log, stderr=subprocess.STDOUT)
        if result.returncode and not (directory/"gate.json").exists():
            update(stage, "execution_error", exit_code=result.returncode)
            state["status"] = "needs_execution_repair"
            dump(out/"pipeline_status.json", state)
            return False
        return wait_gate(directory, stage)
    if not (out/"bc_serial/interruption.json").exists():
        if not wait_gate(out/"bc_serial", "serial_BC"):
            return
    if not wait_gate(out/"shared_dataset", "shared_installation_setup"):
        return
    simple = out/"shared_simple_controls"
    if simple.exists():
        if not wait_gate(simple, "simple_controls"):
            return
    elif not run("simple_controls", "gfr1_shared_remount_compare.py",
            ["--manifest",out/"shared_dataset/development.json", "--output",simple,
             "--methods","B","C","F0","F1","F2"], simple):
            return
    update("scene_profiles", "running")
    with (out/"scene_profiles.log").open("x") as log:
        profiled = subprocess.run([sys.executable, str(ROOT/"hil-serl/tools/gfr1_shared_remount_profiles.py"),
            "--run", str(simple)], stdout=log, stderr=subprocess.STDOUT)
    if profiled.returncode:
        update("scene_profiles", "execution_error", exit_code=profiled.returncode)
        state["status"] = "needs_execution_repair"
        dump(out/"pipeline_status.json", state)
        return
    update("scene_profiles", "passed")
    controls = json.loads((simple/"episodes.json").read_text())
    import numpy as np
    eligible = {method: [r for r in controls if r["method"]==method] for method in ("C", "F0", "F1", "F2")}
    eligible = {m:rows for m,rows in eligible.items() if rows and all(r["completed"] for r in rows)}
    reference = min(eligible, key=lambda m: np.median([r["cycle_ms"]["p95"] for r in eligible[m]]))
    dump(out/"simple_reference.json", dict(method=reference, rule="lowest development median P95 among completely successful C/F0/F1/F2", frozen_before_heldout=True))
    # Dependency installation is delayed until timed baseline work finishes.
    # Existing simulation packages stay isolated and unchanged.
    learning = Path("/tmp/aviator-remount-learning-deps")
    update("learning_dependencies", "installing")
    with (out/"learning_dependencies.log").open("x") as log:
        result = subprocess.run([sys.executable, "/tmp/aviator-remount-pip.pyz", "install", "--target", str(learning),
            "--extra-index-url", "https://download.pytorch.org/whl/cpu", "torch==2.5.1+cpu",
            "stable-baselines3==2.3.2", "numpy==1.26.4", "gymnasium==0.29.1"], stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        update("learning_dependencies", "execution_error", exit_code=result.returncode)
        state["status"] = "needs_execution_repair"
        dump(out/"pipeline_status.json", state)
        return
    os.environ["PYTHONPATH"] = os.environ.get("PYTHONPATH", "")+os.pathsep+str(learning)
    update("learning_dependencies", "ready")
    for seed in RULES["training_seeds"]:
        train = out/f"seed_{seed}"
        if not run(f"train_seed_{seed}", "gfr1_train_shared_remount.py",
            ["--dataset",out/"shared_dataset", "--baseline-gate",simple/"gate.json",
             "--profile-json",simple/"scene_profiles.json", "--output",train,"--seed",seed], train):
            return
        checkpoint = json.loads((train/"gate.json").read_text())["selected"]["checkpoint"]
        state["qualified_shared_policy_exists"] = True
        dev = out/f"development_seed_{seed}"
        if not run(f"development_seed_{seed}", "gfr1_shared_remount_compare.py",
            ["--manifest",out/"shared_dataset/development.json", "--checkpoint",checkpoint,
             "--output",dev,"--methods","A","B","C","D","E"], dev):
            return
        held = out/f"heldout_seed_{seed}"
        held_methods = ["A","C","E"] + ([reference] if reference != "C" else [])
        if not run(f"heldout_seed_{seed}", "gfr1_shared_remount_compare.py",
            ["--manifest",out/"shared_dataset/heldout.json", "--checkpoint",checkpoint,
             "--output",held,"--methods",*held_methods], held):
            return
        comparisons = json.loads((held/"summary.json").read_text())["paired"]
        decision = next(p for p in comparisons if p["baseline"]=="C" and p["proposed"]=="E")
        if not decision["relative_latency_passed"]:
            update(f"relative_latency_seed_{seed}", "failed", decision=decision)
            state["status"] = "stopped_on_failed_condition"
            state["stop_stage"] = f"relative_latency_seed_{seed}"
            dump(out/"pipeline_status.json", state)
            return
        update(f"relative_latency_seed_{seed}", "passed", decision=decision)
        if reference != "C":
            fixed_comparison = next(p for p in comparisons if p["baseline"]==reference and p["proposed"]=="E")
            if fixed_comparison["reliability_passed"] and not fixed_comparison["relative_latency_passed"]:
                update(f"strong_simple_reference_seed_{seed}", "failed", decision=fixed_comparison)
                state["status"] = "stopped_on_failed_condition"
                state["stop_stage"] = f"strong_simple_reference_seed_{seed}"
                dump(out/"pipeline_status.json", state)
                return
    # Residual experiments require separate true/estimated geometry execution;
    # no success claim is emitted just because the matched-model phase passed.
    state["status"] = "matched_geometry_complete_residual_phase_pending"
    dump(out/"pipeline_status.json", state)


if __name__ == "__main__":
    main()
