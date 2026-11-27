"""True/estimated geometry audit after frozen-policy heldout qualification.

This is kinematic command replay, not a contact-force/impedance simulation.
Residual levels are exploratory, not inferred from real calibration data.
"""
import argparse
import json
import os
from pathlib import Path
from types import SimpleNamespace
import time
import xml.etree.ElementTree as ET
import numpy as np

from tools.audit_static_collision import CollisionChecker
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_evaluate import ROOT, RULES, dump, sha, snapshot, audit_motion
from tools.gfr1_same_state_diagnosis import safe
from tools.gfr1_shared_remount_env import SharedRemountEnv
from tools.validate_differential_lift import AviatorClosure


def estimated_model(source, target, family, residual_m):
    tree = ET.parse(source)
    bodies = {b.get("name"):b for b in tree.iter("body")}
    if family == "wall_gap":
        changes = [("aviator_wall_left",1,residual_m), ("aviator_wall_right",1,-residual_m)]
    elif family == "wheel_dx":
        changes = [("steering_wheel",0,residual_m)]
    else:
        changes = [("AR5-5_07L-W4C4A2_base",0,.5*residual_m), ("AR5-5_07R-W4C4A2_base",0,-.5*residual_m)]
    for name, axis, delta in changes:
        position = np.fromstring(bodies[name].get("pos", "0 0 0"), sep=" ")
        position[axis] += delta
        bodies[name].set("pos", " ".join(format(v,".16g") for v in position))
    tree.write(target, encoding="unicode")


def run_case(env, record, method, policy, sink):
    env._model_records = [record]
    env.reset(seed=20261125, options={"trajectory_index":0})
    true_fk = AviatorClosure(record["true_model"])
    true_checker = CollisionChecker(record["true_model"])
    horizon = int(np.floor(2*(len(env._traj["x"])-1)*env.traj_dt/env.dt+1e-8))
    termination = "simulation_duration_limit"
    errors, times = [], []
    for step in range(horizon):
        before = snapshot(env)
        started = time.perf_counter_ns()
        result, direct, _ = env.method_step(method, policy)
        elapsed = (time.perf_counter_ns()-started)/1e6
        times.append(elapsed)
        _, _, ended, truncated, info = result
        audit = None
        if info["beta_executed"] > 0:
            view = SimpleNamespace(_x=env._x, _q=env._q, dt=env.dt, lookup=env.lookup, fk=true_fk)
            audit = audit_motion(view, true_checker, before["_x"], before["_q"])
            errors = audit["violations"]
        sink.write(json.dumps(safe(dict(case=record["case"], method=method, step=step,
            elapsed_ms=elapsed, before=before, after=snapshot(env), estimated_acceptance=info,
            true_audit=audit, direct_accepted=direct)), allow_nan=False)+"\n")
        if errors:
            termination = "accepted_motion_failed_true_geometry_audit"
            break
        if ended or truncated:
            termination = info["termination"]
            break
    return dict(case=record["case"], method=method, completed=termination=="end", termination=termination,
        true_geometry_violations=errors, steps=len(times), p95_ms=float(np.percentile(times,95)),
        total_compute_ms=float(sum(times)), progress=float(env._u/(len(env._traj["x"])-1)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, required=True)
    a = p.parse_args()
    root = a.root.resolve()
    os.sched_setaffinity(0,{2})
    import torch
    from stable_baselines3 import SAC
    torch.set_num_threads(1)
    out = root/"calibration_residual"
    out.mkdir(exist_ok=False)
    records = json.loads((root/"shared_dataset/heldout.json").read_text())["records"]
    dump(out/"protocol.json", dict(scope=__doc__, families=["wall_gap","wheel_dx","base_dx"],
        residual_m=[.00005,.0001,.00025], tracking_error_gate=False, rules=RULES,
        stopped_on="any E failure; C failures recorded for comparison", source_sha256=sha(__file__)))
    data = ROOT/"hil-serl/data/aviator"
    rows = []
    with use_projector("combined"), (out/"steps.jsonl").open("x",buffering=1) as sink:
        for seed in RULES["training_seeds"]:
            checkpoint = json.loads((root/f"seed_{seed}/gate.json").read_text())["selected"]["checkpoint"]
            model = SAC.load(checkpoint, device="cpu")
            checkpoint_hash = sha(checkpoint)
            env = SharedRemountEnv(str(data/"manifold_phi_stale_pitch-10deg"), records[0]["model"], str(data/"trajectory_source"))
            for residual in (.00005,.0001,.00025):
                for family in ("wall_gap","wheel_dx","base_dx"):
                    for index, source in enumerate(records):
                        target = out/f"estimate_{family}_{residual:.5f}_{index:03d}.xml"
                        if not target.exists():
                            estimated_model(source["model"],target,family,residual)
                        record = dict(source, model=str(target), estimated_model=str(target),
                            true_model=source["model"], case=f"{source['case']}/{family}/{residual:.5f}")
                        order = ("C","E") if index%2==0 else ("E","C")
                        for method in order:
                            row = run_case(env, record, method,
                                lambda core:model.predict(core.last_policy_observation,deterministic=True)[0],sink)
                            row.update(seed=seed, family=family, residual_m=residual, checkpoint_sha256=checkpoint_hash)
                            rows.append(row)
                            dump(out/"episodes.json",rows)
                            print(json.dumps(row),flush=True)
                        if any(not r["completed"] for r in rows if r["method"]=="E"):
                            dump(out/"gate.json",dict(passed=False,reason="E_failed_residual_condition",last_pair=rows[-2:],checkpoint_unchanged=sha(checkpoint)==checkpoint_hash))
                            env.close()
                            return
            if sha(checkpoint)!=checkpoint_hash:
                raise RuntimeError("Frozen checkpoint changed")
            env.close()
    dump(out/"gate.json",dict(passed=True,scope="exploratory kinematic residual audit only",episodes=len(rows)))


if __name__ == "__main__":
    main()
