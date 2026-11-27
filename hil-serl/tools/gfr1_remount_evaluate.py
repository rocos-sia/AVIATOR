"""Audited serial evaluation for the remount experiment (no hardware control).

Tracking errors are deliberately not success gates: the experiment validates
kinematic continuation and computation, not a simulated impedance controller.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import time
import traceback

import numpy as np

from experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.audit_static_collision import CollisionChecker
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_pilot import PilotEnv, ROOT, build_models, initialize, closure_errors
from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
from tools.gfr1_same_state_diagnosis import safe, STATE_FIELDS
from tools.gfr1_scheduler_benchmark import Meter, instrumentation, stats
from tools.validate_branch_connector import swept_clearance

RULES = {
    "scope": "kinematic simulation; no impedance or hardware tracking claim",
    "tracking_error_gate": False,
    "closure_translation_m": 0.0005, "closure_rotation_deg": 0.25,
    "clearance_m": 0.005, "clearance_numerical_tolerance_m": 1e-9,
    "joint_speed_rad_s": 1.5, "joint_speed_tolerance_rad_s": 2e-5,
    "joint_limit_tolerance_rad": 1e-9, "swept_subdivisions": 5,
    "simulation_duration_ratio_max": 2.0,
    "cycle_reference_ms": 5.0, "cycle_reference_is_hard_gate": False,
    "development_required_completion_fraction": 1.0,
    "minimum_useful_paired_p95_gain_ms": 1.0,
    "training_seeds": [11, 23, 37], "training_steps_per_seed": 60000,
    "training_evaluation_steps": [20000, 40000, 60000],
    "early_training_stop": "nonfinite learner or accepted-motion safety violation only",
    "training_qualification": "all development cases complete; then freeze checkpoint before heldout",
    "relative_latency_gate": "heldout installation-cluster 95% lower bound of C minus E P95 > 1 ms, each seed positive",
    "stage1_gate": "B and C complete all assigned development cases without audit violations",
    "fixed_rule_failure": "record failure; does not by itself stop RL training",
    "boundary_clearance_margin_m": 0.00025, "boundary_joint_margin_rad": 0.05,
    "reversal_window_s": 0.1,
}
FIXED = {"F0": [-1., -1.], "F1": [-1., -.5], "F2": [-1., 0.]}
FIELDS = (*STATE_FIELDS, "_repair_cycle", "_repair_previous_raw", "_repair_seed")


def dump(path, payload):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(safe(payload), indent=2, allow_nan=False))
    temporary.replace(path)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot(env):
    return {key: copy.deepcopy(getattr(env, key)) for key in FIELDS}


def restore(env, state):
    for key, value in state.items():
        setattr(env, key, copy.deepcopy(value))


class EvaluationEnv(PilotEnv):
    def action_step(self, raw, direct):
        raw = np.asarray(raw, dtype=np.float64)
        if raw.shape != (2,) or not np.isfinite(raw).all() or np.any(np.abs(raw) > 1):
            raise ValueError("Invalid normalized proposal")
        first = None
        if direct:
            first, _ = self._candidate(self._x, self._q, self._path_state(self._u)[1],
                raw * self.r_max, 1., self._previous_n, structured=False)
            if first["feasible"]:
                result = GFRManifoldEnv.step(self, raw, {"accepted": first, "candidates": [first]})
                self._repair_cycle += 1
                self._repair_previous_raw = raw.astype(np.float32)
                return result, True, first
        return RepairedGFRManifoldEnv.step(self, raw), False if direct else None, first

    def method_step(self, method, policy=None):
        if method == "A":
            result, _ = self.legacy_qp_step()
            return result, None, None
        if method in FIXED:
            raw = np.asarray(FIXED[method])
        elif method in ("D", "E"):
            raw = policy(self)
        else:
            raw = np.clip(self._r_prev / self.r_max, -1., 1.)
        return self.action_step(raw, method in ("C", "E", *FIXED))


def audit_motion(env, audit, x0, q0):
    endpoint, contact = audit.check(env._x, env._q)
    swept, swept_contact = swept_clearance(audit, x0, q0, env._x, env._q, subdivisions=5)
    residual = env.fk.error(env._x, env._q)
    translation, rotation = closure_errors(residual)
    velocity = (env._q - q0) / env.dt
    margin = float(np.min(np.minimum(env._q - env.lookup.joint_lower.ravel(),
                                    env.lookup.joint_upper.ravel() - env._q)))
    reasons = []
    if not all(np.isfinite(v).all() for v in [env._x, env._q, residual, velocity]):
        reasons.append("nonfinite_state")
    if min(endpoint, swept) < RULES["clearance_m"] - 1e-9:
        reasons.append("clearance")
    if contact or swept_contact:
        reasons.append("contact")
    if np.max(np.abs(velocity)) > RULES["joint_speed_rad_s"] + 2e-5:
        reasons.append("joint_speed")
    if margin < -1e-9:
        reasons.append("joint_limit")
    if np.max(translation) > RULES["closure_translation_m"] or np.max(rotation) > np.deg2rad(RULES["closure_rotation_deg"]):
        reasons.append("closure_geometry")
    return dict(violations=reasons, endpoint_clearance_m=endpoint, swept_clearance_m=swept,
        endpoint_contact=contact, swept_contact=swept_contact, closure_translation_m=translation,
        closure_rotation_rad=rotation, joint_margin_rad=margin, joint_velocity_rad_s=velocity)


def scene_labels(env, last_clearance):
    lower, upper = env.lookup.joint_lower.ravel(), env.lookup.joint_upper.ravel()
    near = min(np.min(env._q-lower), np.min(upper-env._q)) < RULES["boundary_joint_margin_rad"]
    near |= last_clearance < env.d_safe + RULES["boundary_clearance_margin_m"]
    i = int(env._u)
    v = env._traj.get("xdot", np.diff(env._traj["x"], axis=0)/env.traj_dt)
    radius = int(RULES["reversal_window_s"]/env.traj_dt)
    window = v[max(0, i-radius):min(len(v), i+radius+1)]
    reversal = bool(np.any(window[1:]*window[:-1] < 0))
    labels = (["boundary"] if near else []) + (["reversal"] if reversal else [])
    return labels or ["regular"]


def rollout(env, record, method, sink, repeat=0, policy=None, snapshots=None, max_steps=None):
    env._model_records = [record]
    audit = CollisionChecker(record["model"])
    steps = []
    failures = []
    termination = "initialization_invalid"
    last_clearance = record["initial_clearance_m"]
    duration = 0.
    try:
        env.reset(seed=20261125, options={"trajectory_index": 0})
        duration = (len(env._traj["x"])-1)*env.traj_dt
        budget = int(np.floor(RULES["simulation_duration_ratio_max"]*duration/env.dt + 1e-8))
        if max_steps is not None:
            budget = min(budget, max_steps)
        if record["valid"]:
            termination = "simulation_duration_limit"
            for index in range(budget):
                before = snapshot(env)
                labels = scene_labels(env, last_clearance)
                if snapshots is not None:
                    for label in labels:
                        if label not in snapshots:
                            snapshots[label] = dict(case=record["case"], record=record, state=before)
                start = time.perf_counter_ns()
                result, direct, first = env.method_step(method, policy)
                elapsed_ms = (time.perf_counter_ns()-start)/1e6
                _, _, terminated, truncated, info = result
                executed = info["beta_executed"] > 0
                audit_result = audit_motion(env, audit, before["_x"], before["_q"]) if executed else None
                if audit_result:
                    last_clearance = min(audit_result["endpoint_clearance_m"], audit_result["swept_clearance_m"])
                    failures.extend(audit_result["violations"])
                row = dict(case=record["case"], method=method, repeat=repeat, step=index,
                    scene=labels, elapsed_ms=elapsed_ms, direct_accepted=direct,
                    before=before, after=snapshot(env), info=info, audit=audit_result,
                    direct_trial=env._json_safe_candidate(first) if first is not None else None)
                if sink is not None:
                    sink.write(json.dumps(safe(row), allow_nan=False)+"\n")
                steps.append(dict(elapsed_ms=elapsed_ms, direct=direct, beta=info["beta_executed"], scenes=labels))
                if failures:
                    termination = "audit_failed"
                    break
                if terminated or truncated:
                    termination = info["termination"]
                    break
            if max_steps is not None and termination == "simulation_duration_limit":
                termination = "warmup_or_diagnostic"
    except Exception as error:
        termination = "exception"
        failures.append(type(error).__name__+": "+str(error))
        if sink is not None:
            sink.write(json.dumps(safe(dict(case=record["case"], method=method, repeat=repeat,
                termination=termination, failure_traceback=traceback.format_exc(), state=snapshot(env))), allow_nan=False)+"\n")
    times = [s["elapsed_ms"] for s in steps]
    over = np.array(times) > RULES["cycle_reference_ms"]
    longest = current = 0
    for value in over:
        current = current+1 if value else 0
        longest = max(longest, current)
    return dict(case=record["case"], method=method, repeat=repeat,
        completed=termination == "end" and not failures, termination=termination,
        failures=failures, steps=len(steps), nominal_duration_s=duration,
        simulation_duration_s=len(steps)*env.dt, progress=float(env._u/max(len(env._traj["x"])-1, 1)),
        cycle_ms=stats(times), total_compute_ms=sum(times), reference_5ms_exceeded=int(over.sum()),
        longest_reference_5ms_exceedance_run=longest,
        direct_steps=sum(s["direct"] is True for s in steps), fallback_steps=sum(s["direct"] is False for s in steps),
        slowed_steps=sum(s["beta"] < 1 for s in steps),
        scenes={label: stats([s["elapsed_ms"] for s in steps if label in s["scenes"]]) for label in ("regular", "reversal", "boundary")})


def profile_scenes(env, snapshots, count=20):
    results = []
    for label, saved in snapshots.items():
        env._model_records = [saved["record"]]
        env.reset(seed=20261125, options={"trajectory_index": 0})
        restore(env, saved["state"])
        good, bad = [], []
        for a in np.linspace(-1., 1., 5):
            for b in np.linspace(-1., 1., 5):
                raw = np.array([a, b])
                c, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                    raw*env.r_max, 1., env._previous_n, structured=False)
                (good if c["feasible"] else bad).append(raw)
        actions = {"direct_pass": (good[0], True) if good else None,
                   "direct_reject_then_fallback": (bad[0], True) if bad else None,
                   "project_same_passing_action": (good[0], False) if good else None}
        for name, option in actions.items():
            if option is None:
                results.append(dict(scene=label, path=name, available=False))
                continue
            raw, direct = option
            meter = Meter()
            measurements = []
            for sample in range(count+3):
                restore(env, saved["state"])
                meter.reset()
                with instrumentation(env, meter), meter.span("cycle"):
                    result, passed, _ = env.action_step(raw, direct)
                if sample >= 3:
                    measurements.append(dict(total_ms=1000*meter.inclusive["cycle"],
                        exclusive_ms={k: 1000*v for k,v in meter.exclusive.items()}, calls=dict(meter.counts),
                        direct_accepted=passed, accepted=result[-1]["beta_executed"] > 0))
            results.append(dict(scene=label, case=saved["case"], path=name, available=True,
                raw=raw, state=saved["state"], cycle_ms=stats([m["total_ms"] for m in measurements]), samples=measurements))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--methods", nargs="+", default=["B", "C"])
    parser.add_argument("--cases", type=int, default=12)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--cpu", type=int, default=2)
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args()
    if any(m not in ("A", "B", "C", *FIXED) for m in args.methods):
        parser.error("D/E require an explicitly frozen trained policy; unavailable here")
    os.sched_setaffinity(0, {args.cpu})
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    dump(out/"rules.json", RULES)
    data = ROOT/"hil-serl/data/aviator"
    base = ROOT/"reference/rocos-mujoco/model/aviator.xml"
    trajectories = sorted((data/"trajectory_source/trajs/val").glob("*.npz"))[:3]
    env = EvaluationEnv(str(data/"manifold_phi_stale_pitch-10deg"), str(base), str(data/"trajectory_source"))
    models = list(build_models(base, out))
    records = initialize(env, trajectories, models)[:args.cases]
    sources = [Path(__file__), ROOT/"hil-serl/tools/gfr1_remount_pilot.py",
        ROOT/"hil-serl/tools/gfr1_repaired_env.py", ROOT/"hil-serl/tools/gfr1_recovery_boundary_ablation.py",
        ROOT/"hil-serl/tools/gfr1_clearance_variants.py", ROOT/"hil-serl/tools/validate_branch_connector.py",
        ROOT/"hil-serl/tools/validate_online_lift.py", ROOT/"hil-serl/tools/audit_static_collision.py",
        ROOT/"hil-serl/examples/experiments/aviator_manifold/gfr_env.py"]
    dump(out/"manifest.json", dict(records=records, methods=args.methods, repeats=args.repeats,
        rules_sha256=sha(out/"rules.json"), source_sha256={str(p):sha(p) for p in sources},
        model_sha256={str(p):sha(p) for _,p,_ in models}, trajectory_sha256={str(p):sha(p) for p in trajectories},
        cpu_affinity=sorted(os.sched_getaffinity(0)), timing="serial workload, next-observation construction charged to current cycle; audit and logging excluded"))
    episodes, snapshots = [], {}
    with use_projector("combined"), (out/"steps.jsonl").open("x", buffering=1) as sink:
        for method in args.methods:
            warm = rollout(env, records[0], method, None, max_steps=50)
            if warm["failures"]:
                dump(out/"warmup_failure.json", warm)
                raise RuntimeError("Warmup failed; inspect saved record")
        stopped = False
        for repeat in range(args.repeats):
            for index, record in enumerate(records):
                order = args.methods[(index+repeat)%len(args.methods):]+args.methods[:(index+repeat)%len(args.methods)]
                for method in order:
                    row = rollout(env, record, method, sink, repeat, snapshots=snapshots)
                    episodes.append(row)
                    dump(out/"episodes.json", episodes)
                    print(json.dumps(safe(row), allow_nan=False), flush=True)
                # Finish the paired case, then stop dependent work if a prerequisite failed.
                if any(not e["completed"] for e in episodes if e["method"] in ("B", "C")):
                    stopped = True
                    break
            if stopped:
                break
        dump(out/"scene_snapshots.json", snapshots)
        if args.profile and not stopped:
            dump(out/"scene_profiles.json", profile_scenes(env, snapshots))
    dump(out/"gate.json", dict(passed=not stopped and len(episodes)==len(records)*len(args.methods)*args.repeats,
        baseline_gate_failed=stopped, allocated_episodes=len(records)*len(args.methods)*args.repeats,
        executed_episodes=len(episodes), pending_episodes=len(records)*len(args.methods)*args.repeats-len(episodes),
        tracking_not_evaluated=True, hard_5ms_gate=False))
    env.close()


if __name__ == "__main__":
    main()
