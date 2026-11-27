"""Offline full-speed teacher and exact direct-action replay audit.

QP and finite candidate search are teacher-only. Every chosen float32 label is
checked and executed again through DirectOnlyEnv at beta=1. No slowed labels,
no repaired next-state labels, and no partial episode counted as success.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
from pathlib import Path
import time
import numpy as np

from tools.audit_static_collision import CollisionChecker
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_remount_evaluate import ROOT, dump, audit_motion, sha
from tools.gfr2_shared_env import SharedProposalEnv


def make_env(record):
    data = ROOT/"hil-serl/data/aviator"
    env = SharedProposalEnv(str(data/"manifold_phi_stale_pitch-10deg"), record["model"], str(data/"trajectory_source"))
    env._model_records = [record]
    env.reset(seed=11, options={"trajectory_index": 0})
    return env


def choose_teacher(env, step, force_search=False):
    """Deterministic feasibility/margin teacher, not a claimed horizon optimizer."""
    previous = np.clip(env._r_prev/env.r_max, -1., 1.)
    proposals, valid = [], []
    evaluations = 0

    def check(action):
        nonlocal evaluations
        action = np.asarray(action, np.float32)
        if any(np.array_equal(action, old) for old in proposals):
            return
        proposals.append(action.copy())
        result, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                                   action*env.r_max, 1., env._previous_n, structured=False)
        evaluations += 1
        if result["feasible"]:
            clearance = min((result["d_actual_m"]-env.d_safe)/.001, 2.)
            margin = min(result["joint_limit_margin_rad"]/.1, 2.)
            score = min(clearance, margin)-.015*float(np.sum((action-previous)**2))-.002*float(action@action)
            valid.append((score, action, result))
        return result

    first = check(previous)
    near = first["d_actual_m"] < env.d_safe+.0005 or first["joint_limit_margin_rad"] < .1
    if first["feasible"] and not force_search and not (near and step % 20 == 0):
        return previous.astype(np.float32), dict(evaluations=evaluations, search=False)
    # The existing combined projector proposes a teacher label. It is never
    # trusted directly: recheck its normalized float32 action without projection.
    projected, _ = env._candidate(env._x, env._q, env._path_state(env._u)[1],
                                 previous*env.r_max, 1., env._previous_n, structured=True)
    evaluations += 1
    if projected.get("feasible"):
        check(np.clip(np.asarray(projected["r_exec"])/env.r_max, -1., 1.))
    for left in np.linspace(-1., 1., 5):
        for right in np.linspace(-1., 1., 5):
            check([left, right])
    if not valid:
        return None, dict(evaluations=evaluations, search=True, failure="teacher_no_full_speed_candidate",
                          first_trial=env._json_safe_candidate(first))
    best = max(valid, key=lambda item: item[0])
    return best[1], dict(evaluations=evaluations, search=True, feasible_candidates=len(valid))


def collect_case(record, output):
    out = Path(output)
    name = record["case"].replace("/", "__")
    env = make_env(record)
    audit = CollisionChecker(record["true_model"])
    observations, actions, states, costs = [], [], [], []
    start = time.perf_counter()
    termination, violations = "initialization_invalid", []
    try:
        if record["valid"]:
            budget = int(np.ceil((len(env._traj["x"])-1)*env.traj_dt/env.dt))+1
            with use_projector("combined"), (out/f"{name}.jsonl").open("x", buffering=1) as log:
                for step in range(budget):
                    obs = env.last_policy_observation.copy()
                    x, q = env._x.copy(), env._q.copy()
                    action, selection = choose_teacher(env, step)
                    if action is None:
                        termination = selection["failure"]
                        log.write(json.dumps(dict(step=step, termination=termination, **selection))+"\n")
                        break
                    _, _, done, truncated, info = env.step(action)
                    if not env._last_direct_accepted:
                        raise RuntimeError("Teacher float32 action failed direct replay")
                    if info["beta_executed"] != 1 or info["projection_calls"] or info["recovery_calls"]:
                        raise AssertionError("Corrective teacher label contaminated direct replay")
                    verdict = audit_motion(env, audit, x, q)
                    violations = verdict["violations"]
                    observations.append(obs)
                    actions.append(action)
                    states.append(np.r_[x, q])
                    costs.append(selection["evaluations"])
                    termination = info["termination"]
                    log.write(json.dumps(dict(step=step, progress=info["progress"], action=action.tolist(),
                        beta=info["beta_executed"], clearance_m=verdict["swept_clearance_m"],
                        joint_margin_rad=verdict["joint_margin_rad"], audit_violations=violations,
                        teacher_selection=selection))+"\n")
                    if violations:
                        termination = "independent_audit_failed"
                        break
                    if done or truncated:
                        break
                else:
                    termination = "step_budget"
        result = dict(case=record["case"], completed=termination == "end" and not violations,
            termination=termination, audit_violations=violations, steps=len(actions),
            progress=float(env._u/(len(env._traj["x"])-1)), elapsed_s=time.perf_counter()-start,
            teacher_candidate_evaluations=sum(costs), replay_projection_calls=0, replay_recovery_calls=0,
            dataset_file=str(out/f"{name}.npz"), model_sha256=sha(record["model"]))
        np.savez_compressed(out/f"{name}.npz", observations=np.asarray(observations, np.float32).reshape(-1,128),
            actions=np.asarray(actions,np.float32).reshape(-1,2), states=np.asarray(states).reshape(-1,16))
        dump(out/f"{name}.summary.json", result)
        return result
    finally:
        env.close()


def collect(manifest, output, workers=4):
    out = Path(output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    records = json.loads(Path(manifest).read_text())["records"]
    rows = []
    dump(out/"protocol.json", dict(manifest=str(manifest), manifest_sha256=sha(manifest),
        teacher="previous action / combined QP / finite full-speed margin search",
        teacher_lookahead=False, beta=1, workers=workers, source_sha256=sha(__file__),
        gate="all allocated training episodes must complete exact direct replay and independent audit"))
    with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
        futures = {pool.submit(collect_case, record, str(out)): record for record in records}
        for future in as_completed(futures):
            try:
                row = future.result()
            except Exception as exc:
                row = dict(case=futures[future]["case"], completed=False, termination="exception", error=repr(exc))
            rows.append(row)
            dump(out/"episodes.json", rows)
            print(json.dumps(row), flush=True)
    result = dict(passed=len(rows)==len(records) and all(r["completed"] for r in rows),
        completed=sum(r["completed"] for r in rows), cases=len(records),
        transitions=sum(r.get("steps",0) for r in rows), failures=[r for r in rows if not r["completed"]])
    dump(out/"gate.json", result)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--workers", type=int, default=4)
    a = p.parse_args()
    collect(a.manifest, a.output, a.workers)
