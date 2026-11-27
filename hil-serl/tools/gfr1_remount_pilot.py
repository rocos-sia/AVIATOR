"""Run paired deterministic portions of the remount validation protocol."""
from __future__ import annotations

import argparse
import json
import os
import platform
import time
from pathlib import Path

import mujoco
import numpy as np

from experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.audit_static_collision import CollisionChecker
from tools.gfr1_build_consistent_model_manifest import repair_q
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
from tools.gfr1_same_state_diagnosis import digest, write_json
from tools.run_n16_installation_error import make_variant
from tools.run_n8_dt_heldout import GOVERNOR_SCHEDULE
from tools.validate_branch_connector import connector_step_y, swept_clearance

ROOT = Path(__file__).resolve().parents[2]
TRANSLATION_LIMIT = 0.0005
ROTATION_LIMIT = np.deg2rad(0.25)


def closure_errors(residual):
    residual = np.asarray(residual).reshape(2, 6)
    return np.linalg.norm(residual[:, :3], axis=1), np.linalg.norm(residual[:, 3:], axis=1)


def closure_ok(residual):
    translation, rotation = closure_errors(residual)
    return bool(np.all(translation <= TRANSLATION_LIMIT) and np.all(rotation <= ROTATION_LIMIT))


class PilotEnv(RepairedGFRManifoldEnv):
    def _candidate(self, *args, **kwargs):
        candidate, detail = super()._candidate(*args, **kwargs)
        if candidate["feasible"] and not closure_ok(candidate["e_next"]):
            candidate.update(feasible=False, reason="closure_tolerance")
        return candidate, detail

    def legacy_qp_step(self):
        """Apply the N8 schedule and 3D QP to the current trajectory tangent."""
        x, q = self._x, self._q
        velocity = self._path_state(self._u)[1]
        lower = np.concatenate(self.lookup.joint_lower)
        upper = np.concatenate(self.lookup.joint_upper)
        nominal = np.array([0.4, -0.4])
        candidates = []
        for task_scale, phase_scale in GOVERNOR_SCHEDULE:
            _, n, prediction, _, beta, r = connector_step_y(
                self.fk, self.checker, x, q, task_scale * velocity,
                phase_scale * nominal, self._previous_n, lower, upper,
                self.dt, beta_stop=self.beta_stop, clearance_margin_m=0.0005,
            )
            if not prediction["feasible"]:
                candidates.append(prediction)
                continue
            candidate, _ = self._candidate(x, q, velocity, r, task_scale * beta, n, structured=False)
            candidates.append(candidate)
            if candidate["feasible"]:
                result = GFRManifoldEnv.step(
                    self, nominal / self.r_max, {"accepted": candidate, "candidates": candidates}
                )
                return result, None
        return self._failed_step(nominal / self.r_max, candidates), None

    def previous_action_step(self, direct):
        raw = np.clip(self._r_prev / self.r_max, -1.0, 1.0)
        if direct:
            candidate, _ = self._candidate(
                self._x, self._q, self._path_state(self._u)[1], raw * self.r_max,
                1.0, self._previous_n, structured=False,
            )
            if candidate["feasible"]:
                result = GFRManifoldEnv.step(
                    self, raw, {"accepted": candidate, "candidates": [candidate]}
                )
                self._repair_cycle += 1
                self._repair_previous_raw = raw.astype(np.float32)
                return result, True
        return super().step(raw), False if direct else None


def build_models(base, destination):
    cases = [
        ("nominal", None, 0.0),
        ("fixture_translation", "fixture_translation", 0.001),
        ("base_translation", "base_translation", 0.001),
        ("base_rotation", "base_rotation", 0.25),
    ]
    for name, family, level in cases:
        if family is None:
            yield name, base, level
            continue
        model = destination / f"{name}.xml"
        make_variant(base, model, family, level)
        mesh_root = base.parent / "aviator_meshes"
        source = model.read_text()
        model.write_text(source.replace('meshdir="aviator_meshes"', f'meshdir="{mesh_root}"', 1))
        yield name, model, level


def initialize(env, trajectories, models):
    records = []
    lower = np.concatenate(env.lookup.joint_lower)
    upper = np.concatenate(env.lookup.joint_upper)
    for name, model, level in models:
        env._set_model(str(model))
        for path in trajectories:
            with np.load(path) as source:
                x = np.array(source["x"][0], dtype=float)
                if not np.allclose(np.diff(source["t"]), env.traj_dt):
                    raise ValueError(f"Unexpected trajectory sample interval: {path}")
            box = env.lookup.safe_interval(x[None, :])
            phase = 0.5 * (box["phi_safe_lo"] + box["phi_safe_hi"])
            query = env.lookup.query(x[None, :], phase, check_safe=False)
            q, _ = repair_q(env.fk, x, np.r_[query["qL"][0], query["qR"][0]], lower, upper)
            residual = env.fk.error(x, q)
            clearance, contact = env.checker.check(x, q)
            valid = bool(
                np.all(np.isfinite(q)) and np.max(np.abs(residual)) <= 1e-7
                and np.all(q >= lower) and np.all(q <= upper)
                and clearance >= env.d_safe and not contact
            )
            records.append(dict(
                case=f"{name}/{path.stem}", model=str(model), level=level,
                trajectory_path=str(path), initial_q=q, initial_x=x,
                initial_clearance_m=clearance, initial_closure_residual_inf=np.max(np.abs(residual)),
                valid=valid,
            ))
    return records


def compare_initial_bases(env, records):
    nominal = {}
    comparison = []
    for record in records:
        env._set_model(record["model"])
        x, q = record["initial_x"], record["initial_q"]
        _, basis, _, _, sigma = env._lift(x, q, np.zeros(2), None)
        trajectory = record["trajectory_path"]
        if record["case"].startswith("nominal/"):
            nominal[trajectory] = basis
            continue
        cross_basis = nominal[trajectory].T @ basis
        comparison.append(dict(
            case=record["case"], valid_initial_state=record["valid"], raw_basis_overlap=cross_basis,
            principal_angle_deg=np.rad2deg(np.arccos(np.clip(np.linalg.svd(cross_basis, compute_uv=False), 0, 1))),
            smallest_closure_singular_value=sigma,
        ))
    return comparison


def rollout(env, index, method, record, audit, step_sink, max_steps=0):
    env.reset(seed=20261125 + index, options={"trajectory_index": index})
    steps = []
    maximum_position, maximum_rotation = np.zeros(2), np.zeros(2)
    minimum_clearance = record["initial_clearance_m"]
    progress = 0.0
    reference_end = env._traj["x"][-1]
    termination = "initialization_invalid"
    if record["valid"]:
        termination = "step_budget"
        budget = int(np.ceil((len(env._traj["x"]) - 1) * env.traj_dt / (env.dt * env.beta_stop))) + 2
        for _ in range(min(budget, max_steps) if max_steps else budget):
            old_x, old_q = env._x.copy(), env._q.copy()
            start = time.perf_counter_ns()
            result, direct = env.legacy_qp_step() if method == "A" else env.previous_action_step(method == "C")
            elapsed_ms = (time.perf_counter_ns() - start) / 1e6
            _, _, terminated, truncated, info = result
            progress = info["progress"]
            executed = info["beta_executed"] > 0
            if executed:
                endpoint, contact = audit.check(env._x, env._q)
                swept, swept_contact = swept_clearance(audit, old_x, old_q, env._x, env._q)
                error = env.fk.error(env._x, env._q)
                speed = np.max(np.abs(env._q - old_q)) / env.dt
                margin = np.min(np.minimum(env._q - env.lookup.joint_lower.ravel(),
                                           env.lookup.joint_upper.ravel() - env._q))
                if (not closure_ok(error) or not np.isfinite(speed)
                        or speed > env.qdot_max + 2e-5 or margin < -1e-9
                        or min(endpoint, swept) < env.d_safe - 1e-9 or contact or swept_contact):
                    raise RuntimeError(f"Independent audit rejected {record['case']} method {method}")
                translation, rotation = closure_errors(error)
                maximum_position = np.maximum(maximum_position, translation)
                maximum_rotation = np.maximum(maximum_rotation, rotation)
                minimum_clearance = min(minimum_clearance, endpoint, swept)
            row = dict(
                step=len(steps), case=record["case"], method=method, elapsed_ms=elapsed_ms,
                direct_accepted=direct, executed=executed, progress=progress, beta=info["beta_executed"],
                candidates=info.get("actual_candidate_evaluations", info["candidate_count"]) + (direct is False),
            )
            steps.append(row)
            if step_sink is not None:
                step_sink.write(json.dumps(row) + "\n")
            if terminated or truncated:
                termination = info["termination"]
                break
    timing = np.array([step["elapsed_ms"] for step in steps])
    percentiles = np.percentile(timing, [50, 95, 99]).tolist() if len(timing) else [None] * 3
    return dict(
        case=record["case"], method=method, steps=len(steps), completed=termination == "end",
        termination=termination, progress=progress, total_compute_ms=float(timing.sum()),
        p50_ms=percentiles[0], p95_ms=percentiles[1], p99_ms=percentiles[2],
        deadline_5ms_exceeded=int(np.count_nonzero(timing > 5.0)),
        direct_steps=sum(step["direct_accepted"] is True for step in steps),
        fallback_steps=sum(step["direct_accepted"] is False for step in steps),
        candidates=sum(step["candidates"] for step in steps),
        audited_steps=sum(step["executed"] for step in steps), min_clearance_m=minimum_clearance,
        max_closure_mm=(maximum_position * 1000).tolist(), max_closure_deg=np.rad2deg(maximum_rotation).tolist(),
        final_task_error=np.abs(reference_end - env._x).tolist(),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--methods", nargs="+", choices=("A", "B", "C"), default=("A", "B", "C"))
    parser.add_argument("--cases", type=int, default=12)
    parser.add_argument("--case-offset", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=0, help="Diagnostic only")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    data = ROOT / "hil-serl/data/aviator"
    base = ROOT / "reference/rocos-mujoco/model/aviator.xml"
    trajectory_dir = data / "trajectory_source"
    paths = sorted((trajectory_dir / "trajs/val").glob("*.npz"))[:3]
    models = list(build_models(base, output))
    env = PilotEnv(str(data / "manifold_phi_stale_pitch-10deg"), str(base), str(trajectory_dir))
    records = initialize(env, paths, models)
    basis_comparison = compare_initial_bases(env, records)
    env._model_records = records
    indices = range(args.case_offset, min(len(records), args.case_offset + args.cases))
    write_json(output / "manifest.json", dict(
        records=[records[index] for index in indices], methods=args.methods, max_steps=args.max_steps,
        initial_basis_comparison=basis_comparison, mujoco=mujoco.__version__,
        cpu_affinity=sorted(os.sched_getaffinity(0)), runner_sha256=digest(Path(__file__)),
        platform=platform.platform(), initialization_residual_limit=1e-7,
        accepted_closure_translation_m=TRANSLATION_LIMIT, accepted_closure_rotation_deg=0.25,
        dt=env.dt, d_safe=env.d_safe, beta_stop=env.beta_stop, qdot_max=env.qdot_max,
        model_sha256={str(path): digest(path) for _, path, _ in models},
        trajectory_sha256={str(path): digest(path) for path in paths},
    ))
    audits = {str(path): CollisionChecker(str(path)) for _, path, _ in models}
    results = []
    with use_projector("combined"), (output / "steps.jsonl").open("x", buffering=1) as steps, (
        output / "episodes.jsonl"
    ).open("x", buffering=1) as episodes:
        warm_record = next((index for index in indices if records[index]["valid"]), None)
        if warm_record is not None:
            record = records[warm_record]
            for method in args.methods:
                rollout(env, warm_record, method, record, audits[record["model"]], None, 100)
        for index in indices:
            record = records[index]
            rotation = index % len(args.methods)
            order = args.methods[rotation:] + args.methods[:rotation]
            for method in order:
                row = rollout(env, index, method, record, audits[record["model"]], steps, args.max_steps)
                results.append(row)
                episodes.write(json.dumps(row, allow_nan=False) + "\n")
                print(json.dumps(row, allow_nan=False), flush=True)
    env.close()
    write_json(output / "results.json", {"episodes": results})


if __name__ == "__main__":
    main()
