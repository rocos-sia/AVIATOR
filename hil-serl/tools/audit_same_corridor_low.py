"""Keep the feasible high reference's start, slide and theta corridor fixed.

Arm A repeats its exact ordered steering waypoints with randomized slower
timing. Arm B changes the ordered steering waypoints inside the same corridor.
The frozen q(theta,s) field and physical checker are identical in both arms.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_field import interpolate_q
from tools.audit_random_steering import SHAPE, PEAKS, init_worker, audit_path
from tools.audit_static_cross_path import summarize


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def corridor_bounds(manifold_dir, field, theta_range, slide):
    """Conservative finite-difference bounds for fixed-slide table queries."""
    lookup = ManifoldLookup(manifold_dir)
    with np.load(field) as f:
        q_grid = f["q"]
    theta = lookup.theta_axis
    x = np.column_stack((theta, np.full(len(theta), slide)))
    q = interpolate_q(lookup, q_grid, x)
    slopes = np.diff(q, axis=0) / np.diff(theta)[:, None]
    lo, hi = theta_range
    relevant_segments = (theta[:-1] < hi) & (theta[1:] > lo)
    relevant_knots = (theta[1:-1] > lo) & (theta[1:-1] < hi)
    maximum_slope = float(np.max(np.abs(slopes[relevant_segments])))
    maximum_jump = float(np.max(np.abs(np.diff(slopes, axis=0)[relevant_knots])))
    audit_theta = np.r_[lo, theta[(theta > lo) & (theta < hi)], hi]
    audit_q = interpolate_q(lookup, q_grid,
                            np.column_stack((audit_theta, np.full(len(audit_theta), slide))))
    lower, upper = lookup.joint_lower.reshape(14), lookup.joint_upper.reshape(14)
    minimum_joint_margin = float(np.min(np.minimum(audit_q - lower, upper - audit_q)))
    # At 100 Hz, |Delta theta| <= 0.006 rad, below one grid cell's width.
    # Consecutive task samples therefore encounter at most one theta knot.
    assert .6 * .01 < float(np.min(np.diff(theta)))
    return dict(max_abs_dq_dtheta=maximum_slope,
                max_neighbor_slope_jump=maximum_jump,
                minimum_joint_limit_margin_rad=minimum_joint_margin,
                joint_speed_upper_bound=maximum_slope * .6,
                conservative_discrete_joint_acceleration_upper_bound=(
                    maximum_slope * 5. + maximum_jump * .6 / .01),
                task_theta_speed_bound=.6, task_theta_acceleration_bound=5., dt=.01,
                sufficient_task_acceleration_bound_at_theta_speed_0_6=(
                    10. - maximum_jump * .6 / .01) / maximum_slope,
                qdot_limit=1.5, qddot_limit=10.,
                interpretation="speed has an analytical bound in the fixed-slide corridor; acceleration bound at task a=5 is inconclusive; joint-limit margin is nearly zero and not robust")


def make_low(seed, mode, high_events, start, end, theta_range, s0):
    rng = np.random.default_rng(seed)
    dt = .01
    current = float(start)
    if mode == "same_waypoints":
        targets = [float(e["target_x"][0]) for e in high_events if "target_x" in e]
    elif mode == "random_route":
        # Preserve start/end and full theta corridor. Randomize the interior
        # turn locations without changing the wheel's allowed angular region.
        low, high = theta_range
        targets = [float(rng.uniform(-.15, .15)),
                   float(rng.uniform(low, low + .12)),
                   float(rng.uniform(high - .12, high)),
                   float(rng.uniform(low + .12, -.12)),
                   float(rng.uniform(.12, high - .12)), float(end)]
    else:
        raise ValueError(mode)
    assert len(targets) == 6
    x_parts = [np.array([current])]
    v_parts = [np.zeros(1)]
    a_parts = [np.zeros(1)]
    j_parts = [np.zeros(1)]
    events = []

    def hold():
        n = int(round(rng.uniform(.10, .60) / dt))
        x_parts.append(np.full(n, current))
        for parts in (v_parts, a_parts, j_parts):
            parts.append(np.zeros(n))
        events.append(dict(kind="hold", n=n))

    hold()
    for target in targets:
        delta = target - current
        # Low-speed task envelope matches the previous random steering audit.
        minimum = max(.25, PEAKS[0] * abs(delta) / rng.uniform(.32, .60),
                      np.sqrt(PEAKS[1] * abs(delta) / 5.),
                      np.cbrt(PEAKS[2] * abs(delta) / 20.))
        n = int(np.ceil(minimum * rng.uniform(1., 1.45) / dt))
        duration = n * dt
        u = np.arange(1, n + 1) / n
        x_parts.append(current + delta * SHAPE(u))
        v_parts.append(delta * SHAPE.deriv(1)(u) / duration)
        a_parts.append(delta * SHAPE.deriv(2)(u) / duration**2)
        j_parts.append(delta * SHAPE.deriv(3)(u) / duration**3)
        events.append(dict(kind="turn", start_theta=current, end_theta=target,
                           duration=duration))
        current = target
        if rng.random() < .75:
            hold()
    hold()
    theta, v, a, j = [np.concatenate(parts) for parts in (x_parts, v_parts, a_parts, j_parts)]
    assert theta_range[0] - 1e-12 <= theta.min() <= theta.max() <= theta_range[1] + 1e-12
    assert np.max(abs(v)) <= .6 + 1e-12
    assert np.max(abs(a)) <= 5. + 1e-12
    assert np.max(abs(j)) <= 20. + 1e-12
    n = len(theta)
    zero = np.zeros(n)
    return dict(t=np.arange(n) * dt,
                x=np.column_stack((theta, np.full(n, s0))),
                xdot=np.column_stack((v, zero)),
                xddot=np.column_stack((a, zero)),
                xdddot=np.column_stack((j, zero))), dict(
                    mode=mode, seed=seed, events=events,
                    duration=(n - 1) * dt,
                    theta_peak=float(np.max(abs(v))),
                    theta_acceleration_peak=float(np.max(abs(a))),
                    theta_jerk_peak=float(np.max(abs(j))))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--archive", type=Path, required=True)
    ap.add_argument("--feasible-audit", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--seed", type=int, default=2026093008)
    ap.add_argument("--per-arm", type=int, default=30)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    args = ap.parse_args()
    if args.output.exists() or args.per_arm < 1 or args.workers < 1:
        ap.error("need new output directory and positive counts")
    prior = json.loads((args.feasible_audit / "summary.json").read_text())
    reference_id = "traj_0047"
    reference = args.archive / "trajs" / (reference_id + ".npz")
    archived = json.loads((args.archive / "episodes.json").read_text())[reference_id]
    assert sha(reference) == archived["sha256"]
    assert prior["summary"]["high"]["pass_with_acceleration"] == 1
    field = Path(prior["field"])
    assert sha(field) == prior["field_sha256"]
    init_worker(args.manifold_dir, args.model, str(field))
    high_result = audit_path(str(reference))
    assert high_result["pass_with_acceleration"]
    with np.load(reference) as f:
        x = f["x"]
    start, end, s0 = float(x[0, 0]), float(x[-1, 0]), float(x[0, 1])
    corridor = [float(x[:, 0].min()), float(x[:, 0].max())]
    assert np.allclose(x[:, 1], s0)
    args.output.mkdir(parents=True)
    task_dir = args.output / "trajs"
    task_dir.mkdir()
    rows = {}
    for mode in ("same_waypoints", "random_route"):
        for _ in range(args.per_arm):
            index = len(rows)
            name = f"traj_{index:04d}"
            trajectory, meta = make_low(args.seed + index, mode, archived["events"],
                                        start, end, corridor, s0)
            np.testing.assert_allclose(trajectory["x"][[0, -1]], x[[0, -1]], atol=1e-12)
            file = task_dir / (name + ".npz")
            np.savez_compressed(file, **trajectory)
            rows[name] = dict(**meta, sha256=sha(file))
    manifest = dict(seed=args.seed, reference_path=str(reference.resolve()),
                    reference_sha256=sha(reference), high_metrics=high_result,
                    field_sha256=sha(field), start_x=x[0].tolist(), end_x=x[-1].tolist(),
                    theta_corridor=corridor, slide=s0,
                    task_limits=dict(theta_speed=.6, theta_acceleration=5., theta_jerk=20.),
                    trajectories=rows)
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"generated {len(rows)} low paths in the high reference corridor", flush=True)
    completed = {}
    with ProcessPoolExecutor(max_workers=args.workers, initializer=init_worker,
                             initargs=(args.manifold_dir, args.model, str(field))) as pool:
        future_to_name = {pool.submit(audit_path, str(task_dir / (name + ".npz"))): name
                          for name in rows}
        for future in as_completed(future_to_name):
            name = future_to_name[future]
            completed[name] = dict(**rows[name], metrics=future.result())
            (args.output / "episodes.json").write_text(json.dumps(dict(sorted(completed.items())), indent=2) + "\n")
            if len(completed) % 5 == 0:
                print(f"audited {len(completed)}/{len(rows)}", flush=True)
    report = dict(reference=dict(path=str(reference.resolve()), sha256=sha(reference), metrics=high_result),
                  field=str(field), field_sha256=sha(field),
                  fixed_start=x[0].tolist(), fixed_end=x[-1].tolist(), fixed_slide=s0,
                  theta_corridor=corridor,
                  total=summarize(completed.values()),
                  by_arm={mode:summarize([r for r in completed.values() if r["mode"]==mode])
                          for mode in ("same_waypoints", "random_route")},
                  limits=dict(qdot=1.5, qddot=10, task_theta_speed=.6,
                              task_theta_acceleration=5., task_theta_jerk=20.,wall_clearance=.005),
                  caveats=["A finite randomized test does not prove all admissible low paths.",
                           "Same-waypoints arm changes timing only; random-route arm changes waypoint geometry in the same corridor.",
                           "Both arms fix the table, start/end, slide coordinate, and allowed theta interval.",
                           "Sample/midpoint geometry and finite-difference dynamics only; no torque/tracking proof."])
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    bounds = corridor_bounds(args.manifold_dir, field, corridor, s0)
    (args.output / "corridor_bounds.json").write_text(json.dumps(bounds, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
