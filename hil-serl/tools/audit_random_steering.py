"""Synthetic intermittent steering: random submovements, pauses and reversals.

This is an engineering stress test, not an empirically fitted human model.
Seventh-order segments join at rest with continuous first three derivatives.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.audit_static_field import interpolate_q
from tools.audit_static_slowdown import geometry, metrics
from tools.audit_static_cross_path import summarize


SHAPE = Polynomial([0, 0, 0, 0, 35, -84, 70, -20])


def derivative_peak(order):
    derivative = SHAPE.deriv(order)
    roots = derivative.deriv().roots()
    interior = [float(r.real) for r in roots if abs(r.imag) < 1e-10 and 0 < r.real < 1]
    return float(np.max(np.abs(derivative(np.r_[0., interior, 1.]))))


PEAKS = np.array([derivative_peak(k) for k in (1, 2, 3)])


def make_path(seed, speed_cap, mode):
    rng = np.random.default_rng(seed)
    dt = .01
    current = np.array([rng.uniform(-.25, .25), rng.uniform(-.11, -.05)])
    initial = current.copy()
    direction = rng.choice([-1., 1.])
    targets = [
        ("small_correction", np.clip(current[0] + rng.uniform(-.10, .10), -.72, .72), rng.uniform(.18, .4)),
        ("large_turn", direction * rng.uniform(.55, .72), rng.uniform(.65, 1.)),
        ("rapid_countersteer", -direction * rng.uniform(.55, .72), rng.uniform(.85, 1.)),
        ("return_toward_center", rng.uniform(-.18, .18), rng.uniform(.3, .65)),
        ("small_correction", rng.uniform(-.25, .25), rng.uniform(.18, .4)),
        ("settle", rng.uniform(-.08, .08), rng.uniform(.3, .7)),
    ]
    positions = [current[None, :]]
    velocities, accelerations, jerks = ([np.zeros((1, 2))] for _ in range(3))
    events = []
    elapsed = 0.

    def hold(duration):
        nonlocal elapsed
        n = int(round(duration / dt))
        if n:
            positions.append(np.tile(current, (n, 1)))
            for parts in (velocities, accelerations, jerks):
                parts.append(np.zeros((n, 2)))
            events.append(dict(kind="hold", start=elapsed, end=elapsed + n * dt))
            elapsed += n * dt

    hold(rng.uniform(.15, .45))
    for kind, theta, speed_fraction in targets:
        slide = initial[1] if mode == "steering_only" else float(np.clip(
            current[1] + rng.uniform(-.025, .025), -.14, -.02))
        target = np.array([theta, slide])
        delta = target - current
        v_limit = np.array([speed_cap * speed_fraction, .04])
        a_limit = np.array([5., 2.])
        j_limit = np.array([20., 10.])
        minimum = max(.25, np.max(PEAKS[0] * abs(delta) / v_limit),
                      np.max(np.sqrt(PEAKS[1] * abs(delta) / a_limit)),
                      np.max(np.cbrt(PEAKS[2] * abs(delta) / j_limit)))
        steps = int(np.ceil(minimum / dt))
        duration = steps * dt
        u = np.arange(1, steps + 1) / steps
        positions.append(current + SHAPE(u)[:, None] * delta)
        for order, parts in enumerate((velocities, accelerations, jerks), 1):
            parts.append(SHAPE.deriv(order)(u)[:, None] * delta / duration**order)
        events.append(dict(kind=kind, start=elapsed, end=elapsed + duration,
                           initial_x=current.tolist(), target_x=target.tolist(),
                           theta_peak=float(PEAKS[0] * abs(delta[0]) / duration)))
        elapsed += duration
        current = target
        if rng.random() < .65:
            hold(rng.uniform(.10, .65))
    hold(.25)
    x, xd, xdd, xddd = map(np.concatenate, (positions, velocities, accelerations, jerks))
    assert np.all(x >= [-.72 - 1e-12, -.14 - 1e-12])
    assert np.all(x <= [.72 + 1e-12, -.02 + 1e-12])
    for values, limits in ((xd, [speed_cap, .04]), (xdd, [5., 2.]), (xddd, [20., 10.])):
        assert np.all(np.max(abs(values), axis=0) <= np.asarray(limits) + 1e-8)
    # Ignore stationary samples when counting changes in movement direction.
    signs = np.sign(xd[abs(xd[:, 0]) > 1e-6, 0])
    reversals = int(np.count_nonzero(np.diff(signs)))
    return dict(t=np.arange(len(x)) * dt, x=x, xdot=xd, xddot=xdd, xdddot=xddd), dict(
        events=events, reversals=reversals,
        duration_s=(len(x) - 1) * dt,
        theta_peak=float(np.max(abs(xd[:, 0]))),
        task_derivative_peaks=[np.max(abs(a), axis=0).tolist() for a in (xd, xdd, xddd)])


def init_worker(manifold, model, field):
    global LOOKUP, CHECKER, GRID
    LOOKUP = ManifoldLookup(manifold)
    CHECKER = CollisionChecker(model)
    with np.load(field) as f:
        assert np.array_equal(f["theta"], LOOKUP.theta_axis)
        assert np.array_equal(f["s"], LOOKUP.s_axis)
        GRID = f["q"]


def audit_path(path):
    with np.load(path) as f:
        x = f["x"]
    q = interpolate_q(LOOKUP, GRID, x)
    assert np.isfinite(q).all()
    result = metrics(q, .01, geometry(CHECKER, LOOKUP, x, q))
    result["first_speed_failure_s"] = next((float((i + 1) * .01) for i in np.flatnonzero(
        np.any(abs(np.diff(q, axis=0) / .01) > 1.5, axis=1))), None)
    result["first_acceleration_failure_s"] = next((float((i + 2) * .01) for i in np.flatnonzero(
        np.any(abs(np.diff(q, n=2, axis=0) / .01**2) > 10, axis=1))), None)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--field", type=Path, default=Path("../outputs/static_field_audit/old_fixed_q_field_00_10.npz"))
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    ap.add_argument("--seed", type=int, default=2026093007)
    ap.add_argument("--per-cell", type=int, default=10)
    ap.add_argument("--workers", type=int, default=2)
    args = ap.parse_args()
    if args.output.exists() or args.per_cell < 1 or args.workers < 1:
        ap.error("need new output directory and positive counts")
    args.output.mkdir(parents=True)
    destination = args.output / "trajs"
    destination.mkdir()
    rows = {}
    for speed in (.6, 1., 1.5):
        for mode in ("steering_only", "steering_with_slide"):
            for _ in range(args.per_cell):
                index = len(rows)
                seed = args.seed + index
                data, meta = make_path(seed, speed, mode)
                name = f"traj_{index:04d}"
                target = destination / (name + ".npz")
                np.savez_compressed(target, **data)
                rows[name] = dict(seed=seed, mode=mode, speed_cap=speed, **meta,
                                  sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    manifest = dict(seed=args.seed, model="synthetic intermittent C3 submovements; not human recordings",
                    dt=.01, trajectories=rows, derivative_shape_peaks=PEAKS.tolist(),
                    task_limits=dict(theta_range=[-.72, .72], slide_range=[-.14, -.02],
                                     speed=[1.5,.04], acceleration=[5.,2.], jerk=[20.,10.]))
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"generated {len(rows)} random paths before collision auditing", flush=True)
    completed = {}
    with ProcessPoolExecutor(max_workers=args.workers, initializer=init_worker,
                             initargs=(args.manifold_dir, args.model, str(args.field))) as pool:
        futures = {pool.submit(audit_path, str(destination / (name + ".npz"))): name for name in rows}
        for future in as_completed(futures):
            name = futures[future]
            completed[name] = dict(**rows[name], metrics=future.result())
            (args.output / "episodes.json").write_text(json.dumps(dict(sorted(completed.items())), indent=2) + "\n")
            if len(completed) % 5 == 0:
                print(f"audited {len(completed)}/{len(rows)}", flush=True)
    report = dict(field=str(args.field.resolve()), field_sha256=hashlib.sha256(args.field.read_bytes()).hexdigest(),
                  summary=summarize(completed.values()),
                  by_speed={str(speed): summarize([r for r in completed.values() if r["speed_cap"] == speed])
                            for speed in (.6, 1., 1.5)},
                  by_cell={f"{speed}/{mode}": summarize([r for r in completed.values()
                           if r["speed_cap"] == speed and r["mode"] == mode])
                           for speed in (.6, 1., 1.5) for mode in ("steering_only", "steering_with_slide")},
                  limits=dict(qdot=1.5, qddot=10, wall_clearance=.005, contact_penetration=.001),
                  caveats=["Synthetic submovements, no empirical human validation.",
                           "Each trajectory has rest boundaries; no abrupt position/velocity jumps.",
                           "Frozen table, no retraining/retuning or failure rejection.",
                           "Points and joint-segment midpoints only, no continuous collision proof.",
                           "No torque, tracking or independent grasp FK audit."])
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
