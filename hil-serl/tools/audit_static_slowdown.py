"""Freeze a high-speed-selected joint table and audit paired slower executions.

Runs on CPU in the mujo environment. No policy/checkpoint selection or test
split is involved. Collision semantics match audit_static_collision.py.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.audit_static_field import interpolate_q
from tools.rescale_task_trajectories import continuous_theta_peak


def slowdown(data, requested_rate, dt=0.01):
    """Preserve both endpoints with a slightly conservative constant rate."""
    t = data["t"]
    duration = float(t[-1] - t[0])
    intervals = int(np.ceil(duration / (requested_rate * dt)))
    rate = duration / (intervals * dt)
    new_t = np.arange(intervals + 1) * dt
    source_t = np.linspace(t[0], t[-1], intervals + 1)
    curves = [BPoly.from_derivatives(t, list(zip(data["x"][:, j],
                                                data["xdot"][:, j],
                                                data["xddot"][:, j])))
              for j in range(2)]
    result = {"t": new_t}
    for order, key in enumerate(("x", "xdot", "xddot", "xdddot")):
        result[key] = rate**order * np.stack(
            [curve.derivative(order)(source_t) for curve in curves], axis=1)
    np.testing.assert_allclose(result["x"][[0, -1]], data["x"][[0, -1]], atol=1e-12)
    assert 0 < rate <= requested_rate + 1e-12
    return result, rate


def geometry(checker, lookup, x, q):
    minimum = float("inf")
    contacts = wall_failures = 0
    for xx, qq in ((x, q), ((x[1:] + x[:-1]) / 2, (q[1:] + q[:-1]) / 2)):
        for point, joints in zip(xx, qq):
            clearance, contact = checker.check(point, joints)
            if not np.isfinite(clearance):
                raise ValueError("nonfinite geometry result")
            minimum = min(minimum, clearance)
            contacts += int(contact)
            wall_failures += int(clearance < 0.005)
    lower, upper = lookup.joint_lower.reshape(14), lookup.joint_upper.reshape(14)
    joint_failures = int(np.sum(np.any((q < lower) | (q > upper), axis=1)))
    return dict(min_wall_clearance_m=minimum, contact_samples=contacts,
                wall_failure_samples=wall_failures, joint_limit_samples=joint_failures,
                geometry_pass=not (contacts or wall_failures or joint_failures))


def metrics(q, dt, geom):
    vmax = float(np.max(np.abs(np.diff(q, axis=0) / dt)))
    amax = float(np.max(np.abs(np.diff(q, n=2, axis=0) / dt**2)))
    return dict(**geom, max_qdot=vmax, max_qddot=amax,
                pass_velocity_geometry=bool(geom["geometry_pass"] and vmax <= 1.5),
                pass_with_acceleration=bool(geom["geometry_pass"] and vmax <= 1.5 and amax <= 10))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--field", type=Path, required=True)
    ap.add_argument("--source", type=Path, required=True, help="directory of high-speed val NPZs")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--rate", type=float, default=0.4)
    ap.add_argument("--target-theta-speed", type=float,
                    help="derive slowdown rate from the source continuous peak")
    ap.add_argument("--trajectory-name", help="audit only this NPZ stem")
    ap.add_argument("--require-high-feasible", action="store_true",
                    help="abort unless the high source passes all audited constraints")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    args = ap.parse_args()
    if not 0 < args.rate < 1:
        ap.error("rate must be strictly between zero and one")
    paths = sorted(args.source.glob("*.npz"))
    if args.trajectory_name:
        paths = [p for p in paths if p.stem == args.trajectory_name]
    if args.limit:
        paths = paths[:args.limit]
    if not paths:
        ap.error("no source paths")
    if args.output.exists():
        ap.error("output already exists; choose a new directory")
    args.output.mkdir(parents=True)
    dest = args.output / "slow_tasks"
    dest.mkdir()
    lookup = ManifoldLookup(args.manifold_dir)
    checker = CollisionChecker(args.model)
    with np.load(args.field) as f:
        assert np.array_equal(f["theta"], lookup.theta_axis)
        assert np.array_equal(f["s"], lookup.s_axis)
        q_grid = f["q"]
    rows = {}
    for i, path in enumerate(paths):
        with np.load(path) as f:
            high = {key: f[key] for key in f.files}
        np.testing.assert_allclose(np.diff(high["t"]), 0.01, atol=1e-10)
        peak = continuous_theta_peak(high)
        requested_rate = args.rate
        if args.target_theta_speed is not None:
            if not 0 < args.target_theta_speed < peak:
                raise ValueError("target speed must be positive and below source peak")
            requested_rate = args.target_theta_speed / peak
        slow, rate = slowdown(high, requested_rate)
        high_q = interpolate_q(lookup, q_grid, high["x"])
        slow_q = interpolate_q(lookup, q_grid, slow["x"])
        if not np.isfinite(high_q).all() or not np.isfinite(slow_q).all():
            raise ValueError("nonfinite interpolated joints")
        high_geom = geometry(checker, lookup, high["x"], high_q)
        high_metrics = metrics(high_q, 0.01, high_geom)
        if args.require_high_feasible and not high_metrics["pass_with_acceleration"]:
            raise ValueError(f"source {path.stem} is not feasible under all audited constraints: {high_metrics}")
        print(f"high source {path.stem}: {high_metrics}", flush=True)
        slow_geom = geometry(checker, lookup, slow["x"], slow_q)
        # Exact time stretching keeps the same q samples and geometry. This
        # separates the scaling identity from denser 100 Hz table sampling.
        stretched = metrics(high_q, 0.01 / rate, high_geom)
        np.testing.assert_allclose(stretched["max_qdot"], rate * high_metrics["max_qdot"], rtol=1e-12)
        np.testing.assert_allclose(stretched["max_qddot"], rate**2 * high_metrics["max_qddot"], rtol=1e-12)
        np.savez_compressed(dest / path.name, **slow)
        rows[path.stem] = dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                               rate=rate, high_samples=len(high_q), slow_samples=len(slow_q),
                               high_theta_peak=continuous_theta_peak(high),
                               slow_theta_peak=continuous_theta_peak(slow),
                               high=high_metrics, exact_time_stretch=stretched,
                               slow_100hz=metrics(slow_q, 0.01, slow_geom))
        (args.output / "episodes.json").write_text(json.dumps(rows, indent=2) + "\n")
        if i % 5 == 0 or i + 1 == len(paths):
            print(f"audited {i + 1}/{len(paths)}", flush=True)
    summary = {}
    for method in ("high", "exact_time_stretch", "slow_100hz"):
        values = [r[method] for r in rows.values()]
        summary[method] = {
            key: sum(v[key] for v in values)
            for key in ("pass_velocity_geometry", "pass_with_acceleration")}
        summary[method].update(
            speed_failures=sum(v["max_qdot"] > 1.5 for v in values),
            acceleration_failures=sum(v["max_qddot"] > 10 for v in values),
            contact_episodes=sum(v["contact_samples"] > 0 for v in values),
            wall_failure_episodes=sum(v["wall_failure_samples"] > 0 for v in values),
            joint_limit_episodes=sum(v["joint_limit_samples"] > 0 for v in values),
            max_qdot=max(v["max_qdot"] for v in values),
            max_qddot=max(v["max_qddot"] for v in values),
            min_wall_clearance_m=min(v["min_wall_clearance_m"] for v in values))
    conditional = {}
    for key in ("pass_velocity_geometry", "pass_with_acceleration"):
        eligible = [r for r in rows.values() if r["high"][key]]
        conditional[key] = dict(high_pass_count=len(eligible),
                                slow_100hz_pass_count=sum(r["slow_100hz"][key] for r in eligible),
                                exact_stretch_pass_count=sum(r["exact_time_stretch"][key] for r in eligible))
    report = dict(field=str(args.field.resolve()), field_sha256=hashlib.sha256(args.field.read_bytes()).hexdigest(),
                  source=str(args.source.resolve()), n_paths=len(rows),
                  requested_rate=args.rate if args.target_theta_speed is None else None,
                  target_theta_speed=args.target_theta_speed,
                  required_high_feasible=args.require_high_feasible,
                  limits=dict(qdot=1.5, qddot=10, wall_clearance=0.005, contact_penetration=0.001),
                  summary=summary, conditional=conditional,
                  caveats=["No torque, tracking or independent grasp FK audit.",
                           "Sample and joint-segment midpoint collision checks; not a continuous proof.",
                           "Acceleration uses second differences, excluding rest-start boundary.",
                           "Selected source paths are diagnostic, not a held-out generalization test."])
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
