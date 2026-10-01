"""Audit a frozen high-speed-selected table on different low-speed val paths."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_collision import CollisionChecker
from tools.audit_static_field import interpolate_q
from tools.audit_static_slowdown import geometry, metrics, slowdown
from tools.rescale_task_trajectories import continuous_theta_peak


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(rows):
    values = [r["metrics"] for r in rows]
    return dict(
        n_paths=len(values),
        pass_velocity_geometry=sum(v["pass_velocity_geometry"] for v in values),
        pass_with_acceleration=sum(v["pass_with_acceleration"] for v in values),
        speed_failures=sum(v["max_qdot"] > 1.5 for v in values),
        acceleration_failures=sum(v["max_qddot"] > 10 for v in values),
        contact_episodes=sum(v["contact_samples"] > 0 for v in values),
        wall_failure_episodes=sum(v["wall_failure_samples"] > 0 for v in values),
        joint_limit_episodes=sum(v["joint_limit_samples"] > 0 for v in values),
        max_qdot=max(v["max_qdot"] for v in values),
        max_qddot=max(v["max_qddot"] for v in values),
        min_wall_clearance_m=min(v["min_wall_clearance_m"] for v in values))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--high-audit", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--theta-speed", type=float, default=0.6)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    args = ap.parse_args()
    if args.output.exists() or args.theta_speed <= 0:
        ap.error("need a new output directory and positive speed")
    previous = json.loads((args.high_audit / "summary.json").read_text())
    prior_episodes = json.loads((args.high_audit / "episodes.json").read_text())
    # Choose the lexicographically first passing high path before reading the
    # target paths. This chooses a reference, never another field or posture.
    reference_id = next(k for k, v in sorted(prior_episodes.items())
                        if v["high"]["pass_velocity_geometry"])
    reference_path = Path(previous["source"]) / (reference_id + ".npz")
    assert digest(reference_path) == prior_episodes[reference_id]["source_sha256"]
    field_path = Path(previous["field"])
    assert digest(field_path) == previous["field_sha256"]
    manifest = json.loads((args.source / "manifest.json").read_text())
    sources = [r for r in manifest["trajectories"] if r["split"] == "val"]
    assert len(sources) == 120
    assert Counter(r["reversals"] for r in sources) == {2: 40, 4: 40, 6: 40}
    lookup = ManifoldLookup(args.manifold_dir)
    checker = CollisionChecker(args.model)
    with np.load(field_path) as f:
        assert np.array_equal(f["theta"], lookup.theta_axis)
        assert np.array_equal(f["s"], lookup.s_axis)
        q_grid = f["q"]
    with np.load(reference_path) as f:
        reference_x = f["x"].copy()
    reference_q = interpolate_q(lookup, q_grid, reference_x)
    reference_metrics = metrics(reference_q, .01, geometry(checker, lookup, reference_x, reference_q))
    assert reference_metrics["pass_velocity_geometry"]
    args.output.mkdir(parents=True)
    task_dest = args.output / "trajs" / "val"
    task_dest.mkdir(parents=True)
    rows = {}
    for i, source in enumerate(sources):
        path = args.source / source["path"]
        assert digest(path) == source["sha256"]
        assert source["sha256"] != prior_episodes[reference_id]["source_sha256"]
        with np.load(path) as f:
            data = {k: f[k] for k in f.files}
        # A different start establishes a different ordered task path, even
        # after timing is removed; overlapping workspace is allowed.
        start_delta = float(np.max(np.abs(data["x"][0] - reference_x[0])))
        assert start_delta > 1e-8
        peak = continuous_theta_peak(data)
        requested_rate = min(1., args.theta_speed / peak)
        if requested_rate == 1.:
            slow, rate = data, 1.
        else:
            slow, rate = slowdown(data, requested_rate)
        assert continuous_theta_peak(slow) <= args.theta_speed + 1e-8
        q = interpolate_q(lookup, q_grid, slow["x"])
        assert np.isfinite(q).all()
        result = metrics(q, .01, geometry(checker, lookup, slow["x"], q))
        dest = task_dest / path.name
        np.savez_compressed(dest, **slow)
        rows[path.stem] = dict(source=source, rate=rate, n_samples=len(q),
                               theta_peak=continuous_theta_peak(slow),
                               start_difference_from_high_reference=start_delta,
                               low_sha256=digest(dest), metrics=result)
        (args.output / "episodes.json").write_text(json.dumps(rows, indent=2) + "\n")
        if i % 10 == 0 or i + 1 == len(sources):
            print(f"cross-path audit {i + 1}/{len(sources)}", flush=True)
    report = dict(
        field=str(field_path), field_sha256=digest(field_path),
        source_manifest=str((args.source / "manifest.json").resolve()),
        source_manifest_sha256=digest(args.source / "manifest.json"),
        reference_high=dict(path=str(reference_path), sha256=digest(reference_path), metrics=reference_metrics),
        low_theta_speed_cap=args.theta_speed,
        summary=summarize(rows.values()),
        by_reversals={str(n): summarize([r for r in rows.values() if r["source"]["reversals"] == n])
                      for n in (2, 4, 6)},
        limits=dict(qdot=1.5, qddot=10, wall_clearance=.005, contact_penetration=.001),
        caveats=["Frozen global table transfer, not replaying the reference joint time series.",
                 "Different paths run separately, starting at their own F(x0); no inter-path joining test.",
                 "Existing val is diagnostic; the new test split was not accessed.",
                 "Reference passes velocity/geometry but not acceleration constraints.",
                 "Point/midpoint collision audit; no continuous proof, torque, tracking or grasp FK audit.",
                 "Initial rest-to-motion acceleration boundary excluded."])
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
