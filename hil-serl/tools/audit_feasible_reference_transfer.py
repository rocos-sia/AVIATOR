"""Require an all-constraints high reference, then audit archived low paths.

Transfers the unchanged global field, not the reference joint time series.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.audit_random_steering import init_worker, audit_path
from tools.audit_static_cross_path import summarize


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--archive", type=Path, required=True)
    ap.add_argument("--high-audit", type=Path, required=True)
    ap.add_argument("--reference", default="traj_0047")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    args = ap.parse_args()
    if args.output.exists():
        ap.error("choose a new output directory")
    prior = json.loads((args.archive / "episodes.json").read_text())
    archive_summary = json.loads((args.archive / "summary.json").read_text())
    high_summary = json.loads((args.high_audit / "summary.json").read_text())
    high_rows = json.loads((args.high_audit / "episodes.json").read_text())
    field = Path(high_summary["field"])
    assert digest(field) == high_summary["field_sha256"] == archive_summary["field_sha256"]
    reference = args.archive / "trajs" / (args.reference + ".npz")
    assert digest(reference) == high_rows[args.reference]["source_sha256"]
    selected = {k: v for k, v in prior.items() if v["speed_cap"] == .6}
    assert len(selected) == 20
    for name, row in selected.items():
        assert digest(args.archive / "trajs" / (name + ".npz")) == row["sha256"]
    init_worker(args.manifold_dir, args.model, str(field))
    high = audit_path(str(reference))
    if not high["pass_with_acceleration"]:
        raise ValueError(f"high reference fails required constraints: {high}")
    print(f"high reference independently passes: {high}", flush=True)
    args.output.mkdir(parents=True)
    rows = {}
    with ProcessPoolExecutor(max_workers=2, initializer=init_worker,
                             initargs=(args.manifold_dir, args.model, str(field))) as pool:
        futures = {pool.submit(audit_path, str(args.archive / "trajs" / (name + ".npz"))): name
                   for name in selected}
        for future in as_completed(futures):
            name = futures[future]
            result = future.result()
            # Verify that linking the corrected reference has not silently
            # altered the field, paths, model outputs or pass/fail criteria.
            for key in ("max_qdot", "max_qddot", "min_wall_clearance_m"):
                np.testing.assert_allclose(result[key], selected[name]["metrics"][key], atol=1e-10, rtol=1e-10)
            for key in ("pass_velocity_geometry", "pass_with_acceleration",
                        "contact_samples", "wall_failure_samples", "joint_limit_samples"):
                assert result[key] == selected[name]["metrics"][key]
            rows[name] = dict(path=str((args.archive / "trajs" / (name + ".npz")).resolve()),
                              sha256=selected[name]["sha256"], mode=selected[name]["mode"], metrics=result)
            (args.output / "episodes.json").write_text(json.dumps(dict(sorted(rows.items())), indent=2) + "\n")
            print(f"low paths verified {len(rows)}/20", flush=True)
    report = dict(field=str(field), field_sha256=digest(field),
                  model=str(Path(args.model).resolve()), model_xml_sha256=digest(args.model),
                  reference=dict(path=str(reference.resolve()), sha256=digest(reference),
                                 theta_peak=prior[args.reference]["theta_peak"], metrics=high),
                  summary=summarize(rows.values()),
                  by_mode={mode:summarize([r for r in rows.values() if r["mode"]==mode])
                           for mode in ("steering_only", "steering_with_slide")},
                  matched_previous_audit=True,
                  caveats=["High path is feasible under the sampled audit; this does not certify the global field.",
                           "Different paths use the same global field, not the same q(t).",
                           "Each starts from its own F(x0); no transition from the high reference endpoint.",
                           "Point/midpoint collision checks and finite-difference kinematics; no torque or tracking proof."])
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
