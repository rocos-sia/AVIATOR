"""Replace three infeasible online tasks with deterministic, audited samples.

The original trajectory generator enforces task-space bounds but has no
physical feasibility test.  Only the three failed rl_train entries are changed;
the DP, validation and test splits retain their original seeds and bytes.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from examples.experiments.aviator_manifold.trajectory_generator import (
    A_MAX, V_MAX, generate_task_trajectory,
)
from tools.audit_prior_states import physical_evaluate


REPLACEMENTS = {64: 653, 136: 661, 360: 666}
ROOT = Path("data/aviator/trajectory_source")
BACKUP = ROOT / "backup_invalid_rl_starts_2026-09-23"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest_path = ROOT / "split_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    rows = {(r["split"], r["index"]): r for r in manifest["trajectories"]}
    existing_seeds = {r["seed"] for r in manifest["trajectories"]}
    lookup = ManifoldLookup("data/aviator/manifold_phi")
    generated = {}
    geometry_input = []
    for index, seed in REPLACEMENTS.items():
        row = rows[("rl_train", index)]
        assert row["seed"] != seed and seed not in existing_seeds
        path = ROOT / row["path"]
        assert sha256(path) == row["sha256"], path
        rng = np.random.default_rng(seed)
        T = float(rng.uniform(manifest["T_min"], manifest["T_max"]))
        traj = generate_task_trajectory(rng, T, dt=manifest["dt"])
        x = traj["x"][0]
        box = lookup.safe_interval(x[None])
        phi = np.clip(np.zeros(2), box["phi_safe_lo"][0], box["phi_safe_hi"][0])
        result = lookup.query(x[None], phi[None], check_safe=False)
        assert int(result["branch"][0]) < 2
        geometry_input.append(np.r_[x, result["qL"][0], result["qR"][0]])
        assert np.all(np.max(np.abs(traj["xdot"]), axis=0) <= V_MAX)
        assert np.all(np.max(np.abs(traj["xddot"]), axis=0) <= A_MAX)
        generated[index] = (seed, T, traj)

    physical = physical_evaluate(
        np.asarray(geometry_input),
        "../AviatorRobot/build/bin/aviator_clearance_trajectory",
        "../AviatorRobot/config/aviator.yaml",
        "data/aviator/online_start_audit",
    )
    for index, result in zip(REPLACEMENTS, physical.itertuples()):
        if (result.d_min < 0.005 or result.task_error > 0.002
                or result.joint_margin < 0 or result.collision != 0):
            raise RuntimeError(f"replacement for trajectory {index} is infeasible: {result}")

    if BACKUP.exists():
        raise FileExistsError(f"backup exists; refusing a second replacement: {BACKUP}")
    BACKUP.mkdir()
    shutil.copy2(manifest_path, BACKUP / manifest_path.name)
    provenance = []
    for index, (seed, T, traj) in generated.items():
        row = rows[("rl_train", index)]
        path = ROOT / row["path"]
        shutil.copy2(path, BACKUP / path.name)
        previous_seed, previous_hash = row["seed"], row["sha256"]
        np.savez(path, **{name: traj[name] for name in ("t", "x", "xdot", "xddot")})
        row.update(seed=seed, T=T, n_samples=len(traj["t"]), sha256=sha256(path),
                   max_abs_xdot=np.max(np.abs(traj["xdot"]), axis=0).tolist(),
                   max_abs_xddot=np.max(np.abs(traj["xddot"]), axis=0).tolist())
        provenance.append({"index": index, "old_seed": previous_seed,
                           "old_sha256": previous_hash, "new_seed": seed,
                           "new_sha256": row["sha256"]})
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    (BACKUP / "replacements.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(provenance, indent=2))


if __name__ == "__main__":
    main()
