"""Freeze genuinely different wheel/base installations and disjoint trajectories.

Initialization failures are retained, never resampled away. A failed setup gate
requires revisiting initial grasp acquisition before training on this protocol.
"""
from __future__ import annotations
import argparse
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco

from tools.gfr1_remount_evaluate import ROOT, dump, sha
from tools.gfr1_remount_pilot import initialize
from tools.gfr1_shared_remount_env import SharedRemountEnv, arm_local_basis
from tools.validate_online_lift import analytic_constraint_matrices


def write_installation(base, destination, parameters):
    tree = ET.parse(base)
    bodies = {body.get("name"): body for body in tree.iter("body")}
    # Four independently varied physical installation parameters. Wheel pose
    # changes the closed chain, unlike the legacy fixture-only wall shift.
    for name, dx, yaw in [("steering_wheel", parameters[0], parameters[1]),
        ("AR5-5_07L-W4C4A2_base", .5*parameters[2], .5*parameters[3]),
        ("AR5-5_07R-W4C4A2_base", -.5*parameters[2], -.5*parameters[3])]:
        body = bodies[name]
        p = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
        q = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
        p[0] += dx
        r = Rotation.from_rotvec([0., 0., np.deg2rad(yaw)]) * Rotation.from_quat(q[[1,2,3,0]])
        q = r.as_quat()[[3,0,1,2]]
        body.set("pos", " ".join(format(v, ".16g") for v in p))
        body.set("quat", " ".join(format(v, ".16g") for v in q))
    tree.find("compiler").set("meshdir", str(base.parent/"aviator_meshes"))
    tree.write(destination, encoding="unicode")


def geometry_signature(model):
    h = hashlib.sha256()
    for field in ("body_pos", "body_quat", "geom_pos", "geom_quat", "geom_size",
                  "site_pos", "site_quat", "jnt_pos", "jnt_axis", "jnt_range",
                  "mesh_vert", "mesh_face"):
        h.update(field.encode())
        h.update(np.asarray(getattr(model, field)).tobytes())
    return h.hexdigest()


def trajectory_signature(path):
    h = hashlib.sha256()
    with np.load(path) as data:
        for name in ("t", "x", "xdot"):
            if name in data:
                h.update(name.encode())
                h.update(np.asarray(data[name], dtype=np.float64).tobytes())
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    base = ROOT/"reference/rocos-mujoco/model/aviator.xml"
    data = ROOT/"hil-serl/data/aviator"
    rng = np.random.default_rng(20261125)
    train = [np.zeros(4)]
    for axis, scale in enumerate([.001, .25, .001, .25]):
        for sign in (-1., 1.):
            row = np.zeros(4)
            row[axis] = sign*scale
            train.append(row)
    # Combinations are included in training, with separate random combinations
    # in development and heldout. All split draws are frozen before compiling.
    train += list(rng.uniform(-1, 1, (3, 4))*[.001, .25, .001, .25])
    development = rng.uniform(-1, 1, (4, 4))*[.001, .25, .001, .25]
    heldout = rng.uniform(-1, 1, (12, 4))*[.001, .25, .001, .25]
    specification = {"train": train, "development": development, "heldout": heldout}
    paths = {"train": sorted((data/"trajectory_source/trajs/rl_train").glob("*.npz"))[:6],
             "development": sorted((data/"trajectory_source/trajs/val").glob("*.npz"))[:3],
             "heldout": sorted((data/"trajectory_source/trajs/test").glob("*.npz"))[:6]}
    expected = {"train": 6, "development": 3, "heldout": 6}
    if any(len(paths[k]) != expected[k] for k in paths):
        raise ValueError("Missing frozen trajectory allocation")
    dump(out/"allocation.json", dict(seed=20261125, parameters=specification,
        parameter_units=["wheel_dx_m", "wheel_yaw_deg", "base_differential_dx_m", "base_differential_yaw_deg"],
        trajectories={k: [str(p) for p in v] for k,v in paths.items()}, initial_grasp="LUT midpoint followed by bounded closure Newton repair; no resampling",
        train_demo_buffer=None, checkpoint_initialization="fresh random SAC; historical 60k never loaded"))
    env = SharedRemountEnv(str(data/"manifold_phi_stale_pitch-10deg"), str(base), str(data/"trajectory_source"))
    signatures, failures, results = {}, [], {}
    for split, parameters in specification.items():
        records, geometric = [], []
        for index, theta in enumerate(parameters):
            model = out/f"{split}_{index:02d}.xml"
            write_installation(base, model, theta)
            local = initialize(env, paths[split], [(f"{split}_{index:02d}", model, 0.)])
            signature = geometry_signature(env.fk.model)
            geometric.append(signature)
            for record in local:
                record.update(installation_parameters=np.asarray(theta), geometry_signature=signature,
                    trajectory_signature=trajectory_signature(record["trajectory_path"]),
                    true_model=str(model), estimated_model=str(model))
                if record["valid"]:
                    _, aq, _ = analytic_constraint_matrices(env.fk, record["initial_x"], record["initial_q"])
                    try:
                        n, component = arm_local_basis(aq)
                        record.update(action_chart_valid=True, action_basis=n, chart_component_min=component,
                            tangent_residual_max=float(np.max(np.abs(aq@n))),
                            basis_orthogonality_error=float(np.max(np.abs(n.T@n-np.eye(2)))))
                    except ValueError as error:
                        record["action_chart_valid"] = False
                        failures.append(dict(case=record["case"], reason="action_chart_invalid", detail=str(error)))
                else:
                    failures.append(dict(case=record["case"], reason="initialization_invalid",
                        clearance_m=record["initial_clearance_m"], residual=record["initial_closure_residual_inf"]))
            records.extend(local)
            print(f"{split} installation {index}: {sum(r['valid'] for r in local)}/{len(local)} valid starts", flush=True)
        payload = dict(records=records, installation_signatures=geometric,
            trajectory_signatures=[trajectory_signature(path) for path in paths[split]],
            split=split, models=[str(out/f"{split}_{i:02d}.xml") for i in range(len(parameters))])
        dump(out/f"{split}.json", payload)
        signatures[split] = payload
        results[split] = dict(installations=len(parameters), cases=len(records), valid=sum(r["valid"] for r in records))
    for left, right in [("train", "development"), ("train", "heldout"), ("development", "heldout")]:
        for key in ("installation_signatures", "trajectory_signatures"):
            if set(signatures[left][key]) & set(signatures[right][key]):
                failures.append(dict(reason="split_leakage", pair=[left, right], field=key))
    dump(out/"gate.json", dict(passed=not failures, failures=failures, splits=results,
        source_sha256={str(Path(__file__)): sha(__file__),
                       str(ROOT/"hil-serl/tools/gfr1_shared_remount_env.py"): sha(ROOT/"hil-serl/tools/gfr1_shared_remount_env.py")}))
    env.close()


if __name__ == "__main__":
    main()
