"""Small, frozen feasibility allocation, not an empirically calibrated envelope."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

from tools.gfr1_remount_evaluate import ROOT, dump, sha
from tools.gfr1_remount_pilot import initialize
from tools.gfr1_prepare_shared_remount import geometry_signature, trajectory_signature
from tools.gfr2_shared_env import SharedProposalEnv, BASES, HANDLES


def perturb_model(source, destination, parameters):
    tree = ET.parse(source)
    nodes = {e.get("name"): e for e in tree.iter() if e.get("name")}
    for i, name in enumerate(("steering_wheel", *BASES)):
        node = nodes[name]
        pose = parameters[i*6:(i+1)*6]
        position = np.fromstring(node.get("pos", "0 0 0"), sep=" ")+pose[:3]
        q = np.fromstring(node.get("quat", "1 0 0 0"), sep=" ")
        r = Rotation.from_rotvec(pose[3:])*Rotation.from_quat(q[[1, 2, 3, 0]])
        node.set("pos", " ".join(map(str, position)))
        node.set("quat", " ".join(map(str, r.as_quat()[[3, 0, 1, 2]])))
    # Small angular regrasp about wheel axis; no radial translation off the rim.
    # This is a kinematic grasp-target perturbation, not grasp/contact validation.
    for i, name in enumerate(HANDLES):
        node = nodes[name]
        arc, *orientation = parameters[18+i*4:22+i*4]
        turn = Rotation.from_rotvec([0., 0., arc])
        position = turn.apply(np.fromstring(node.get("pos"), sep=" "))
        q = np.fromstring(node.get("quat"), sep=" ")
        r = turn*Rotation.from_quat(q[[1, 2, 3, 0]])*Rotation.from_rotvec(orientation)
        node.set("pos", " ".join(map(str, position)))
        node.set("quat", " ".join(map(str, r.as_quat()[[3, 0, 1, 2]])))
    tree.find("compiler").set("meshdir", str(ROOT/"reference/rocos-mujoco/model/aviator_meshes"))
    tree.write(destination, encoding="unicode")


def prepare(out):
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(20261126)
    scales = np.r_[np.tile([.0005]*3+[np.deg2rad(.1)]*3, 3),
                   np.tile([np.deg2rad(.25)]+[np.deg2rad(.1)]*3, 2)]
    allocation = {"train": rng.uniform(-1, 1, (8, 26))*scales,
                  "development": rng.uniform(-1, 1, (4, 26))*scales,
                  "heldout": rng.uniform(-1, 1, (4, 26))*scales}
    allocation["train"][0] = 0.
    dump(out/"allocation.json", dict(seed=20261126, parameters=allocation,
        scope="local kinematic feasibility pilot; not measured installation/calibration bounds",
        installation_translation_per_axis_m=.0005, installation_rotvec_per_axis_deg=.1,
        grasp_arc_deg=.25, grasp_local_rotvec_per_axis_deg=.1,
        residual_stage="not enabled until exact-calibration BC qualifies",
        reset_failures="retained without resampling", calibration="true=estimated in this prerequisite"))
    data = ROOT/"hil-serl/data/aviator"
    base = ROOT/"reference/rocos-mujoco/model/aviator.xml"
    env = SharedProposalEnv(str(data/"manifold_phi_stale_pitch-10deg"), str(base), str(data/"trajectory_source"))
    signatures, failed, sizes = {}, [], {}
    try:
        for split, values in allocation.items():
            directory = {"train": "rl_train", "development": "val", "heldout": "test"}[split]
            paths = sorted((data/"trajectory_source/trajs"/directory).glob("*.npz"))[:2]
            assert len(paths) == 2
            records, models = [], []
            for i, theta in enumerate(values):
                model = out/f"{split}_{i:02d}.xml"
                perturb_model(base, model, theta)
                local = initialize(env, paths, [(f"{split}_{i:02d}", model, 0.)])
                signature = geometry_signature(env.fk.model)
                models.append(str(model))
                for record in local:
                    record.update(true_model=str(model), estimated_model=str(model),
                                  geometry_signature=signature, trajectory_signature=trajectory_signature(record["trajectory_path"]))
                    if record["valid"]:
                        env._model_records = [record]
                        obs, _ = env.reset(options={"trajectory_index": 0})
                        assert obs.shape == (128,)
                    else:
                        failed.append(record)
                records.extend(local)
                print(json.dumps(dict(split=split, installation=i, valid=sum(r["valid"] for r in local), cases=len(local))), flush=True)
            payload = dict(records=records, models=models)
            dump(out/f"{split}.json", payload)
            signatures[split] = {key: {r[key] for r in records} for key in ("geometry_signature", "trajectory_signature")}
            sizes[split] = dict(cases=len(records), valid=sum(r["valid"] for r in records))
        for a, b in (("train", "development"), ("train", "heldout"), ("development", "heldout")):
            for key in signatures[a]:
                if signatures[a][key] & signatures[b][key]:
                    raise ValueError("Split leakage: "+key)
        result = dict(passed=not failed, failures=failed, splits=sizes, source_sha256=sha(__file__))
        dump(out/"gate.json", result)
        return result
    finally:
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    prepare(parser.parse_args().output)
