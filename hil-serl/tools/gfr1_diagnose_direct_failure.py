"""Offline, same-state counterfactuals at standalone-policy first failures.

No state is committed. Grid/stochastic acceptance is local feasibility, not
full-trajectory success. The frozen checkpoint is never updated.
"""
import argparse
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import torch
from stable_baselines3 import SAC
from tools.gfr1_direct_only_env import DirectOnlyEnv
from tools.gfr1_remount_evaluate import ROOT, dump, sha, restore


def forbidden(*args,**kwargs):
    raise AssertionError("No corrective executor is allowed in diagnosis")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run",type=Path,required=True)
    p.add_argument("--dataset",type=Path,required=True)
    p.add_argument("--checkpoint",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    a=p.parse_args()
    torch.set_num_threads(1);torch.manual_seed(20261126)
    model=SAC.load(a.checkpoint,device="cpu")
    original_hash=sha(a.checkpoint)
    manifests={s:json.loads((a.dataset/(s+".json")).read_text())["records"] for s in ["train","development","heldout"]}
    selected={}
    for split,records in manifests.items():
        for record in records:
            if split=="development" or record['case'].endswith('/traj_0000'):
                selected[record['case']]=(split,record)
    failed={}
    with (a.run/"steps.jsonl").open() as stream:
        for line in stream:
            row=json.loads(line)
            if row.get("direct_accepted") is False and row["case"] in selected:
                failed[row["case"]]=row
    data=ROOT/"hil-serl/data/aviator"
    env=DirectOnlyEnv(str(data/"manifold_phi_stale_pitch-10deg"),next(iter(selected.values()))[1]['model'],str(data/"trajectory_source"))
    results=[]
    with patch("experiments.aviator_manifold.gfr_env.project_r",forbidden),patch("tools.gfr1_repaired_env.select_action",forbidden):
        for case,(split,record) in selected.items():
            row=failed[case]
            env._model_records=[record];env.reset(seed=20261125,options={"trajectory_index":0})
            state=row["before"]
            for name in ["_x","_q","_previous_n","_r_prev","_repair_previous_raw"]:
                if state[name] is not None:state[name]=np.asarray(state[name],dtype=float)
            restore(env,state)
            obs=env._get_obs()
            deterministic=model.predict(obs,deterministic=True)[0]
            tensor,_=model.policy.obs_to_tensor(obs)
            with torch.no_grad():
                mu,log_std,_=model.actor.get_action_dist_params(tensor)
                stochastic=model.actor(tensor.repeat(32,1),deterministic=False).cpu().numpy()
            previous=np.clip(env._r_prev/env.r_max,-1,1)
            named=[("policy_mean",deterministic),("zero",np.zeros(2)),("previous_accepted",previous)]
            named += [("grid",np.array([x,y])) for x in np.linspace(-1,1,5) for y in np.linspace(-1,1,5)]
            named += [("stochastic",action) for action in stochastic]
            trials=[]
            for name,action in named:
                candidate,_=env._candidate(env._x,env._q,env._path_state(env._u)[1],
                    action*env.r_max,1.,env._previous_n,structured=False)
                trials.append(dict(kind=name,action=action,feasible=candidate["feasible"],
                    clearance_m=candidate.get("d_actual_m"),joint_margin_rad=candidate.get("joint_limit_margin_rad")))
            np.testing.assert_array_equal(env._q,state["_q"])
            np.testing.assert_array_equal(env._x,state["_x"])
            assert env._u==state["_u"]
            raw=np.asarray(row["info"]["policy_action_raw"])
            error=float(np.max(abs(raw-deterministic)))
            if error>1e-6:raise AssertionError(f"Policy replay mismatch {case}: {error}")
            result=dict(case=case,split=split,failed_step=row["step"],policy_replay_max_error=error,
                gaussian_mu=mu.cpu().numpy()[0],gaussian_std=log_std.exp().cpu().numpy()[0],trials=trials)
            results.append(result)
            print(json.dumps(dict(case=case,grid_valid=sum(t["feasible"] for t in trials if t["kind"]=="grid"),
                stochastic_valid=sum(t["feasible"] for t in trials if t["kind"]=="stochastic"))),flush=True)
    summary=dict(cases=len(results),deterministic_rejection_reproduced=sum(not r["trials"][0]["feasible"] for r in results),
        zero_pass=sum(r["trials"][1]["feasible"] for r in results),
        previous_pass=sum(r["trials"][2]["feasible"] for r in results),
        cases_with_feasible_grid_action=sum(any(t["feasible"] for t in r["trials"] if t["kind"]=="grid") for r in results),
        stochastic_passes=sum(t["feasible"] for r in results for t in r["trials"] if t["kind"]=="stochastic"),
        stochastic_attempts=32*len(results),checkpoint_unchanged=sha(a.checkpoint)==original_hash,
        scope="first-failure snapshots; beta=1; all development cases, first trajectory per training/heldout installation; local counterfactuals only")
    dump(a.output,dict(summary=summary,cases=results))
    env.close()


if __name__=="__main__":main()
