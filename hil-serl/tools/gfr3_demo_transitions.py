"""Replay frozen successful demonstrations into exact SAC transitions/reset states."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing as mp
from pathlib import Path
import numpy as np
from tools.gfr1_remount_evaluate import dump, sha, audit_motion
from tools.audit_static_collision import CollisionChecker
from tools.gfr2_teacher import make_env


def pack_state(env):
    n=env._previous_n
    return np.r_[env._u,env._x,env._q,np.zeros(28) if n is None else n.ravel(),
                 env._r_prev,env._beta_prev,env._last_intervention,env._last_rejection,
                 env._last_recovery,env._last_sigma,float(n is not None)]


def unpack_state(env, state):
    s=np.asarray(state,dtype=float)
    if s.shape!=(53,) or not np.isfinite(s).all():
        raise ValueError("Invalid curriculum state")
    env._u=float(s[0]);env._x=s[1:3].copy();env._q=s[3:17].copy()
    env._previous_n=s[17:45].reshape(14,2).copy() if s[52] else None
    env._r_prev=s[45:47].copy();env._beta_prev=float(s[47])
    env._last_intervention,env._last_rejection,env._last_recovery,env._last_sigma=map(float,s[48:52])
    env._repair_cycle=int(round(env._u*env.traj_dt/env.dt))
    env._repair_previous_raw=(env._r_prev/env.r_max).astype(np.float32)
    return env._get_obs()


def critical_frames(observations,actions):
    change=np.linalg.norm(actions-observations[:,33:35],axis=1)>.1
    boundary=(observations[:,30]<.55)|(observations[:,31]<.05)
    reversal=np.zeros(len(actions),dtype=bool)
    reversal[1:]=np.any(observations[1:,2:4]*observations[:-1,2:4]<0,axis=1)
    important=change|boundary|reversal
    # Include the approach to an adjustment, not only the final boundary frame.
    for offset in range(1,21):
        important[:-offset]|=change[offset:]|reversal[offset:]
    return important


def replay_case(record,source,out):
    env=make_env(record)
    audit=CollisionChecker(record["true_model"])
    with np.load(source) as old:
        expected=old["observations"].copy();actions=old["actions"].copy()
    obs=[];nxt=[];rewards=[];terminal=[];states=[]
    worst=0.
    try:
        for i,action in enumerate(actions):
            before=env.last_policy_observation.copy()
            worst=max(worst,float(np.max(np.abs(before-expected[i]))))
            if worst>1e-5:
                raise AssertionError("Teacher observation replay mismatch")
            state=pack_state(env);states.append(state)
            # Check complete state restoration and successor equivalence at
            # periodic anchors, including the initial null-axis None state.
            if i%100==0:
                np.testing.assert_allclose(unpack_state(env,state),before,atol=1e-6,rtol=0)
            x,q=env._x.copy(),env._q.copy()
            after,reward,done,truncated,info=env.step(action)
            if not env._last_direct_accepted or info["projection_calls"] or info["recovery_calls"]:
                raise AssertionError("Invalid demo transition")
            verdict=audit_motion(env,audit,x,q)
            if verdict["violations"]:
                raise AssertionError(str(verdict["violations"]))
            if bool(done or truncated)!=(i==len(actions)-1):
                raise AssertionError("Teacher terminal flags disagree with episode boundary")
            obs.append(before);nxt.append(after);rewards.append(reward/100.);terminal.append(float(done))
        if info["termination"]!="end":
            raise AssertionError("Incomplete teacher episode")
        name=record["case"].replace("/","__")
        destination=Path(out)/f"{name}.npz"
        np.savez_compressed(destination,observations=np.asarray(obs,np.float32),actions=actions,
            next_observations=np.asarray(nxt,np.float32),rewards=np.asarray(rewards,np.float32),
            terminals=np.asarray(terminal,np.float32),states=np.asarray(states),
            critical=critical_frames(np.asarray(obs),actions))
        row=dict(case=record["case"],file=str(destination),transitions=len(actions),
            terminal_count=sum(terminal),passed=True,max_observation_replay_error=worst,
            source_sha256=sha(source),transition_sha256=sha(destination))
        dump(Path(out)/f"{name}.json",row)
        return row
    finally:
        env.close()


def prepare(source_root,out,workers=8):
    root=Path(source_root).resolve();out=Path(out).resolve()
    out.mkdir(parents=True,exist_ok=False)
    if not json.loads((root/"teacher/gate.json").read_text())["passed"]:
        raise ValueError("Teacher gate failed")
    records=json.loads((root/"dataset/train.json").read_text())["records"]
    episodes={r["case"]:r for r in json.loads((root/"teacher/episodes.json").read_text())}
    rows=[]
    with ProcessPoolExecutor(max_workers=workers,mp_context=mp.get_context("spawn")) as pool:
        futures={pool.submit(replay_case,r,episodes[r["case"]]["dataset_file"],str(out)):r for r in records}
        for future in as_completed(futures):
            try:row=future.result()
            except Exception as exc:row=dict(case=futures[future]["case"],passed=False,error=repr(exc))
            rows.append(row);dump(out/"episodes.json",rows);print(json.dumps(row),flush=True)
    result=dict(passed=len(rows)==len(records) and all(r["passed"] for r in rows),cases=len(records),
        transitions=sum(r.get("transitions",0) for r in rows),reward_scale=.01,
        failures=[r for r in rows if not r["passed"]],source_root=str(root),source_sha256=sha(__file__))
    dump(out/"gate.json",result)
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source-root",required=True);p.add_argument("--output",required=True)
    p.add_argument("--workers",type=int,default=8)
    a=p.parse_args();prepare(a.source_root,a.output,a.workers)
