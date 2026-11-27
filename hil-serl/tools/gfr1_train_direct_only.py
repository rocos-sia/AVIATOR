"""Fresh SAC with reject-and-terminate dynamics and no corrective executor.

Device is explicit; CUDA requests never silently fall back to CPU. Environment
workers run in separate processes. --smoke produces unqualified debug artifacts.
"""
import argparse
from functools import partial
import json
import os
from pathlib import Path
import numpy as np
from tools.audit_static_collision import CollisionChecker
from tools.gfr1_direct_only_env import DirectOnlyEnv
from tools.gfr1_remount_evaluate import ROOT, dump, sha, rollout, audit_motion
from tools.gfr1_same_state_diagnosis import safe


def forbidden(*args, **kwargs):
    raise AssertionError("Projection/recovery called in direct-only training")


def install_guards():
    from experiments.aviator_manifold import gfr_env
    from tools import gfr1_repaired_env
    gfr_env.project_r = forbidden
    gfr1_repaired_env.select_action = forbidden


class TrainingDirectEnv(DirectOnlyEnv):
    def __init__(self, dataset, output, rank):
        manifest=Path(dataset)/"train.json"
        first=json.loads(manifest.read_text())["models"][0]
        data=ROOT/"hil-serl/data/aviator"
        super().__init__(str(data/"manifold_phi_stale_pitch-10deg"),first,str(data/"trajectory_source"),model_manifest=str(manifest))
        self.stream=(Path(output)/f"worker_{rank}.jsonl").open("x",buffering=1)
        self.worker_rank=rank
        self.transitions=0

    def _load_trajectory(self,index=None):
        index=int(self.np_random.integers(len(self._model_records))) if index is None else int(index)
        self.current_case=self._model_records[index]["case"]
        return super()._load_trajectory(index)

    def reset(self,*,seed=None,options=None):
        result=super().reset(seed=seed,options=options)
        self.audit=CollisionChecker(self.model_path)
        self.episode_steps=0
        self.episode_return=0.
        self.stream.write(json.dumps(dict(event="reset",case=self.current_case,transition=self.transitions))+"\n")
        return result

    def step(self,action):
        before_x,before_q=self._x.copy(),self._q.copy()
        before_obs=self.last_policy_observation.copy()
        result=super().step(action)
        obs,reward,terminated,truncated,info=result
        self.transitions+=1;self.episode_steps+=1;self.episode_return+=reward
        verdict=audit_motion(self,self.audit,before_x,before_q) if info["beta_executed"]>0 else None
        self.stream.write(json.dumps(safe(dict(event="transition",case=self.current_case,transition=self.transitions,
            observation=before_obs,action=action,accepted=info["beta_executed"]>0,
            progress=info["progress"],termination=info["termination"],reward=reward,
            reward_components=info["reward_components"],direct_trial=self._json_safe_candidate(self._last_direct_candidate),
            independent_audit=verdict)),allow_nan=False)+"\n")
        if verdict and verdict["violations"]:
            raise RuntimeError("Accepted motion failed independent audit: "+str(verdict["violations"]))
        if not np.isfinite(obs).all() or not np.isfinite(reward):
            raise FloatingPointError("Nonfinite transition")
        if terminated or truncated:
            self.stream.write(json.dumps(dict(event="episode_end",case=self.current_case,
                transition=self.transitions,steps=self.episode_steps,reward=self.episode_return,
                termination=info["termination"],progress=info["progress"]))+"\n")
        return result

    def close(self):
        self.stream.close()
        super().close()


def make_worker(dataset,output,rank):
    install_guards()
    return TrainingDirectEnv(dataset,output,rank)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--device",choices=["cpu","cuda"],required=True)
    p.add_argument("--num-envs",type=int,default=4)
    p.add_argument("--steps",type=int,default=60000)
    p.add_argument("--eval-interval",type=int,default=20000)
    p.add_argument("--learning-starts",type=int,default=2000)
    p.add_argument("--seed",type=int,default=11)
    p.add_argument("--smoke",action="store_true")
    a=p.parse_args()
    if a.num_envs<1 or a.steps<1 or a.eval_interval<1 or a.steps%a.num_envs or a.eval_interval%a.num_envs:
        p.error("Positive steps/evaluation interval must be divisible by num-envs")
    import torch
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    if a.device=="cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but this PyTorch installation has no usable CUDA device; no CPU fallback")
    torch.set_num_threads(1)
    if not json.loads((a.dataset/"gate.json").read_text())["passed"]:
        raise RuntimeError("Dataset prerequisite failed")
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    configuration=dict(device=a.device,num_envs=a.num_envs,steps=a.steps,seed=a.seed,smoke=a.smoke,
        environment="DirectOnlyEnv",actor_initialization="fresh random",observation_dim=98,
        rejection="terminate; no state commit; no QP/governor/alternative action",
        checkpoint_success_metric="whole development trajectories completed without any corrective execution",
        independent_audit=True,torch=torch.__version__,cuda=torch.version.cuda,
        training_manifest_sha256=sha(a.dataset/"train.json"),
        development_manifest_sha256=sha(a.dataset/"development.json"),
        source_sha256={str(Path(__file__)):sha(__file__),str(Path(__file__).with_name("gfr1_direct_only_env.py")):sha(Path(__file__).with_name("gfr1_direct_only_env.py"))})
    dump(out/"config.json",configuration)
    factories=[partial(make_worker,str(a.dataset.resolve()),str(out),rank) for rank in range(a.num_envs)]
    env=DummyVecEnv(factories) if a.num_envs==1 else SubprocVecEnv(factories,start_method="spawn")
    install_guards()
    records=json.loads((a.dataset/"development.json").read_text())["records"]
    if a.smoke:records=records[:1]
    data=ROOT/"hil-serl/data/aviator"
    evaluation=DirectOnlyEnv(str(data/"manifold_phi_stale_pitch-10deg"),records[0]["model"],str(data/"trajectory_source"))
    summaries=[]
    class Checkpoints(BaseCallback):
        def _on_step(self):
            if self.num_timesteps%a.eval_interval==0 or self.num_timesteps>=a.steps:
                checkpoint=out/f"{'smoke_' if a.smoke else ''}checkpoint_{self.num_timesteps}"
                self.model.save(checkpoint)
                rows=[]
                with (out/f"evaluation_{self.num_timesteps}.jsonl").open("x",buffering=1) as sink:
                    for record in records:
                        row=rollout(evaluation,record,"R",sink,
                            policy=lambda core:self.model.predict(core.last_policy_observation,deterministic=True)[0])
                        row["rejected_steps"]=row.pop("fallback_steps")
                        row["fallback_steps"]=0
                        rows.append(row)
                summary=dict(transitions=self.num_timesteps,checkpoint=str(checkpoint)+".zip",
                    completed=sum(r["completed"] for r in rows),cases=len(rows),episodes=rows)
                summaries.append(summary);dump(out/"evaluations.json",summaries)
                print(json.dumps({k:v for k,v in summary.items() if k!="episodes"}),flush=True)
            return True
    model=SAC("MlpPolicy",env,device=a.device,seed=a.seed,learning_starts=a.learning_starts,
        learning_rate=3e-4,batch_size=256,buffer_size=100000,gamma=.995,tau=.005,
        train_freq=1,gradient_steps=a.num_envs,policy_kwargs=dict(net_arch=[128,128]),verbose=0)
    try:
        model.learn(total_timesteps=a.steps,callback=Checkpoints())
        passed=not a.smoke and any(s["completed"]==s["cases"] for s in summaries)
        dump(out/"gate.json",dict(passed=passed,smoke=a.smoke,execution_completed=True,
            transitions=model.num_timesteps,scope="standalone RL without correction",
            reason="debug_run_only" if a.smoke else ("qualified" if passed else "standalone_completion_requirement_not_met")))
    except Exception as error:
        dump(out/"gate.json",dict(passed=False,execution_completed=False,reason=repr(error)))
        raise
    finally:
        env.close();evaluation.close()


if __name__=="__main__":main()
