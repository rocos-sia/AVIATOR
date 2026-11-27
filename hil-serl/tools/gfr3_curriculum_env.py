"""Training-only demo resets with strict rejection, no corrective execution."""
import json
from pathlib import Path
import numpy as np
from tools.gfr1_remount_evaluate import ROOT, audit_motion
from tools.audit_static_collision import CollisionChecker
from tools.gfr1_train_direct_only import install_guards
from tools.gfr2_shared_env import SharedProposalEnv
from tools.gfr3_demo_transitions import unpack_state


def curriculum(global_steps):
    if global_steps<20000:return 64,.25
    if global_steps<60000:return 256,.25
    if global_steps<90000:return 1024,.5
    return None,1.


class CurriculumEnv(SharedProposalEnv):
    def __init__(self,source_root,demo_root,out,rank,workers):
        root=Path(source_root)
        all_records=json.loads((root/"dataset/train.json").read_text())["records"]
        models=sorted({r["model"] for r in all_records})
        if len(models)%workers:
            raise ValueError("Installation count must be divisible by workers")
        self.worker_models=models[rank::workers]
        self.records=[r for r in all_records if r["model"] in self.worker_models]
        data=ROOT/"hil-serl/data/aviator"
        super().__init__(str(data/"manifold_phi_stale_pitch-10deg"),self.worker_models[0],str(data/"trajectory_source"))
        self._model_records=self.records
        self.examples=[]
        for record in self.records:
            file=Path(demo_root)/(record["case"].replace("/","__")+".npz")
            with np.load(file) as archive:
                self.examples.append({k:archive[k].copy() for k in ("states","observations","critical")})
        self.audit=CollisionChecker(self.worker_models[0])
        self.audit_model=self.worker_models[0]
        self.rank=rank;self.workers=workers;self.local_transitions=0
        self.reset_rng=np.random.default_rng(11+rank)
        self.stream=(Path(out)/f"worker_{rank}.jsonl").open("x",buffering=1)

    def reset(self,*,seed=None,options=None):
        if seed is not None:self.reset_rng=np.random.default_rng(seed)
        # Rotate installations in blocks, keeping just one live model per
        # worker rather than compiling an installation on every short failure.
        model=self.worker_models[(self.local_transitions//256)%len(self.worker_models)]
        choices=[i for i,r in enumerate(self.records) if r["model"]==model]
        index=int(self.reset_rng.choice(choices))
        obs,info=super().reset(seed=seed,options={"trajectory_index":index})
        if self.audit_model!=model:
            self.audit=CollisionChecker(model);self.audit_model=model
        self.case=self.records[index]["case"]
        horizon,full_probability=curriculum(self.local_transitions*self.workers)
        full=bool(self.reset_rng.random()<full_probability)
        frame=0
        if not full:
            example=self.examples[index]
            critical=np.flatnonzero(example["critical"])
            if len(critical) and self.reset_rng.random()<.5:
                frame=int(self.reset_rng.choice(critical))
            else:frame=int(self.reset_rng.integers(len(example["states"])))
            obs=unpack_state(self,example["states"][frame])
            np.testing.assert_allclose(obs,example["observations"][frame],atol=1e-5,rtol=0)
        self.segment_limit=None if full else horizon
        self.episode_steps=0;self.episode_reward=0.;self.start_frame=frame
        self.stream.write(json.dumps(dict(event="reset",case=self.case,start_frame=frame,
            from_original_start=full,segment_limit=self.segment_limit,global_steps=self.local_transitions*self.workers))+"\n")
        return obs,info

    def step(self,action):
        x,q,u=self._x.copy(),self._q.copy(),self._u
        before_obs=self.last_policy_observation.copy()
        obs,reward,terminated,truncated,info=super().step(action)
        self.local_transitions+=1;self.episode_steps+=1
        reward=reward/100.
        self.episode_reward+=reward
        accepted=self._last_direct_accepted
        verdict=audit_motion(self,self.audit,x,q) if accepted else None
        if not accepted:
            np.testing.assert_array_equal(self._x,x);np.testing.assert_array_equal(self._q,q)
            assert self._u==u and terminated and not truncated
        if verdict and verdict["violations"]:
            raise RuntimeError("Accepted motion audit failed: "+str(verdict["violations"]))
        if info["projection_calls"] or info["recovery_calls"] or info["governor_calls"]:
            raise AssertionError("Corrective execution in standalone SAC")
        if not terminated and not truncated and self.segment_limit is not None and self.episode_steps>=self.segment_limit:
            truncated=True
            info["curriculum_segment_end"]=True
        info.update(accepted=accepted,case=self.case,start_frame=self.start_frame,
            scaled_reward=reward,curriculum_truncated=bool(truncated and not terminated))
        if not accepted or self.local_transitions%100==0:
            self.stream.write(json.dumps(dict(event="transition",case=self.case,transition=self.local_transitions,
                observation=before_obs.tolist(),action=np.asarray(action).tolist(),next_observation=obs.tolist(),
                before_x=x.tolist(),before_q=q.tolist(),after_x=self._x.tolist(),after_q=self._q.tolist(),
                accepted=accepted,reward=reward,terminated=terminated,truncated=truncated,
                progress=info["progress"],trial=self._json_safe_candidate(self._last_direct_candidate)))+"\n")
        if terminated or truncated:
            self.stream.write(json.dumps(dict(event="episode_end",case=self.case,start_frame=self.start_frame,
                steps=self.episode_steps,reward=self.episode_reward,progress=info["progress"],
                full_task_success=bool(self.start_frame==0 and info["termination"]=="end"),
                termination=info["termination"],curriculum_truncated=bool(truncated and not terminated)))+"\n")
        return obs,reward,terminated,truncated,info

    def close(self):
        if hasattr(self,"stream"):self.stream.close()
        super().close()


def make_worker(source_root,demo_root,out,rank,workers):
    install_guards()
    return CurriculumEnv(source_root,demo_root,out,rank,workers)
