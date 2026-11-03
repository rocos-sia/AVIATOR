"""Opt-in whole-path adapter: original repaired backend plus candidate variant."""
import json,os
from pathlib import Path
from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
from tools.gfr1_clearance_variants import use_projector
from tools.gfr1_same_state_diagnosis import safe

class ClearanceRegressionEnv(RepairedGFRManifoldEnv):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.variant=os.environ['AVIATOR_CLEARANCE_VARIANT']
        self._ablation_stream=None

    def step(self,action,precomputed=None):
        calls=[]
        with use_projector(self.variant,calls.append):
            result=super().step(action,precomputed)
        obs,reward,terminated,truncated,info=result
        attempts=[t for call in calls for t in call['attempts']]
        info.update(clearance_variant=self.variant,
            projection_calls=len(calls),euler_checks=len(attempts),
            euler_endpoint_rejections=sum(not t['endpoint_valid'] for t in attempts),
            euler_swept_rejections=sum(not t['swept_valid'] for t in attempts),
            linear_empty_count=sum(c['linear_empty'] for c in calls),
            affine_violation_calls=sum(c['affine_violation'] for c in calls))
        directory=Path(os.environ['AVIATOR_CLEARANCE_TRACE_DIR']);directory.mkdir(exist_ok=True,parents=True)
        if self._ablation_stream is None:
            self._ablation_stream=(directory/f'pid{os.getpid()}.jsonl').open('x',buffering=1)
        keys=['policy_action_raw','selected_action_raw','action_executed_r','beta_executed','progress','termination',
              'd_actual_m','endpoint_validation','swept_validation','qdot_max_rad_s','joint_limit_margin_rad',
              'alternative_recovered','boundary_correction_iterations','actual_candidate_evaluations',
              'projection_calls','euler_checks','euler_endpoint_rejections','euler_swept_rejections',
              'linear_empty_count','affine_violation_calls','executor_wall_s']
        row={k:info.get(k) for k in keys};row.update(episode=self._repair_episode,step=self._repair_cycle,variant=self.variant)
        self._ablation_stream.write(json.dumps(safe(row),allow_nan=False)+'\n')
        return result

    def close(self):
        if self._ablation_stream is not None:
            self._ablation_stream.close();self._ablation_stream=None
        super().close()
