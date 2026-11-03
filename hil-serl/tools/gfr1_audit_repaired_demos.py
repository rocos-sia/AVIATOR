#!/usr/bin/env python3
"""Audit the retained teacher buffer and reset-step compatibility with repair.

This is explicitly not a full replay of all historical transitions.
"""
import argparse
import json
from pathlib import Path
import pickle
import numpy as np
from tools.gfr1_prepare_teacher_buffer import sha256
from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
from tools.gfr1_same_state_diagnosis import write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--demo-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[2]
    audit=json.loads((a.demo_dir/'demo_manifest.json').read_text())
    assert audit['passed'] and audit['episode_count']==100 and audit['instrumentation_unknown']==0
    buffer=a.demo_dir/'structured_visible.pkl'
    assert sha256(buffer)==audit['buffer_sha256']
    with buffer.open('rb') as f:demos=pickle.load(f)
    assert len(demos)==302125
    manifest=json.loads((a.demo_dir/'train_manifest.json').read_text())
    core=RepairedGFRManifoldEnv(str(root/'hil-serl/data/aviator/manifold_phi_stale_pitch-10deg'),
        manifest['models'][0],str(root/'hil-serl/data/aviator/trajectory_source'),
        model_manifest=str(a.demo_dir/'train_manifest.json'),mode='structured')
    rows=[]
    for index,ep in enumerate(audit['episodes']):
        seq=demos[ep['transition_start']:ep['transition_start']+ep['transition_count']]
        assert ep['termination']=='end' and abs(ep['progress']-1)<1e-9
        assert all(bool(t['dones'])==(j==len(seq)-1) for j,t in enumerate(seq))
        assert all(float(t['masks'])==float(j!=len(seq)-1) for j,t in enumerate(seq))
        before,_=core.reset(seed=11,options={'trajectory_index':index})
        tr=seq[0]
        np.testing.assert_array_equal(before['state'],tr['observations']['state'])
        after,reward,done,truncated,info=core.step(tr['actions'])
        error=float(np.max(np.abs(after['state']-tr['next_observations']['state'])))
        reward_error=abs(reward-float(tr['rewards']))
        assert info['endpoint_validation'] and info['swept_validation']
        rows.append(dict(episode=index,first_step_observation_max_abs_error=error,
                         first_step_reward_abs_error=reward_error,accepted=True,
                         alternative_recovered=info['alternative_recovered']))
    passed=all(r['first_step_observation_max_abs_error']<=2e-6 and r['first_step_reward_abs_error']<=1e-6 for r in rows)
    result=dict(passed=passed,episodes=100,transitions=302125,buffer_sha256=audit['buffer_sha256'],
        initialization='random SAC; demos are replay prior, no BC initialization',
        limitations='Reset and first accepted step checked for each of 100 tasks. Interior transitions retain historical accepted teacher provenance; no claim of exhaustive new-executor replay.',
        maximum_first_step_obs_difference=max(r['first_step_observation_max_abs_error'] for r in rows),
        maximum_first_step_reward_difference=max(r['first_step_reward_abs_error'] for r in rows),rows=rows)
    write_json(a.output,result);print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    assert passed


if __name__=='__main__':main()
