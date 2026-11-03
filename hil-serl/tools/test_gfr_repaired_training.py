import copy
import json
import pickle
from pathlib import Path
import numpy as np
import pytest
from experiments.aviator_manifold.gfr_env import GFRManifoldEnv
from tools.gfr1_repaired_env import RepairedGFRManifoldEnv
from tools.gfr1_recovery_boundary_ablation import select_action, independent_check

ROOT = Path(__file__).resolve().parents[2]
SNAP = ROOT/'outputs/differential_lift_feasibility_20261101/gfr_same_state_40k_v2'


def make_env(cls, state):
    return cls(str(ROOT/'hil-serl/data/aviator/manifold_phi_stale_pitch-10deg'),
               state['model'], str(ROOT/'hil-serl/data/aviator/trajectory_source'),
               model_manifest=state['frozen']['eval_manifest'], mode='structured')


@pytest.mark.parametrize('index', [0, 4])
def test_training_repair_matches_frozen_selector_and_independent_checker(index):
    with (SNAP/f'episode_{index:02d}/pre_stop_state.pkl').open('rb') as f: state=pickle.load(f)
    core=make_env(RepairedGFRManifoldEnv,state)
    core.reset(seed=20262000+index,options={'trajectory_index':index})
    core._traj=copy.deepcopy(state['trajectory'])
    for k,v in state['fields'].items():setattr(core,k,copy.deepcopy(v))
    core._repair_previous_raw=np.asarray(state['previous_raw']).copy()
    chosen, history, _=select_action(core,state['action'],state['previous_raw'],True,True,
                                    np.random.default_rng(core._repair_seed))
    assert chosen is not None
    independent_check(core,core._x.copy(),core._q.copy(),chosen[1])
    qn=chosen[1]['q_next'].copy()
    obs,reward,done,trunc,info=core.step(state['action'])
    np.testing.assert_array_equal(core._q,qn)
    np.testing.assert_array_equal(info['policy_action_raw'],np.asarray(state['action'],float))
    assert reward==sum(info['reward_components'].values())
    assert info['actual_candidate_evaluations']==sum(len(h['trials']) for h in history)
    assert info['endpoint_validation'] and info['swept_validation']
    assert np.isfinite(obs['state']).all()
    core.reset(seed=1,options={'trajectory_index':index})
    assert core._repair_cycle==0
    np.testing.assert_array_equal(core._repair_previous_raw,np.zeros(2))


def test_exhausted_stop_keeps_original_reward_and_state(monkeypatch):
    with (SNAP/'episode_00/pre_stop_state.pkl').open('rb') as f:state=pickle.load(f)
    core=make_env(RepairedGFRManifoldEnv,state)
    core.reset(seed=1,options={'trajectory_index':0})
    rejected={'feasible':False,'reason':'empty_slice'}
    monkeypatch.setattr(core,'_candidate',lambda *a,**k:(dict(rejected),{}))
    raw=np.zeros(2)
    original=GFRManifoldEnv.step(core,raw)
    q=core._q.copy();u=core._u
    repaired=core._failed_step(raw,[dict(rejected) for _ in core.governor_schedule])
    assert original[1:4]==repaired[1:4]
    assert repaired[1]<-100
    np.testing.assert_array_equal(q,core._q)
    assert core._u==u
    assert repaired[4]['reward_components']==original[4]['reward_components']
    assert repaired[4]['beta_executed']==0


def test_repair_does_not_consume_reset_rng():
    with (SNAP/'episode_00/pre_stop_state.pkl').open('rb') as f:state=pickle.load(f)
    old=make_env(GFRManifoldEnv,state);new=make_env(RepairedGFRManifoldEnv,state)
    for seed in (11,None,None):
        o,_=old.reset(seed=seed);n,_=new.reset(seed=seed)
        np.testing.assert_array_equal(o['state'],n['state'])


def test_scratch_entry_rejects_sac_initialization(monkeypatch):
    import sys
    from tools.gfr1_repaired_entry import main
    monkeypatch.setattr(sys,'argv',['entry','train','--sac_init_checkpoint_path=old'])
    with pytest.raises(ValueError,match='from scratch'):main()
