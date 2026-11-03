import numpy as np
from types import SimpleNamespace

from tools.gfr1_recovery_boundary_ablation import tightened_bounds


def test_defect_compensation_does_not_relax_physical_bounds():
    lo,hi=np.array([-1.,-1.]),np.array([1.,1.])
    delta=np.array([.01,-.02])
    new_lo,new_hi,l,u=tightened_bounds(lo,hi,delta,np.zeros(2),np.zeros(2))
    np.testing.assert_allclose(new_lo,[-1.,-.98])
    np.testing.assert_allclose(new_hi,[.99,1.])
    # Any Euler point in the compensated box remains within the physical box
    # after adding the frozen measured defect.
    assert np.all(new_lo+delta>=lo) and np.all(new_hi+delta<=hi)
    lo2,hi2,_,_=tightened_bounds(lo,hi,-delta/2,l,u)
    assert np.all(lo2>=new_lo) and np.all(hi2<=new_hi)


def test_zero_defect_keeps_bounds_exactly():
    lo,hi=np.array([-2.,-.4]),np.array([1.,.6])
    nl,nh,_,_=tightened_bounds(lo,hi,np.zeros(2),np.zeros(2),np.zeros(2))
    np.testing.assert_array_equal(nl,lo)
    np.testing.assert_array_equal(nh,hi)


def test_alternatives_only_after_policy_exhaustion(monkeypatch):
    from tools import gfr1_recovery_boundary_ablation as module
    seen=[]
    def evaluator(core,action,beta,boundary):
        seen.append((np.asarray(action).copy(),beta,boundary))
        return {'feasible': bool(np.all(np.asarray(action)==0))}, []
    monkeypatch.setattr(module,'evaluate_candidate',evaluator)
    core=SimpleNamespace(governor_schedule=(1.,.5))
    picked,_,exhausted=module.select_action(core,np.zeros(2),np.ones(2),False,True,np.random.default_rng(1))
    assert not exhausted and len(seen)==1
    seen.clear()
    picked,_,exhausted=module.select_action(core,np.ones(2),-np.ones(2),False,True,np.random.default_rng(1))
    assert exhausted and len(seen)==5  # 2 policy + 2 previous + first zero.
    np.testing.assert_array_equal(picked[0],np.zeros(2))
    seen.clear()
    picked,_,_=module.select_action(core,np.ones(2),np.zeros(2),True,False,np.random.default_rng(1))
    assert picked is None and len(seen)==2
    assert all(s[2] for s in seen)


def test_real_snapshots_boundary_fix_preserves_checker_and_separates_failure_classes():
    import copy
    import pickle
    from pathlib import Path
    import pytest
    from experiments.aviator_manifold import gfr_env
    from tools.gfr1_recovery_boundary_ablation import evaluate_candidate, independent_check
    root=Path(__file__).resolve().parents[2]
    snapshots=root/'outputs/differential_lift_feasibility_20261101/gfr_same_state_40k_v2'
    if not snapshots.exists():
        pytest.skip('Local immutable regression snapshots unavailable')
    # Repo root is one level above hil-serl.
    original_project=gfr_env.project_r
    for index,expected in [(4,True),(0,False)]:
        with (snapshots/f'episode_{index:02d}/pre_stop_state.pkl').open('rb') as f:
            state=pickle.load(f)
        core=gfr_env.GFRManifoldEnv(
            str(root/'hil-serl/data/aviator/manifold_phi_stale_pitch-10deg'),
            state['model'],str(root/'hil-serl/data/aviator/trajectory_source'),
            model_manifest=state['frozen']['eval_manifest'],mode='structured')
        core.reset(options={'trajectory_index':index})
        core._traj=copy.deepcopy(state['trajectory'])
        for key,value in state['fields'].items():setattr(core,key,copy.deepcopy(value))
        original,_=evaluate_candidate(core,state['action'],1.,False)
        repaired,history=evaluate_candidate(core,state['action'],1.,True)
        assert not original['feasible']
        assert repaired['feasible']==expected
        assert gfr_env.project_r is original_project
        np.testing.assert_array_equal(core._q,state['fields']['_q'])
        if expected:
            assert history[0]['violated']==['joint_limit']
            assert repaired['joint_limit_margin_rad']>=-1e-9
            independent_check(core,core._x,core._q,repaired)
        else:
            assert len(history)==1  # Clearance failure must not trigger joint compensation.
