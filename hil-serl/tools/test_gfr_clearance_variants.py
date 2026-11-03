"""Regression contracts for the isolated clearance ablation."""
import json
from pathlib import Path
import numpy as np
from tools.gfr1_clearance_variants import inward_step, pair_distances, use_projector
from tools.gfr1_recovery_boundary_ablation import evaluate_candidate
from experiments.aviator_manifold.gfr_env import GFRManifoldEnv

ROOT = Path('outputs/differential_lift_feasibility_20261103/gfr_recovery_root_cause')

def test_backtrack_retains_offset_affine_set():
    polygon = [np.array([.4,-1]),np.array([1.,-1]),np.array([1.,1]),np.array([.4,1])]
    r = np.array([.5,0.])
    assert (r*.5)[0] < .4  # historical shrink-to-zero leaves the feasible set
    for _ in range(6):
        r = inward_step(r,polygon)
        assert .4 <= r[0] <= 1. and abs(r[1]) <= 1.

def test_degenerate_polygon_anchor():
    for poly in ([[.4,.2]],[[.4,.2],[.8,.2]]):
        result=inward_step(np.array(poly[0]),poly)
        assert abs(result[1]-.2)<1e-15
        assert .4 <= result[0] <= .8

def test_geometry_and_original_equivalence():
    e=json.loads((ROOT/'pilot/events.jsonl').read_text())
    core=GFRManifoldEnv(manifold_dir='hil-serl/data/aviator/manifold_phi_stale_pitch-10deg',
        model_path=e['model'],trajectory_dir='hil-serl/data/aviator/trajectory_source',
        model_manifest='outputs/differential_lift_feasibility_20261103/gfr_final60k_holdout/selection/batch_0_manifest.json',split='test')
    core.reset(seed=20262006,options={'trajectory_index':6})
    core._q=np.array(e['q']);core._x=np.array(e['x']);core._u=e['u'];core._previous_n=np.array(e['previous_n'])
    ds=pair_distances(core.checker,core._x,core._q)
    assert len(ds)>1 and abs(ds.min()-core.checker.check(core._x,core._q)[0])<1e-12
    for raw in (e['policy_raw'],e['recovery_raw']):
        for beta in core.governor_schedule:
            ref,_=evaluate_candidate(core,raw,beta,boundary=True)
            with use_projector('original'):
                actual,_=evaluate_candidate(core,raw,beta,boundary=True)
            assert actual['feasible']==ref['feasible']
            if ref['feasible']:
                np.testing.assert_array_equal(actual['q_next'],ref['q_next'])
                np.testing.assert_array_equal(actual['r_exec'],ref['r_exec'])

def test_original_polygon_tolerance_does_not_abort_affine():
    from tools.gfr1_clearance_variants import make_projector
    from tools.validate_branch_connector import project_r
    class ConstantChecker:
        def check(self,x,q):return 1.,False
    q=np.zeros(14);h=q.copy();h[0]=1.1;n=np.zeros((14,2));n[0,0]=1.
    args=(ConstantChecker(),np.zeros(2),np.zeros(2),q,h,n,np.array([.40000001,0.]),np.full(14,-10.),np.full(14,10.),.005)
    r,old=project_r(*args)
    r2,new=make_projector('affine')(*args)
    assert old['feasible'] and new['feasible']
    np.testing.assert_array_equal(r,r2)
