import numpy as np
import pytest
from tools.gfr1_prepare_unseen_regression import trajectory_signature


def test_duplicate_content_detected_across_splits_and_archive_metadata(tmp_path):
    data=dict(t=np.arange(5)*.01, x=np.arange(10).reshape(5,2),
              xdot=np.ones((5,2)), xddot=np.zeros((5,2)))
    a=tmp_path/'train.npz'; b=tmp_path/'test_renamed.npz'
    np.savez(a,**data)
    np.savez_compressed(b,**data,irrelevant_metadata='different archive')
    assert trajectory_signature(a)==trajectory_signature(b)
    data['xdot'][2,1]=2.
    np.savez(b,**data)
    assert trajectory_signature(a)!=trajectory_signature(b)


def test_nonfinite_trajectory_rejected_before_selection(tmp_path):
    p=tmp_path/'bad.npz'
    np.savez(p,t=[0,.01],x=[[0,0],[float('nan'),0]],xdot=np.zeros((2,2)),xddot=np.zeros((2,2)))
    with pytest.raises(ValueError,match='Non-finite'):
        trajectory_signature(p)


def test_new_runner_uses_identical_frozen_execution_helpers():
    from tools import gfr1_unseen_recovery_regression as new
    from tools import gfr1_full_recovery_regression as old
    from tools import gfr1_recovery_boundary_ablation as repair
    assert new.execute_or_stop is old.execute_or_stop
    assert new.full_budget is old.full_budget
    assert new.select_action is repair.select_action
    assert new.independent_check is repair.independent_check
