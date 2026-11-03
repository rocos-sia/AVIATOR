import numpy as np
import pytest
from pathlib import Path
from tools.gfr1_prepare_60k_holdout import choose
from tools.gfr1_prepare_unseen_regression import trajectory_signature
from tools.gfr1_run_60k_holdout import summarize


def data(path,value):
    np.savez(path,t=np.arange(3)*.01,x=np.full((3,2),value),xdot=np.ones((3,2)),xddot=np.zeros((3,2)))


def test_selection_excludes_used_names_and_duplicate_content(tmp_path):
    paths=[tmp_path/f'traj_{i:04d}.npz' for i in range(5)]
    for i,p in enumerate(paths):data(p,i)
    data(paths[3],2)
    selected,rejected=choose(paths,{'traj_0000.npz'},{trajectory_signature(paths[1])},2)
    assert selected==[paths[2],paths[4]]
    assert len(rejected)==3
    with pytest.raises(RuntimeError):choose(paths,{'traj_0000.npz'},{trajectory_signature(paths[1])},3)


def test_eval_copy_only_changes_wiring_and_role():
    root=Path(__file__).parent
    source=(root/'gfr1_repaired_eval.py').read_text()
    expected=source.replace('"evaluation_role": "development gate, not untouched test"','"evaluation_role": "prospective new-trajectory test; same development-tested perturbation models"')
    expected=expected.replace('from experiments.aviator_gfr.config import TrainConfig',
        'from experiments.aviator_gfr import config as _gfr_config\nfrom tools.gfr1_repaired_env import RepairedGFRManifoldEnv\n_gfr_config.GFRManifoldEnv = RepairedGFRManifoldEnv\nTrainConfig = _gfr_config.TrainConfig')
    assert (root/'gfr1_holdout60k_eval.py').read_text()==expected


def test_partial_or_duplicate_evaluation_cannot_be_complete():
    with pytest.raises(ValueError):summarize([])
    with pytest.raises(ValueError):summarize([{'trajectory_path':'same'}]*20)
