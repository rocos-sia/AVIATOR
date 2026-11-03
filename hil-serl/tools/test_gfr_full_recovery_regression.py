import pytest
from tools.gfr1_full_recovery_regression import full_budget, completed
from tools.gfr1_full_recovery_regression import execute_or_stop


def test_full_budget_covers_slowed_entire_trajectory():
    assert full_budget(804, .01, .005, .125) == 12850
    assert full_budget(804, .01, .005, .125) > 2000


def test_completion_requires_actual_end_not_budget_or_progress_only():
    assert completed('end', 1.)
    assert not completed('full_horizon_budget_exhausted', 1.)
    assert not completed('governor_exhausted', .999)
    assert not completed('end', .24)


def test_invalid_budget_fails_before_execution():
    with pytest.raises(ValueError):
        full_budget(804, .01, .005, 0.)


def test_rejected_candidate_never_enters_execution_api():
    from types import SimpleNamespace
    env = SimpleNamespace(unwrapped=SimpleNamespace(_u=2., _traj={'x':[0]*5}))
    obs = {'state':[1,2]}
    result = execute_or_stop(env, obs, [0,0], None, [{'feasible':False}])
    assert result[0] is obs and result[2] and not result[3]
    assert result[4]['termination'] == 'governor_exhausted'
    assert result[4]['progress'] == .5
    assert env.unwrapped._u == 2.


def test_execution_receives_policy_raw_and_accepted_alternative():
    from types import SimpleNamespace
    seen = []
    env = SimpleNamespace(step=lambda raw, candidate: seen.append((raw,candidate)))
    accepted = {'feasible':True}
    execute_or_stop(env, {}, [1,1], ([0,0],accepted), [accepted])
    assert seen == [([1,1], {'accepted':accepted, 'candidates':[accepted]})]
