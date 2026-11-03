from tools.gfr1_bc_support import checkpoint_hash, qualify_bc, select_covered_records


def test_previous_action_mask_preserves_other_inputs_and_source():
    import numpy as np
    import pytest
    from tools.gfr1_bc_support import policy_state
    for shape in ((40,), (1, 40), (5, 1, 40)):
        original = np.arange(np.prod(shape), dtype=np.float32).reshape(shape)
        snapshot = original.copy()
        masked = policy_state(original, True)
        assert np.array_equal(original, snapshot)
        assert np.all(masked[..., 33:35] == 0)
        assert np.array_equal(masked[..., :33], original[..., :33])
        assert np.array_equal(masked[..., 35:], original[..., 35:])
        assert np.array_equal(policy_state(original, False), original)
    with pytest.raises(ValueError):
        policy_state(np.zeros(39), True)


def report(n=10):
    return {'returncode': 0, 'instrumentation_unknown': 0, 'agent_kind': 'bc',
            'episodes': [{'model': str(i // 5), 'completed': i < n,
                          'progress': 1. if i < n else .4,
                          'minimum_executed_clearance_m': .006,
                          'max_closure_residual_inf': 1e-6,
                          'accepted_validation_violations': 0} for i in range(10)]}


def test_requires_both_checkpoints_and_stability():
    assert qualify_bc({10000: report()}) == 'pending'
    assert qualify_bc({10000: report(9), 20000: report(9)}) == 'pass'
    assert qualify_bc({10000: report(8), 20000: report()}) == 'stop_bc_not_stable'
    assert qualify_bc({10000: report(), 20000: report(0)}) == 'stop_bc_not_stable'


def test_instrumentation_and_unsafe_acceptance_never_pass():
    for field, value in [('returncode', 1), ('instrumentation_unknown', 1), ('agent_kind', 'sac')]:
        bad = report()
        bad[field] = value
        assert qualify_bc({10000: bad, 20000: report()}) == 'instrumentation_error'
    bad = report()
    bad['episodes'][0]['minimum_executed_clearance_m'] = float('nan')
    assert qualify_bc({10000: bad, 20000: report()}) == 'instrumentation_error'
    bad = report()
    bad['episodes'][0]['accepted_validation_violations'] = 1
    assert qualify_bc({10000: bad, 20000: report()}) == 'stop_validation_violation'
    bad = report()
    bad['returncode'] = 1
    assert qualify_bc({10000: report(0), 20000: bad}) == 'instrumentation_error'


def test_selection_is_independent_of_input_order():
    records = [{'model': m, 'trajectory_path': f'traj_{i:04}.npz'} for m in ('a', 'b') for i in range(50)]
    expected = select_covered_records(records)
    assert select_covered_records(list(reversed(records))) == expected
    assert len(expected) == 10
    assert expected[0]['trajectory_path'] == 'traj_0000.npz'
    assert expected[-1]['trajectory_path'] == 'traj_0049.npz'


def test_checkpoint_tamper_changes_hash(tmp_path):
    (tmp_path / 'weights').write_bytes(b'initial')
    first = checkpoint_hash(tmp_path)
    (tmp_path / 'weights').write_bytes(b'changed')
    assert checkpoint_hash(tmp_path) != first


def test_initialization_experiment_keeps_integrity_and_instrumentation_gates():
    import pytest
    from tools.gfr1_bc_support import validate_bc_warmstart_report
    q = {'evaluations': {'10000': report(0), '20000': report(0)},
         'decision': 'stop_bc_not_stable', 'training': {}, 'sources_unchanged': True}
    with pytest.raises(ValueError):
        validate_bc_warmstart_report(q)
    assert validate_bc_warmstart_report(q, True) == 'stop_bc_not_stable'
    q['training']['mask_previous_action'] = True
    with pytest.raises(ValueError):
        validate_bc_warmstart_report(q, True)
    q['training'].clear()
    q['evaluations']['20000']['returncode'] = 1
    q['decision'] = 'instrumentation_error'
    with pytest.raises(ValueError):
        validate_bc_warmstart_report(q, True)
