"""Gate regressions: an evaluator crash must never be counted as zero success."""
from tools.gfr1_gate_support import TrainingGate, atomic_json, evaluate_gate


def report(rate, rc=0):
    return {'returncode': rc, 'instrumentation_unknown': 0,
            'summary': {'episodes': 10, 'completion_rate': rate}}


def test_gate_requires_two_valid_evaluations():
    assert evaluate_gate({20000: report(0.)}) == 'pending'
    assert evaluate_gate({20000: report(0.), 40000: report(0.)}) == 'stop_both_zero'
    assert evaluate_gate({20000: report(.1), 40000: report(0.)}) == 'continue'
    assert evaluate_gate({20000: report(0.), 40000: report(.1)}) == 'continue'
    assert evaluate_gate({20000: report(0.), 40000: report(0., 1)}) == 'instrumentation_error'
    bad = report(float('nan'))
    assert evaluate_gate({20000: report(0.), 40000: bad}) == 'instrumentation_error'


def test_ready_and_decision(tmp_path):
    gate = TrainingGate(tmp_path)
    gate.checkpoint_ready(40000, tmp_path / 'checkpoint_40000')
    assert (tmp_path / 'ready_40000.json').exists()
    atomic_json(tmp_path / 'decision.json', {'decision': 'continue'})
    assert gate.wait_for_decision()
    assert not gate.waiting


def test_stop_acknowledged(tmp_path):
    gate = TrainingGate(tmp_path)
    atomic_json(tmp_path / 'decision.json', {'decision': 'stop_both_zero'})
    assert not gate.wait_for_decision()
    assert gate.stop_requested()


def test_external_stop(tmp_path):
    gate = TrainingGate(tmp_path)
    (tmp_path / 'STOP').touch()
    assert not gate.wait_for_decision()
