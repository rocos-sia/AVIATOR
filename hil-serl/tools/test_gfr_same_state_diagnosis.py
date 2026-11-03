"""Exercise diagnostic tracing against unchanged original solver outcomes."""
import numpy as np
import pytest

from tools.gfr1_same_state_diagnosis import ProjectTrace, violations
from tools.validate_branch_connector import project_r


class Checker:
    def __init__(self, curvature=0.):
        self.curvature = curvature

    def check(self, x, q):
        return .01 - self.curvature * q[0] ** 2, False


@pytest.mark.parametrize('velocity,curvature,stage,count', [
    (.1, 0., 'euler_accepted', 1),
    (3., 0., 'linear_slice_empty', 0),
    (1., 1000., 'euler_backtracking_exhausted', 6),
])
def test_trace_preserves_solver_and_captures_every_trial(velocity, curvature, stage, count):
    q = np.zeros(14)
    h = np.zeros(14)
    h[0] = velocity
    n = np.zeros((14, 2))
    n[12:, :] = np.eye(2)
    args = (Checker(curvature), np.zeros(2), np.zeros(2), q, h, n,
            np.array([.4, -.2]), -np.ones(14) * 4, np.ones(14) * 4, .005)
    expected = project_r(*args)
    actual, trace = ProjectTrace(project_r).call(*args)
    np.testing.assert_array_equal(actual[0], expected[0])
    assert actual[1] == expected[1]
    assert trace['stage'] == stage
    assert len(trace['trials']) == count
    if curvature:
        assert all(t['violated'] == ['endpoint_clearance', 'swept_clearance'] for t in trace['trials'])
    if stage == 'linear_slice_empty':
        assert trace['empty_at_constraint'] == 'joint_0_speed_upper'


def test_speed_limit_and_contact_not_mislabeled_as_clearance():
    assert violations(.01, .01, False, False, 2., .1, .005, 1.5) == ['joint_speed']
    assert violations(.01, .01, True, False, 1., -.001, .005, 1.5) == ['endpoint_contact', 'joint_limit']
