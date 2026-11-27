"""Semantic checks for no-training foresight qualification."""
import copy
import unittest
from types import SimpleNamespace
import numpy as np

from tools.gfr7_n0_qualify import (
    aggregate, compare_replay, prefix_cost, reference_pair, event_horizon,
    analyze_state, reversal_indices,
)


def synthetic(costs, completed=True):
    rows = [dict(u=(i+1)*.5, q=[0.]*14, executed=[0., 0.],
                 candidate_calls=c, direct=True, elapsed_ms=1.)
            for i, c in enumerate(costs)]
    return dict(completed=completed, rows=rows, steps=len(rows),
                candidate_calls=sum(costs), all_direct=True, total_compute_ms=len(rows))


class N0Tests(unittest.TestCase):
    def test_common_progress_not_candidate_length(self):
        result = synthetic([1, 2, 3])
        self.assertEqual(prefix_cost(result, .75), (0, 3, 2))
        self.assertEqual(prefix_cost(result, 2.), (1, 0, 0))

    def test_reversal_across_stop_is_detected(self):
        velocity = np.array([[1., 0.], [0., 0.], [0., 0.], [-1., 0.]])
        np.testing.assert_array_equal(reversal_indices(velocity), [3])

    def test_regret_eta_and_failure_are_distinct(self):
        # The third proposal looks cheapest initially but later costs more.
        results = [synthetic([3, 7]), synthetic([2, 2]), synthetic([1, 20])]
        value = analyze_state(results, {'one_step': .005}, 0., .01)
        self.assertEqual(value['available_gain'], 6)
        self.assertEqual(value['selections']['one_step']['regret'], 17)
        self.assertLess(value['selections']['one_step']['eta'], 0)
        value = analyze_state([synthetic([2], False)], {'one_step': .01}, 0., .01)
        self.assertTrue(value['all_failed'])
        self.assertIsNone(value['selections']['one_step']['index'])

    def test_determinism_excludes_only_times(self):
        original = synthetic([1, 2]); other = copy.deepcopy(original)
        other['total_compute_ms'] = 50; other['rows'][0]['elapsed_ms'] = 99
        compare_replay(original, other)
        other['rows'][0]['executed'][0] = .2
        with self.assertRaises(AssertionError):
            compare_replay(original, other)

    def test_future_reference_shares_prefix_and_reverses(self):
        t = np.arange(301)*.01
        trajectory = dict(x=np.c_[.1*t, .01*t], xdot=np.tile([.1, .01], (len(t), 1)),
                          xddot=np.zeros((len(t), 2)))
        env = SimpleNamespace(_traj=trajectory, _u=0., traj_dt=.01)
        variants, meta = reference_pair(env)
        for k in ('x', 'xdot', 'xddot'):
            np.testing.assert_array_equal(variants[0][k][:36], variants[1][k][:36])
        dim = meta['reverse_dimension']
        self.assertGreater(variants[0]['xdot'][80, dim]*variants[1]['xdot'][80, dim]*-1, 0)
        env._traj = variants[1]
        self.assertIsNotNone(event_horizon(env, 125.))
        self.assertIsNone(event_horizon(env, 40.))

    def test_completion_regression_blocks_gate(self):
        rows = []
        for i in range(4):
            selections = {'one_step': dict(completed=True, cost=10),
                          'h025': dict(completed=i != 3, cost=2 if i != 3 else None)}
            rows.append(dict(analysis=dict(available_gain=8, future_call_range=8,
                                          first_q_range_rad=.001, baseline_completed=True,
                                          baseline_cost=10, selections=selections)))
        result = aggregate(rows)
        self.assertNotIn('h025', result['foresight_candidates'])
        self.assertEqual(result['horizon_comparisons']['h025']['completion_regressions'], 1)


if __name__ == '__main__':
    unittest.main()
