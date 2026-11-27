"""Failure records must survive independent audit and executor exceptions."""
import io
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from tools.gfr1_remount_evaluate import FIELDS, rollout


class FailureRecordingTests(unittest.TestCase):
    def core(self):
        env = SimpleNamespace(**{k: 0. for k in FIELDS})
        env._q = np.zeros(14)
        env._x = np.zeros(2)
        env._previous_n = None
        env._r_prev = np.zeros(2)
        env._repair_previous_raw = np.zeros(2)
        env.dt = .005
        env.traj_dt = .01
        env.d_safe = .005
        env._traj = {"x": np.zeros((2, 2))}
        env.lookup = SimpleNamespace(joint_lower=-np.ones(14), joint_upper=np.ones(14))
        env.reset = lambda **kw: None
        return env

    def test_audit_failure_is_saved_and_not_counted_as_completed(self):
        env = self.core()
        env.method_step = lambda *args: ((None, 0., False, True,
            {"beta_executed": 1., "termination": "end"}), True, None)
        sink = io.StringIO()
        record = dict(case="test", model="unused", valid=True, initial_clearance_m=.006)
        verdict = dict(violations=["clearance"], endpoint_clearance_m=.004, swept_clearance_m=.004)
        with patch("tools.gfr1_remount_evaluate.CollisionChecker"), patch(
                "tools.gfr1_remount_evaluate.audit_motion", return_value=verdict):
            row = rollout(env, record, "C", sink)
        self.assertFalse(row["completed"])
        self.assertEqual(row["termination"], "audit_failed")
        self.assertEqual(row["steps"], 1)
        self.assertEqual(json.loads(sink.getvalue())["audit"]["violations"], ["clearance"])

    def test_executor_exception_is_a_failure_record(self):
        env = self.core()
        def broken(*args):
            raise ArithmeticError("test failure")
        env.method_step = broken
        sink = io.StringIO()
        record = dict(case="test", model="unused", valid=True, initial_clearance_m=.006)
        with patch("tools.gfr1_remount_evaluate.CollisionChecker"):
            row = rollout(env, record, "B", sink)
        self.assertFalse(row["completed"])
        self.assertEqual(row["termination"], "exception")
        self.assertIn("ArithmeticError", json.loads(sink.getvalue())["failure_traceback"])

    def test_invalid_initialization_stays_in_denominator(self):
        env = self.core()
        record = dict(case="test", model="unused", valid=False, initial_clearance_m=.004)
        with patch("tools.gfr1_remount_evaluate.CollisionChecker"):
            row = rollout(env, record, "B", io.StringIO())
        self.assertFalse(row["completed"])
        self.assertEqual(row["termination"], "initialization_invalid")
        self.assertEqual(row["steps"], 0)


if __name__ == "__main__":
    unittest.main()
