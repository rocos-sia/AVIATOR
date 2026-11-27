import unittest
from unittest.mock import patch
import numpy as np
from tools.gfr1_direct_only_env import DirectOnlyEnv


class DirectOnlyTests(unittest.TestCase):
    def core(self, feasible):
        e=DirectOnlyEnv.__new__(DirectOnlyEnv)
        e._q=np.arange(14,dtype=float);e._x=np.array([.1,.2]);e._u=3.
        e._previous_n=None;e._r_prev=np.zeros(2);e.r_max=1.5
        e._repair_cycle=0;e._repair_previous_raw=np.zeros(2)
        e._traj={"x":np.zeros((20,2))}
        e._get_obs=lambda:np.zeros(98,dtype=np.float32)
        e._path_state=lambda u:(np.zeros(2),np.ones(2))
        e._candidate=lambda *args,**kwargs:({"feasible":feasible,"q_next":np.full(14,999.),"x_next":np.full(2,999.)},{})
        return e

    def test_rejected_proposal_never_moves_state_or_invokes_fallback(self):
        e=self.core(False);q=e._q.copy();x=e._x.copy()
        with patch("tools.gfr1_repaired_env.select_action",side_effect=AssertionError("fallback")),patch("experiments.aviator_manifold.gfr_env.project_r",side_effect=AssertionError("projection")):
            _,reward,terminated,truncated,info=e.step(np.array([.2,-.3]))
        np.testing.assert_array_equal(e._q,q);np.testing.assert_array_equal(e._x,x)
        self.assertEqual(e._u,3.);self.assertTrue(terminated);self.assertFalse(truncated)
        self.assertEqual(info["termination"],"direct_rejected")
        self.assertEqual(info["beta_executed"],0.)
        self.assertEqual(info["recovery_calls"],0)
        self.assertLess(reward,-1000.)

    def test_accepted_candidate_is_submitted_once_without_projection(self):
        e=self.core(True)
        returned=(np.zeros(98),0.,False,False,{"task_progress_delta":.1,"termination":"none"})
        with patch("experiments.aviator_manifold.gfr_env.GFRManifoldEnv.step",return_value=returned) as commit,patch("tools.gfr1_repaired_env.select_action",side_effect=AssertionError("fallback")):
            _,passed,_=e.action_step(np.array([.2,-.3]))
        self.assertTrue(passed);self.assertEqual(commit.call_count,1)
        np.testing.assert_array_equal(commit.call_args.args[1],np.array([.2,-.3]))
        self.assertEqual(e._repair_cycle,1)

    def test_cannot_request_projected_mode(self):
        with self.assertRaises(ValueError):self.core(True).action_step(np.zeros(2),False)


if __name__=="__main__":unittest.main()
