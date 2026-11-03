import unittest
from unittest.mock import patch as mockpatch
from types import SimpleNamespace
import numpy as np
from tools.gfr1_scheduler_benchmark import Meter,patch,cached_selection,teacher_selection

class Tests(unittest.TestCase):
    def test_exclusive_partition(self):
        m=Meter()
        with m.span('total'):
            with m.span('child'):
                sum(range(100))
        self.assertAlmostEqual(sum(m.exclusive.values()),m.inclusive['total'],places=10)
    def test_class_distance_patch_tracks_new_instance(self):
        class Checker:
            def check(self):return 3
        old,new=Checker(),Checker();m=Meter()
        with patch(type(old),'check',m.wrap('distance_check',type(old).check)):
            self.assertEqual(old.check(),3);self.assertEqual(new.check(),3)
        self.assertEqual(m.counts['distance_check'],2)
    def test_patch_restored_exception(self):
        x=SimpleNamespace(v=1)
        try:
            with patch(x,'v',2):raise ValueError()
        except ValueError:pass
        self.assertEqual(x.v,1)
    def test_teacher_does_not_expand_success(self):
        core=SimpleNamespace(_repair_previous_raw=np.array([.3,.4],dtype=np.float32),governor_schedule=[1.,.5])
        with mockpatch('tools.gfr1_recovery_boundary_ablation.evaluate_candidate',return_value=({'feasible':True,'beta':1.,'d_actual_m':.01},[{'result':{}}])) as ev:
            raw,(chosen,history,exhausted),diag=teacher_selection(core)
            self.assertEqual(ev.call_count,1);self.assertFalse(exhausted)
            np.testing.assert_array_equal(raw,core._repair_previous_raw)
    def test_teacher_search_only_after_primary_exhausted(self):
        core=SimpleNamespace(_repair_previous_raw=np.array([.3,.4],dtype=np.float32),governor_schedule=[1.,.5],_repair_seed=1,_repair_cycle=0)
        def ev(c,raw,beta,boundary):
            ok=not np.array_equal(raw,c._repair_previous_raw)
            return dict(feasible=ok,beta=beta,d_actual_m=float(raw[0])),[{'result':{}}]
        with mockpatch('tools.gfr1_recovery_boundary_ablation.evaluate_candidate',side_effect=ev) as f:
            raw,(chosen,history,exhausted),diag=teacher_selection(core)
            self.assertFalse(exhausted);self.assertTrue(diag['teacher_expanded']);self.assertEqual(chosen[1]['beta'],1.)
            np.testing.assert_array_equal(raw,chosen[0])
            self.assertEqual(diag['teacher_search_candidates'],f.call_count)
            self.assertEqual(len(history),1)
            self.assertGreater(f.call_count,25)
    def test_cache_consumed_once_and_state_guarded(self):
        from tools import gfr1_repaired_env as re
        c=SimpleNamespace(_q=np.zeros(14),_x=np.zeros(2),_u=0.)
        raw=np.zeros(2);result=('chosen','history',False)
        with cached_selection(c,raw,result):
            self.assertEqual(re.select_action(c,raw,None),result)
            with self.assertRaises(AssertionError):re.select_action(c,raw,None)
        with self.assertRaises(AssertionError):
            with cached_selection(c,raw,result):
                c._q[0]=1.;re.select_action(c,raw,None)

if __name__=='__main__':unittest.main()
