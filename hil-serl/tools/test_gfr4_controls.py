"""Checks that protect the interpretation of the controlled experiments."""
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from tools.gfr3_train_sac_demo import fingerprint
from tools.gfr4_common import FixedReplaySampler, restore
from tools.gfr4_controlled_env import schedule, bc_weight
from tools.gfr4_train_controlled import success_retention


class Controls(unittest.TestCase):
    def test_unevaluated_episodes_are_not_failures(self):
        result=success_retention({'a','b'},[dict(case='a',completed=True)])
        self.assertEqual(result['lost_baseline_successes'],[])
        self.assertEqual(result['unevaluated_baseline_successes'],['b'])
        result=success_retention({'a','b'},[dict(case='a',completed=False),dict(case='c',completed=True)])
        self.assertEqual(result['lost_baseline_successes'],['a'])
        self.assertEqual(result['gained_successes'],['c'])

    def test_sampling_has_exact_failure_share_and_no_future_data(self):
        arrays = [np.zeros((256,n), np.float32) for n in (128,2,1,128,1)]
        arrays[2][[3,19],0] = -10; arrays[4][[3,19],0] = 1
        arrays[2][128:,0] = -10; arrays[4][128:,0] = 1
        replay = SimpleNamespace(size=128, arrays=arrays, device=torch.device('cpu'))
        sampler = FixedReplaySampler(replay, 'stratified')
        for _ in range(5):
            indices = sampler.indices()
            self.assertLess(indices.max(),128)
            self.assertEqual(int((arrays[2][indices,0]<-9).sum()),32)
        one = FixedReplaySampler(replay,'uniform',11)
        two = FixedReplaySampler(replay,'uniform',11)
        np.testing.assert_array_equal(one.indices(),two.indices())

    def test_only_named_factors_change(self):
        self.assertEqual(schedule('hold',60000),(256,.25))
        self.assertEqual(schedule('hold',120000),(256,.25))
        self.assertEqual(schedule('advance',60000),(1024,.5))
        self.assertEqual(schedule('advance',90000),(None,1.))
        self.assertAlmostEqual(bc_weight('fixed',60000),.14)
        self.assertAlmostEqual(bc_weight('fixed',120000),.14)
        self.assertAlmostEqual(bc_weight('decay',60000),.14)
        self.assertAlmostEqual(bc_weight('decay',120000),.05)

    @unittest.skipUnless(torch.cuda.is_available(),'CUDA required')
    def test_restore_and_critic_only_update_really_freeze_actor(self):
        artifacts=Path('outputs/gfr_sac_demo_paired_gpu_20261126')
        source=Path('outputs/gfr_shared_distill_gpu_20261126')
        learner, identities=restore(artifacts,source)
        saved=torch.load(artifacts/'bc/sac_060000.pt',map_location='cuda',weights_only=True)
        for name in ('actor','q1','q2','target1','target2'):
            for key,value in getattr(learner,name).state_dict().items():
                torch.testing.assert_close(value,saved[name][key],rtol=0,atol=0)
        self.assertEqual(learner.updates,60000)
        self.assertEqual(learner.critic_warmup_updates,1000)
        self.assertEqual(learner.q_opt.state_dict()['param_groups'],saved['critic_optimizer']['param_groups'])
        learner.actor.requires_grad_(False);learner.log_alpha.requires_grad_(False)
        alpha=learner.log_alpha.clone()
        with np.load(artifacts/'bc/online_replay.npz') as data:
            indices=np.flatnonzero(data['rewards'][:60000].ravel() < -9)[:32]
            batch=tuple(torch.as_tensor(data[key][indices],device='cuda') for key in
                        ('observations','actions','rewards','next_observations','terminals'))
        learner.update(batch,(batch[0],batch[1]),0.,critic_only=True)
        self.assertEqual(fingerprint(learner.actor),identities['actor'])
        self.assertNotEqual(fingerprint(learner.q1),identities['q1'])
        torch.testing.assert_close(learner.log_alpha,alpha,rtol=0,atol=0)


if __name__=='__main__':
    unittest.main()
