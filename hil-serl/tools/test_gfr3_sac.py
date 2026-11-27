"""Guard the learning semantics that can silently invalidate a SAC comparison."""
import tempfile
import unittest
from pathlib import Path
import numpy as np
import torch
from tools.gfr2_train_shared_bc import Actor,load_actor
from tools.gfr3_sac import SAC,OnlineReplay
from tools.gfr3_train_sac_demo import terminal_transition,fingerprint
from tools.gfr3_curriculum_env import curriculum
from tools.gfr3_demo_transitions import critical_frames


class TransitionTests(unittest.TestCase):
    def test_terminal_successor_is_not_auto_reset(self):
        reset=np.ones((3,128),np.float32)
        info=[dict(terminal_observation=np.full(128,2.),**{"TimeLimit.truncated":False}),
              dict(terminal_observation=np.full(128,3.),**{"TimeLimit.truncated":True}),{}]
        successor,done=terminal_transition(reset,np.array([True,True,False]),info)
        np.testing.assert_array_equal(successor[:,0],[2.,3.,1.])
        np.testing.assert_array_equal(done,[1.,0.,0.])
        np.testing.assert_array_equal(reset,np.ones_like(reset))

    def test_curriculum_reaches_original_starts(self):
        self.assertEqual(curriculum(0),(64,.25))
        self.assertEqual(curriculum(20000),(256,.25))
        self.assertEqual(curriculum(60000),(1024,.5))
        self.assertEqual(curriculum(90000),(None,1.))

    def test_critical_sampling_includes_approach_and_not_next_episode(self):
        obs=np.zeros((100,128));obs[:,30]=1.;obs[:,31]=1.
        act=np.zeros((100,2));act[50]=[1.,0.]
        mask=critical_frames(obs,act)
        self.assertTrue(mask[30:51].all());self.assertFalse(mask[:30].any());self.assertFalse(mask[51:].any())


@unittest.skipUnless(torch.cuda.is_available(),"CUDA required")
class LearningTests(unittest.TestCase):
    def test_transfer_fair_critics_updates_and_export(self):
        torch.manual_seed(42)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"bc.pt"
            bc=Actor(np.zeros(128),np.ones(128))
            torch.save(dict(state_dict=bc.state_dict()),path)
            warm=SAC(path,"bc",11);cold=SAC(path,"random",11)
            self.assertEqual(fingerprint(warm.q1),fingerprint(cold.q1))
            self.assertEqual(fingerprint(warm.q2),fingerprint(cold.q2))
            self.assertNotEqual(fingerprint(warm.actor),fingerprint(cold.actor))
            obs=torch.randn(32,128,device="cuda")
            with torch.no_grad():
                np.testing.assert_allclose(warm.predict(obs.cpu().numpy(),True),bc(obs.cpu()).numpy(),atol=2e-6)
            before=fingerprint(warm.actor)
            batch=(obs,torch.zeros(32,2,device="cuda"),torch.ones(32,1,device="cuda"),
                   obs.clone(),torch.ones(32,1,device="cuda"))
            warm.update(batch,(obs,batch[1]),.2,critic_only=True)
            self.assertEqual(before,fingerprint(warm.actor))
            self.assertEqual(warm.critic_warmup_updates,1)
            metrics=warm.update(batch,(obs,batch[1]),.2)
            self.assertNotEqual(before,fingerprint(warm.actor))
            self.assertEqual(warm.updates,1)
            self.assertTrue(all(np.isfinite(v) for v in metrics.values()))
            export=Path(directory)/"actor.pt";warm.export(export)
            clone=load_actor(export,torch.device("cuda"))
            with torch.no_grad():
                torch.testing.assert_close(clone(obs),warm.actor.sample(obs,True)[0],atol=1e-7,rtol=0.)

    def test_replay_keeps_terminals_and_finite_cuda_samples(self):
        replay=OnlineReplay(16,torch.device("cuda"))
        replay.add(np.zeros((8,128)),np.zeros((8,2)),np.arange(8),np.ones((8,128)),np.ones(8))
        sampled=replay.sample(32)
        self.assertEqual(replay.size,8)
        self.assertTrue(all(str(x.device)=="cuda:0" for x in sampled))
        torch.testing.assert_close(sampled[-1],torch.ones_like(sampled[-1]))
        torch.testing.assert_close(sampled[3],torch.ones_like(sampled[3]))


if __name__=="__main__":unittest.main()
