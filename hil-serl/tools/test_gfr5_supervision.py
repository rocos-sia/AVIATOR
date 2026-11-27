"""Protect known terminal returns and group-disjoint boundary supervision."""
import unittest
from types import SimpleNamespace
import numpy as np
import torch
from tools.gfr5_pair_data import anchors,split_case,action_grid
from tools.gfr5_validate_supervision import value_update


class DataTests(unittest.TestCase):
    def test_split_is_by_case_not_candidate_action(self):
        self.assertEqual(split_case('train_00/traj_0000'),'fit')
        self.assertEqual(split_case('train_00/traj_0001'),'trajectory_validation')
        self.assertEqual(split_case('train_06/traj_0000'),'installation_test')
        self.assertEqual(split_case('train_07/traj_0001'),'installation_test')

    def test_action_grid_keeps_teacher_and_terminal_anchor(self):
        obs=np.zeros((100,128));act=np.zeros((100,2))
        selected=anchors(obs,act)
        self.assertIn(99,selected)
        self.assertEqual(len(selected),len(set(selected)))
        grid=action_grid([[0,0],[.13,.27]])
        self.assertEqual(len(grid),26)
        self.assertTrue(any(np.array_equal(a,np.array([.13,.27],np.float32)) for a in grid))


@unittest.skipUnless(torch.cuda.is_available(),'CUDA required')
class TerminalTests(unittest.TestCase):
    def test_terminal_supervision_does_not_bootstrap_or_invent_success(self):
        class Network(torch.nn.Module):
            def __init__(self,bias):
                super().__init__();self.value=torch.nn.Parameter(torch.tensor(float(bias),device='cuda'))
            def forward(self,obs,action):return self.value.expand(len(obs),1)
        class Actor:
            def sample(self,obs):return torch.zeros(len(obs),2,device='cuda'),torch.zeros(len(obs),1,device='cuda')
        q1,q2=Network(2),Network(2)
        learner=SimpleNamespace(actor=Actor(),q1=q1,q2=q2,target1=Network(999),target2=Network(999),
            log_alpha=torch.tensor(-6.,device='cuda'),critic_warmup_updates=0,
            q_opt=torch.optim.Adam(list(q1.parameters())+list(q2.parameters()),lr=1e-3))
        obs=torch.zeros(2,128,device='cuda');action=torch.zeros(2,2,device='cuda')
        rewards=torch.tensor([[-10.],[1.]],device='cuda');terminal=torch.ones(2,1,device='cuda')
        batch=(obs,action,rewards,obs,terminal)
        failure=tuple(x[:1] for x in batch);success=tuple(x[1:] for x in batch)
        losses=value_update(learner,batch,[failure,success])
        self.assertAlmostEqual(losses['td_loss'],145.)
        self.assertAlmostEqual(losses['failure_anchor'],288.)
        self.assertAlmostEqual(losses['success_anchor'],2.)
        self.assertEqual(learner.critic_warmup_updates,1)
        bad=(*success[:4],torch.zeros_like(success[4]))
        with self.assertRaises(AssertionError):value_update(learner,batch,[bad])


if __name__=='__main__':unittest.main()
