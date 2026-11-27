import unittest
import numpy as np
import torch
from tools.gfr6_coverage_data import local_features,select_seven
from tools.gfr6_fit_controls import DiagnosticModel,Sampler


class Controls(unittest.TestCase):
    def test_local_prediction_uses_correct_bound_direction(self):
        h=np.zeros(14);basis=np.zeros((14,2));basis[0,0]=1;basis[1,1]=1
        margin=np.full(14,.001)
        forward=local_features(h,basis,margin,margin,[1,0],.005,1.5,1.5)
        backward=local_features(h,basis,margin,margin,[-1,0],.005,1.5,1.5)
        self.assertEqual(forward.shape,(56,))
        self.assertAlmostEqual(float(forward[14]),1.)
        self.assertLess(forward[42],0)
        self.assertGreater(forward[28],0)
        self.assertLess(backward[28],0)
        self.assertGreater(backward[42],0)

    def test_equal_capacity_and_common_initialization(self):
        torch.manual_seed(11);a=DiagnosticModel(np.zeros(128),np.ones(128))
        torch.manual_seed(11);b=DiagnosticModel(np.zeros(128),np.ones(128))
        for x,y in zip(a.parameters(),b.parameters()):torch.testing.assert_close(x,y,rtol=0,atol=0)
        obs=torch.zeros(4,128);action=torch.zeros(4,2);empty=torch.zeros(4,56)
        torch.testing.assert_close(a(obs,action,empty),b(obs,action,empty),rtol=0,atol=0)
        self.assertEqual(a(obs,action,empty).shape,(4,2))

    def test_seven_distinct_anchors_even_when_extrema_coincide(self):
        obs=np.zeros((30,128));obs[:,3]=np.linspace(-1,1,30)
        selected=select_seven(obs,np.zeros((30,2)))
        self.assertEqual(len(selected),7);self.assertEqual(len(set(selected)),7)
        self.assertIn(0,selected);self.assertIn(29,selected)

    def test_sampler_never_labels_nonterminal_success_as_goal(self):
        arrays={'feasible':np.array([0,1,1,0,1,1]),'terminals':np.array([1,1,0,1,1,0])}
        groups=[dict(split='narrow_fit',start=0,end=3,mixed=True),dict(split='broad_fit',start=3,end=6,mixed=True)]
        sampler=Sampler(arrays,groups,'narrow_fit',11)
        batch=sampler.classification();terminal=sampler.terminals()
        self.assertEqual(len(batch),256);self.assertLess(batch.max(),3)
        self.assertEqual(len(terminal),64)
        self.assertTrue((arrays['terminals'][terminal]==1).all())
        self.assertTrue((arrays['feasible'][terminal[:32]]==0).all())
        self.assertTrue((arrays['feasible'][terminal[32:]]==1).all())

    @unittest.skipUnless(torch.cuda.is_available(),'CUDA required')
    def test_two_head_update_executes_on_gpu(self):
        model=DiagnosticModel(np.zeros(128),np.ones(128)).cuda()
        optimizer=torch.optim.Adam(model.parameters(),lr=3e-4)
        obs=torch.randn(8,128,device='cuda');action=torch.zeros(8,2,device='cuda');local=torch.randn(8,56,device='cuda')
        out=model(obs,action,local)
        loss=torch.nn.functional.binary_cross_entropy_with_logits(out[:,0],torch.zeros(8,device='cuda'))
        loss=loss+(out[:,1]-torch.tensor([-1.,.1]*4,device='cuda')).square().mean()
        optimizer.zero_grad(set_to_none=True);loss.backward();optimizer.step()
        self.assertTrue(bool(torch.isfinite(loss)))
        self.assertEqual(next(model.parameters()).device.type,'cuda')


if __name__=='__main__':unittest.main()
