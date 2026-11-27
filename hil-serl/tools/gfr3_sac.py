"""Twin-critic CUDA SAC with permanent balanced demo replay and mean-action BC."""
import copy
import math
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from tools.gfr2_train_shared_bc import Actor, load_actor, cuda_device

GAMMA=.999


class GaussianActor(Actor):
    def __init__(self,mean,scale):
        super().__init__(mean,scale)
        self.log_std=nn.Linear(256,2)
        nn.init.zeros_(self.log_std.weight)
        nn.init.constant_(self.log_std.bias,-3.)

    def sample(self,obs,deterministic=False):
        hidden=self.net[:4]((obs-self.mean)/self.scale)
        mu=self.net[4](hidden)
        if deterministic:return torch.tanh(mu),None
        log_std=self.log_std(hidden).clamp(-5.,1.)
        dist=torch.distributions.Normal(mu,log_std.exp())
        pre=dist.rsample()
        action=torch.tanh(pre)
        log_prob=(dist.log_prob(pre)-2.*(math.log(2.)-pre-F.softplus(-2.*pre))).sum(-1,keepdim=True)
        return action,log_prob


class Critic(nn.Module):
    def __init__(self,mean,scale):
        super().__init__()
        self.register_buffer("mean",torch.as_tensor(mean,dtype=torch.float32))
        self.register_buffer("scale",torch.as_tensor(scale,dtype=torch.float32))
        self.net=nn.Sequential(nn.Linear(130,256),nn.LayerNorm(256),nn.ReLU(),
            nn.Linear(256,256),nn.LayerNorm(256),nn.ReLU(),nn.Linear(256,1))

    def forward(self,obs,action):
        return self.net(torch.cat(((obs-self.mean)/self.scale,action),dim=-1))


class SAC:
    def __init__(self,reference_checkpoint,initialization,seed=11):
        self.device=cuda_device()
        bc=load_actor(reference_checkpoint,self.device)
        mean=bc.mean.detach().cpu().numpy();scale=bc.scale.detach().cpu().numpy()
        torch.manual_seed(seed)
        self.actor=GaussianActor(mean,scale).to(self.device)
        # Isolate critic RNG from actor initialization and BC transfer.
        torch.manual_seed(seed+1000)
        self.q1=Critic(mean,scale).to(self.device);self.q2=Critic(mean,scale).to(self.device)
        self.target1=copy.deepcopy(self.q1).requires_grad_(False)
        self.target2=copy.deepcopy(self.q2).requires_grad_(False)
        self.transfer_max_error=None
        if initialization=="bc":
            missing,unexpected=self.actor.load_state_dict(bc.state_dict(),strict=False)
            assert missing==["log_std.weight","log_std.bias"] and not unexpected
            probe=torch.randn(64,128,device=self.device)
            with torch.no_grad():
                self.transfer_max_error=float((self.actor.sample(probe,True)[0]-bc(probe)).abs().max())
            if self.transfer_max_error>1e-6:raise AssertionError("BC mean-action transfer mismatch")
        elif initialization!="random":raise ValueError(initialization)
        self.actor_opt=torch.optim.Adam(self.actor.parameters(),lr=1e-4)
        self.q_opt=torch.optim.Adam(list(self.q1.parameters())+list(self.q2.parameters()),lr=3e-4)
        self.log_alpha=torch.tensor(math.log(.01),device=self.device,requires_grad=True)
        self.alpha_opt=torch.optim.Adam([self.log_alpha],lr=1e-4)
        self.updates=0;self.critic_warmup_updates=0
        # Identical stochastic sampling stream after optional transfer checks.
        torch.manual_seed(seed+2000)

    def predict(self,obs,deterministic=False):
        with torch.no_grad():
            return self.actor.sample(torch.as_tensor(obs,dtype=torch.float32,device=self.device),deterministic)[0].cpu().numpy()

    def update(self,batch,demo,bc_weight,critic_only=False):
        obs,action,reward,nxt,terminal=batch
        alpha=self.log_alpha.exp().detach()
        with torch.no_grad():
            future,log_prob=self.actor.sample(nxt)
            target=reward+GAMMA*(1.-terminal)*(torch.minimum(self.target1(nxt,future),self.target2(nxt,future))-alpha*log_prob)
        q1=self.q1(obs,action);q2=self.q2(obs,action)
        qloss=F.mse_loss(q1,target)+F.mse_loss(q2,target)
        if not torch.isfinite(qloss):raise FloatingPointError("Nonfinite critic loss")
        self.q_opt.zero_grad(set_to_none=True);qloss.backward()
        nn.utils.clip_grad_norm_(list(self.q1.parameters())+list(self.q2.parameters()),10.)
        self.q_opt.step()
        metrics=dict(critic_loss=float(qloss.detach()),target_q_mean=float(target.mean()),q_mean=float(((q1.mean()+q2.mean())/2.).detach()))
        if not critic_only:
            for net in (self.q1,self.q2):net.requires_grad_(False)
            proposed,log_prob=self.actor.sample(obs)
            actor_loss=(alpha*log_prob-torch.minimum(self.q1(obs,proposed),self.q2(obs,proposed))).mean()
            demo_obs,demo_actions=demo
            predicted,_=self.actor.sample(demo_obs,True)
            bc_loss=F.mse_loss(predicted,demo_actions)
            loss=actor_loss+bc_weight*bc_loss
            if not torch.isfinite(loss):raise FloatingPointError("Nonfinite actor loss")
            self.actor_opt.zero_grad(set_to_none=True);loss.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(),1.)
            self.actor_opt.step()
            for net in (self.q1,self.q2):net.requires_grad_(True)
            alpha_loss=-(self.log_alpha*(log_prob.detach()-4.)).mean()
            self.alpha_opt.zero_grad(set_to_none=True);alpha_loss.backward();self.alpha_opt.step()
            with torch.no_grad():self.log_alpha.clamp_(math.log(1e-5),math.log(.1))
            metrics.update(actor_loss=float(actor_loss.detach()),bc_loss=float(bc_loss.detach()),
                           alpha=float(self.log_alpha.exp().detach()),bc_weight=bc_weight)
            self.updates+=1
        else:self.critic_warmup_updates+=1
        with torch.no_grad():
            for live,target_net in ((self.q1,self.target1),(self.q2,self.target2)):
                for p,t in zip(live.parameters(),target_net.parameters()):t.lerp_(p,.005)
        return metrics

    def export(self,path):
        # Existing standalone evaluator consumes the deterministic mean actor.
        weights={k:v for k,v in self.actor.state_dict().items() if not k.startswith("log_std.")}
        torch.save(dict(state_dict=weights),path)

    def save(self,path,environment_steps):
        torch.save(dict(actor=self.actor.state_dict(),q1=self.q1.state_dict(),q2=self.q2.state_dict(),
            target1=self.target1.state_dict(),target2=self.target2.state_dict(),log_alpha=self.log_alpha.detach(),
            actor_optimizer=self.actor_opt.state_dict(),critic_optimizer=self.q_opt.state_dict(),
            alpha_optimizer=self.alpha_opt.state_dict(),environment_steps=environment_steps,
            gradient_updates=self.updates,critic_warmup_updates=self.critic_warmup_updates,
            torch_rng=torch.get_rng_state(),cuda_rng=torch.cuda.get_rng_state()),path)


class DemoReplay:
    def __init__(self,files,device):
        keys=("observations","actions","rewards","next_observations","terminals")
        parts={k:[] for k in keys};weights=[];offset=0
        self.case_ranges=[]
        for file in files:
            with np.load(file) as data:
                n=len(data["actions"]);critical=data["critical"].astype(bool)
                weight=np.full(n,.5/n)
                if critical.any():weight[critical]+=.5/critical.sum()
                else:weight+=.5/n
                weights.extend(weight)
                for k in keys:parts[k].append(data[k])
                self.case_ranges.append((offset,offset+n));offset+=n
        self.arrays=tuple(torch.as_tensor(np.concatenate(parts[k]),dtype=torch.float32,device=device)
                          .reshape(-1,1 if k in ("rewards","terminals") else (2 if k=="actions" else 128)) for k in keys)
        self.weights=torch.as_tensor(weights,dtype=torch.float32,device=device)
        self.size=offset

    def sample(self,count):
        indices=torch.multinomial(self.weights,count,replacement=True)
        return tuple(x[indices] for x in self.arrays)


class OnlineReplay:
    def __init__(self,capacity,device,seed=11):
        self.device=device;self.rng=np.random.default_rng(seed);self.capacity=capacity;self.position=0;self.size=0
        self.arrays=[np.empty((capacity,n),np.float32) for n in (128,2,1,128,1)]

    def add(self,obs,actions,rewards,nxt,terminal):
        count=len(obs);indices=(np.arange(count)+self.position)%self.capacity
        for array,values in zip(self.arrays,(obs,actions,rewards,nxt,terminal)):
            array[indices]=np.asarray(values).reshape(count,-1)
        self.position=(self.position+count)%self.capacity;self.size=min(self.size+count,self.capacity)

    def sample(self,count):
        indices=self.rng.integers(self.size,size=count)
        return tuple(torch.as_tensor(x[indices],device=self.device) for x in self.arrays)
