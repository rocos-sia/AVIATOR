"""Sequential, memory-bounded supervision of the 2x2 retention experiment."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from tools.gfr1_remount_evaluate import dump, sha
from tools.gfr4_common import require_memory_limit


def main(a):
    group = require_memory_limit()
    root=a.root.resolve(); root.mkdir(parents=True,exist_ok=True)
    if (root/'protocol.json').exists():
        raise FileExistsError('A frozen controlled-run protocol already exists')
    critic_status=json.loads((root/'critic/status.json').read_text())
    if not critic_status.get('execution_completed'):
        raise RuntimeError('Finish the frozen-actor diagnostic first')
    old=json.loads((a.artifacts/'bc/config.json').read_text())
    sources={**old['source_sha256'],**{str(p.resolve()):sha(p) for p in Path(__file__).parent.glob('gfr4_*.py')}}
    sources[str(Path(__file__).with_name('test_gfr4_controls.py').resolve())]=sha(Path(__file__).with_name('test_gfr4_controls.py'))
    assert all(sha(p)==h for p,h in sources.items())
    inputs={str(a.artifacts/'bc'/name):sha(a.artifacts/'bc'/name) for name in
            ('sac_060000.pt','actor_060000.pt','online_replay.npz','evaluation_060000/episodes.json')}
    inputs.update(old['demo_hashes'])
    inputs.update({str(a.source/f'dataset/{split}.json'):sha(a.source/f'dataset/{split}.json')
                   for split in ('train','development')})
    # Fixed before the first arm runs; success-dependent stopping would spoil
    # the paired budget. Numerical/audit/instrumentation failures still stop.
    arms=[('decay','advance'),('fixed','hold'),('fixed','advance'),('decay','hold')]
    protocol=dict(arms=[dict(bc_mode=b,curriculum_mode=c) for b,c in arms],
        seed=11, initial_checkpoint='original BC-initialized 60000',
        additional_environment_steps_per_arm=60000, final_step=120000,
        evaluations=list(range(70000,120001,10000)),
        controls='same initial networks/optimizers/RNG/replay, reward, sampling, learning rate, acceptance rules, and evaluation cases',
        resumed_environment_state='new common resets; no claim of exact continuation of the old process',
        actor_bc_fixed=.14,actor_bc_decay='0.14 at60k linearly to0.05 at120k',
        hold_curriculum=dict(segment_steps=256,original_start_probability=.25),
        advance_curriculum='original 60-90k:1024/.5; >=90k:full original starts',
        online_replay='unchanged uniform; critic experiment weights never imported',
        training_workers=2,evaluation_workers=1,memory_max_bytes=int((group/'memory.max').read_text()),
        stop_on='nonfinite training, accepted-motion audit violation, invalid instrumentation, resource failure',
        retention_alert='log lost original successful cases and lost fixed-state actions every10k; do not change paired budget',
        heldout_used=False,latency_claim=False,tracking_gate=False,five_ms_gate=False,
        sources=sources,inputs=inputs)
    dump(root/'protocol.json',protocol)
    child=None; current='initializing'; started=time.perf_counter(); interrupted=False
    def stop(signum,frame):
        nonlocal interrupted
        interrupted=True
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    def command(name,args,status=None):
        nonlocal child,current
        current=name
        with (root/f'{name}.log').open('x') as stream:
            child=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            dump(root/'processes.json',dict(supervisor=os.getpid(),child=child.pid,phase=name))
            while child.poll() is None:
                detail=json.loads(status.read_text()) if status and status.exists() else None
                memory={key:(group/key).read_text().strip() for key in ('memory.current','memory.peak','memory.events') if (group/key).exists()}
                dump(root/'status.json',dict(phase=name,detail=detail,elapsed_s=time.perf_counter()-started,
                     memory=memory,queued=[f'{b}_{c}' for b,c in arms if not (root/f'{b}_{c}/status.json').exists()]))
                time.sleep(5)
            if child.returncode or interrupted:
                raise RuntimeError(f'{name} exited {child.returncode}; see {name}.log')
    try:
        command('implementation_tests',[sys.executable,'-m','unittest','tools.test_gfr3_sac','tools.test_gfr4_controls','-v'])
        prepared=root/'retention_states/manifest.json'
        if prepared.exists():
            prior=json.loads(prepared.read_text())
            assert prior['passed'] and len(prior['cases'])==9
            assert sha(root/'retention_states/states.npz')==prior['state_sha256']
            for entry in prior['cases']:
                log=a.artifacts/entry['origin_arm']/'evaluation_120000'/f"{entry['case'].replace('/', '__')}.jsonl"
                assert sha(log)==entry['log_sha256']
        else:
            command('prepare_retention',[sys.executable,str(Path(__file__).with_name('gfr4_retention_probes.py')),
                '--artifacts',str(a.artifacts),'--source',str(a.source),'--diagnosis',str(a.diagnosis),
                '--output',str(root/'retention_states')])
        # Baseline uses the same nine states as every subsequent evaluation.
        code='from pathlib import Path; from tools.gfr4_retention_probes import evaluate_probes; evaluate_probes(Path(__import__("sys").argv[1]),Path(__import__("sys").argv[2]),Path(__import__("sys").argv[3]))'
        if (root/'baseline_retention.json').exists():
            prior=json.loads((root/'baseline_retention.json').read_text())
            assert prior['checkpoint_sha256']==sha(a.artifacts/'bc/actor_060000.pt') and prior['cases']==9
        else:
            command('baseline_retention',[sys.executable,'-c',code,str(a.artifacts/'bc/actor_060000.pt'),
                                        str(root/'retention_states'),str(root/'baseline_retention.json')])
        for mode in ('hold','advance'):
            command('smoke_'+mode,[sys.executable,str(Path(__file__).with_name('gfr4_train_controlled.py')),
                '--artifacts',str(a.artifacts),'--source',str(a.source),'--root',str(root),
                '--output',str(root/('smoke_'+mode)),'--bc-mode','fixed','--curriculum-mode',mode,
                '--end-step','60032','--smoke'],root/('smoke_'+mode)/'status.json')
        for bc,course in arms:
            name=f'{bc}_{course}'
            command(name,[sys.executable,str(Path(__file__).with_name('gfr4_train_controlled.py')),
                '--artifacts',str(a.artifacts),'--source',str(a.source),'--root',str(root),
                '--output',str(root/name),'--bc-mode',bc,'--curriculum-mode',course],root/name/'status.json')
        configs=[json.loads((root/f'{b}_{c}/config.json').read_text()) for b,c in arms]
        assert all(conf['initial_networks']==configs[0]['initial_networks'] for conf in configs)
        assert all(sha(p)==h for p,h in {**sources,**inputs}.items())
        curves={f'{b}_{c}':json.loads((root/f'{b}_{c}/evaluations.json').read_text()) for b,c in arms}
        result=dict(execution_completed=True,matched_initial_networks=True,inputs_unchanged=True,
                    curves=curves,elapsed_s=time.perf_counter()-started,single_seed=True,
                    heldout_evaluated=False,relative_latency_evaluated=False)
        dump(root/'results.json',result)
        dump(root/'status.json',dict(phase='completed',execution_completed=True,results=str(root/'results.json'),
                                     heldout_evaluated=False,scientific_qualification='see full trajectory curves'))
    except BaseException as error:
        dump(root/'status.json',dict(phase='stopped_on_error',failed_stage=current,error=repr(error),execution_completed=False))
        raise
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('artifacts','source','diagnosis','root'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    for name in ('artifacts','source','diagnosis','root'):
        setattr(args,name,getattr(args,name).resolve())
    main(args)
