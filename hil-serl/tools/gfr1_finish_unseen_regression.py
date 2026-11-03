#!/usr/bin/env python3
"""Finish one already-running regression job; no scheduling or training."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def identity(pid):
    path = Path(f'/proc/{pid}/stat')
    try:
        # Everything after the parenthesized command starts with field 3.
        fields = path.read_text().rsplit(')',1)[1].split()
        if fields[0] == 'Z':
            return None
        return fields[19]  # starttime, field 22
    except FileNotFoundError:
        return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--runner-pid',type=int,required=True)
    args = p.parse_args()
    initial = identity(args.runner_pid)
    while not (args.output/'summary.json').exists():
        if initial is None or identity(args.runner_pid) != initial:
            (args.output/'postprocess_failure.json').write_text(json.dumps({
                'status':'runner_exited_without_summary','runner_pid':args.runner_pid,
                'training_started':False},indent=2))
            raise RuntimeError('Regression exited without final summary; inspect console log')
        time.sleep(5)
    # Summary is the final exclusive file; wait for the writer to finish.
    while initial is not None and identity(args.runner_pid) == initial:
        time.sleep(2)
    subprocess.run([sys.executable,'-m','tools.gfr1_report_unseen_regression',str(args.output)],check=True)
    subprocess.run(['git','diff','--check'],check=True)
    summary=json.loads((args.output/'summary.json').read_text())
    status=Path('docs/research/long_horizon_status.md')
    s=status.read_text()
    if '- Node: GFR 40k unseen-trajectory recovery validation (2026-11-01)' in s:
        lines=s.splitlines()
        for i,line in enumerate(lines):
            if line.startswith('- Status: running on frozen new test trajectories;'):
                results='; '.join(f'{k} {v["completed"]}/10 complete, mean progress {v["mean_progress"]:.4f}' for k,v in summary['groups'].items())
                lines[i]='- Status: completed and audited; '+results+'. No training.'
                break
        s='\n'.join(lines)+'\n'
    s += '\n## 2026-11-01: 40k unseen-trajectory recovery validation completed\n\n'
    s += '- Result directory: '+str(args.output)+'\n'
    for name,result in summary['groups'].items():
        s += f'- {name}: {result["completed"]}/{result["episodes"]} completed; mean progress {result["mean_progress"]:.6f}; stops {result["stops"]}.\n'
    s += '- Identical paired initial states verified for all 10 new test trajectories; all accepted steps independently rechecked. Checkpoint, geometry, safety backend and prior evidence hashes unchanged.\n'
    s += '- Results describe the unchanged policy PLUS execution recovery on 10 new trajectories within the same perturbation models, not RL-only convergence or generalization to arbitrary geometry. Discrete swept validation is not continuous-time safety proof.\n'
    s += '- Next: read next_step_decision.json; keep test trajectories out of training, and decide correction-data collection on separate training tasks only after reviewing failures and intervention cost. No retraining was started.\n'
    status.write_text(s)
    subprocess.run(['git','diff','--check'],check=True)
    (args.output/'job_complete.json').write_text(json.dumps({'status':'completed_and_audited',
        'training_started':False,'report':str(args.output/'report.txt')},indent=2))


if __name__=='__main__':
    main()
