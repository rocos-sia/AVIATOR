"""Supervise two CUDA SAC arms through a fixed interaction-budget learning curve."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from tools.gfr1_remount_evaluate import dump,sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root",type=Path,required=True);p.add_argument("--source-root",type=Path,required=True)
    p.add_argument("--smoke",action="store_true")
    a=p.parse_args();root=a.root.resolve();source=a.source_root.resolve()
    group=next(line.split(":",2)[2] for line in Path("/proc/self/cgroup").read_text().splitlines() if line.startswith("0::"))
    limit_file=Path("/sys/fs/cgroup")/group.lstrip("/")/"memory.max"
    limit=limit_file.read_text().strip()
    if limit=="max" or int(limit)>12*1024**3:
        raise RuntimeError("Run under a user systemd unit with MemoryMax <= 12G; unbounded memory is forbidden")
    if not json.loads((root/"demonstrations/gate.json").read_text())["passed"]:
        raise RuntimeError("Demo export gate failed")
    steps=32 if a.smoke else 120000
    protocol=dict(arms=["bc","random"],seed=11,environment_steps_per_arm=steps,
        comparisons_at=[0,steps] if a.smoke else [0,20000,60000,120000],
        device="cuda:0",execution_order="bc then random, sequential",workers_per_arm=2,
        memory_max_bytes=int(limit),memory_cgroup=str(limit_file),
        shared_demo_pool=str(root/"demonstrations"),source_root=str(source),
        hypothesis="whether more SAC interaction/updates improve standalone full-task success, and whether BC initialization helps",
        scope="exact-calibration local pilot; single seed; no latency or hardware claim",
        stop_on_zero_success=False,heldout_used=False,smoke=a.smoke,
        source_sha256={str(f.resolve()):sha(f) for f in Path(__file__).parent.glob("gfr3_*.py")})
    if (root/"paired_protocol.json").exists():raise FileExistsError("Existing paired run")
    dump(root/"paired_protocol.json",protocol)
    children={};logs={};started=time.perf_counter();interrupted=False
    def stop(signum,frame):
        nonlocal interrupted
        interrupted=True
        for process in children.values():
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        for arm in ("bc","random"):
            cmd=[sys.executable,str(Path(__file__).with_name("gfr3_train_sac_demo.py")),
                 "--root",str(root),"--source-root",str(source),"--initialization",arm,"--steps",str(steps)]
            if a.smoke:cmd.append("--smoke")
            logs[arm]=(root/f"{arm}.log").open("x")
            children[arm]=subprocess.Popen(cmd,stdout=logs[arm],stderr=subprocess.STDOUT,start_new_session=True)
            dump(root/"processes.json",dict(supervisor=os.getpid(),children={k:v.pid for k,v in children.items()}))
            while children[arm].poll() is None:
                statuses={}
                for name in ("bc","random"):
                    file=root/name/"status.json"
                    statuses[name]=json.loads(file.read_text()) if file.exists() else {"phase":"queued"}
                memory={}
                for key in ("memory.current","memory.peak"):
                    file=limit_file.parent/key
                    if file.exists():memory[key]=int(file.read_text())
                dump(root/"status.json",dict(phase="stopping_on_error" if interrupted else "running_paired_sac",
                    elapsed_s=time.perf_counter()-started,arms=statuses,smoke=a.smoke,memory=memory))
                time.sleep(5)
            if children[arm].returncode!=0 or interrupted:
                interrupted=True
                break
        returns={arm:proc.wait() for arm,proc in children.items()}
        summaries={}
        for arm in children:
            gate=root/arm/"gate.json";curves=root/arm/"evaluations.json"
            summaries[arm]=dict(returncode=returns[arm],gate=json.loads(gate.read_text()) if gate.exists() else None,
                learning_curve=json.loads(curves.read_text()) if curves.exists() else [])
        configs=[json.loads((root/arm/"config.json").read_text()) for arm in children if (root/arm/"config.json").exists()]
        matched=len(configs)==2 and all(configs[0][key]==configs[1][key] for key in
            ("initial_critic_sha256","demo_hashes","seed","environment_transitions_budget","batch_size","gamma","curriculum"))
        result=dict(execution_completed=all(code==0 for code in returns.values()) and not interrupted,
            matched_controls=matched,smoke=a.smoke,arms=summaries,elapsed_s=time.perf_counter()-started,
            heldout_evaluated=False,relative_latency_evaluated=False,
            frozen_sources_unchanged=all(sha(f)==h for f,h in protocol["source_sha256"].items()))
        dump(root/"run_summary.json",result)
        qualified=any(row["gate"] and row["gate"].get("passed") for row in summaries.values())
        dump(root/"status.json",dict(phase="smoke_finished" if a.smoke else
            ("training_finished_ready_for_freezing" if qualified else "stopped_at_budget_without_qualification"),
            execution_completed=result["execution_completed"],matched_controls=matched,
            environment_steps_per_arm=steps,qualified_any=qualified,heldout_evaluated=False))
        if not result["execution_completed"] or not matched:raise RuntimeError("Paired training/controls failed")
    finally:
        for process in children.values():
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for stream in logs.values():stream.close()


if __name__=="__main__":main()
