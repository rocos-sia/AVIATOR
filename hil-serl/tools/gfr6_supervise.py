"""Finish the coverage gate before launching the finite GPU comparison."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from tools.gfr1_remount_evaluate import dump
from tools.gfr4_common import require_memory_limit


def main(root):
    require_memory_limit()
    test_log=root/'implementation_tests.log';attempt=1
    while test_log.exists():
        attempt+=1;test_log=root/f'implementation_tests_{attempt}.log'
    with test_log.open('x') as sink:
        check=subprocess.run([sys.executable,'-m','unittest','tools.test_gfr6_controls','-v'],stdout=sink,stderr=subprocess.STDOUT)
    if check.returncode:raise RuntimeError('Implementation checks failed')
    while True:
        active=subprocess.check_output(['systemctl','--user','show','aviator-gfr6-data.service','-p','ActiveState','--value'],text=True).strip()
        if active not in ('active','activating','deactivating'):break
        dump(root/'pipeline_status.json',dict(phase='waiting_for_data_gate',implementation_checks_passed=True))
        time.sleep(10)
    gate=root/'data/gate.json'
    service_result=subprocess.check_output(['systemctl','--user','show','aviator-gfr6-data.service','-p','Result','--value'],text=True).strip()
    if service_result!='success' or not gate.exists() or not json.loads(gate.read_text())['passed']:
        status=dict(phase='stopped_data_gate',training_started=False,service_result=service_result,
             data_error=json.loads((root/'error.json').read_text()) if (root/'error.json').exists() else None)
        dump(root/'pipeline_status.json',status);dump(root/'status.json',status)
        return
    dump(root/'pipeline_status.json',dict(phase='running_coverage_feature_controls'))
    with (root/'fitting.log').open('x') as sink:
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('gfr6_fit_controls.py')),'--root',str(root)],stdout=sink,stderr=subprocess.STDOUT)
    dump(root/'pipeline_status.json',dict(phase='completed' if result.returncode==0 else 'stopped_fit_error',returncode=result.returncode))
    if result.returncode:raise RuntimeError('Controlled fitting failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    main(p.parse_args().root.resolve())
