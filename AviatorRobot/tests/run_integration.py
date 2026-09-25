#!/usr/bin/env python3
"""Run controller and simulator as separate processes on a private test bus."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

simulator, controller, config = map(lambda p: str(Path(p).resolve()), sys.argv[1:])
bus = os.getpid()
with tempfile.TemporaryDirectory(prefix='aviator-test-') as work:
    sim_path = Path(work) / 'simulator.log'
    control_path = Path(work) / 'controller.log'
    with sim_path.open('w') as sim_log, control_path.open('w') as control_log:
        sim = subprocess.Popen([simulator, '--ecat-id', str(bus), '--duration', '180'],
                               cwd=work, stdout=sim_log, stderr=subprocess.STDOUT)
        try:
            for _ in range(200):
                if (Path('/dev/shm') / f'aviator{bus}').exists():
                    break
                if sim.poll() is not None:
                    raise RuntimeError('Simulator exited before initialization')
                time.sleep(.05)
            result = subprocess.run([controller, config, str(bus)], cwd=work,
                                    stdout=control_log, stderr=subprocess.STDOUT, timeout=160)
            control_log.flush()
            output = control_path.read_text()
            if result.returncode or 'INTEGRATION PASS' not in output:
                raise RuntimeError(f'Controller exit code: {result.returncode}\n{output}')
            for line in output.splitlines():
                if 'Verified ' in line or 'Expected rejection:' in line or 'INTEGRATION PASS' in line:
                    print(line)
        except Exception:
            sim_log.flush()
            print(sim_path.read_text())
            control_log.flush()
            print(control_path.read_text())
            raise
        finally:
            sim.terminate()
            try:
                sim.wait(timeout=3)
            except subprocess.TimeoutExpired:
                sim.kill()
                sim.wait()
            for name in [f'aviator{bus}', f'ecm{bus}', f'pd_input{bus}', f'pd_output{bus}',
                         *[f'sem.sync{bus}_{i}' for i in range(10)]]:
                (Path('/dev/shm') / name).unlink(missing_ok=True)
