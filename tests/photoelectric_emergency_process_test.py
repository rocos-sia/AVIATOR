"""Real Core + device server + bus; fake arm IO, no physical hardware."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import uuid

import yaml
import zmq

bus, driver, core, root = sys.argv[1:]
root = Path(root)


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


for scenario in ('zero_burst', 'invalid', 'timeout', 'startup_zero', 'initializing'):
    with tempfile.TemporaryDirectory(prefix='aviator-optical-emergency-') as folder:
        directory = Path(folder)
        cfg = yaml.safe_load((root / 'config/system.yaml').read_text())
        endpoints = [f'tcp://127.0.0.1:{port()}' for _ in range(3)]
        cfg.update(robot=str(root / 'config/robot.yaml'),
                   bus=dict(publish=endpoints[0], subscribe=endpoints[1]),
                   manipulator_service=endpoints[2], core_hand={'enabled': False})
        cfg['photoelectric_emergency'] = dict(enabled=True, publisher_id='aviator_photoelectric',
                                             channel='X1', timeout_ms=150)
        path = directory / 'system.yaml'
        path.write_text(yaml.safe_dump(cfg))
        context = zmq.Context()
        pub = context.socket(zmq.PUB); pub.connect(endpoints[0])
        sub = context.socket(zmq.SUB); sub.subscribe('flight.state'); sub.connect(endpoints[1])
        clock = socket.gethostname() + '-' + Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        session = str(uuid.uuid4())
        children, logs = [], []
        sequence, value, valid = 0, 1, True
        transmitting = False
        next_send = 0

        def send(v):
            global sequence
            sequence += 1
            msg = dict(msg_type='PhotoelectricState', version='1.0', sequence=sequence,
                       timestamp=time.time_ns() // 1000, sample_mono_us=time.monotonic_ns() // 1000,
                       clock_id=clock, publisher_id='aviator_photoelectric', session_id=session,
                       valid=valid, value=v, channel='X1', status='ok' if valid else 'timeout')
            pub.send_multipart([b'photoelectric.state', json.dumps(msg).encode()])

        def wait(predicate, timeout=10):
            global next_send
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                now = time.monotonic()
                if transmitting and now >= next_send:
                    send(value); next_send = now + .03
                if sub.poll(5):
                    msg = json.loads(sub.recv_multipart()[1])
                    if predicate(msg):
                        return msg
                assert all(p.poll() is None for p in children), 'child process exited'
            raise AssertionError('Missing expected state')

        def start(name, args):
            log = (directory / (name + '.log')).open('w'); logs.append(log)
            child = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=log, stderr=log,
                                     text=True, env={**os.environ, 'LD_BIND_NOW': '1'})
            children.append(child)
            return child

        try:
            start('bus', [bus, '--input', endpoints[0], '--output', endpoints[1],
                          '--lock-file', str(directory / 'bus.lock')])
            start('server', [driver, 'server', str(path),
                             'emergency_enable' if scenario == 'initializing' else 'success'])
            process = start('core', [core, '--config', str(path), '--console'])
            wait(lambda m: m.get('system', {}).get('state') == 'INIT')
            time.sleep(.2)
            assert 'ENABLE ' not in (directory / 'server.log').read_text(), 'enabled before optical evidence'
            transmitting = True
            if scenario == 'startup_zero':
                value = 0
            else:
                if scenario == 'initializing':
                    wait(lambda m: 'ENABLE_PENDING' in (directory / 'server.log').read_text())
                else:
                    wait(lambda m: m.get('system', {}).get('state') == 'READY')
                if scenario in ('zero_burst', 'initializing'):
                    send(0); send(1)  # A single queued zero cannot be overwritten by a later one.
                elif scenario == 'invalid':
                    valid = False
                else:
                    transmitting = False
            began = time.monotonic()
            result = wait(lambda m: m.get('system', {}).get('state') == 'EMERGENCY_STOP' and
                          m['system'].get('emergency_power_off', {}).get('confirmed'), timeout=3)
            elapsed = time.monotonic() - began
            assert all(not result['arms'][side]['enabled'] for side in ('left', 'right')), result
            if scenario == 'startup_zero':
                assert 'ENABLE ' not in (directory / 'server.log').read_text()
            transmitting, valid, value = True, True, 1
            process.stdin.write('RESET_ERROR\nENTER_STANDBY\nSTART_CONTROL\n'); process.stdin.flush()
            end = time.monotonic() + .3
            while time.monotonic() < end:
                result = wait(lambda m: True)
                assert result['system']['state'] == 'EMERGENCY_STOP', result
                assert all(not result['arms'][side]['enabled'] for side in ('left', 'right')), result
            print(f'PASS {scenario}: FSM latched, dual arms disabled, observed confirmation {elapsed:.3f}s', flush=True)
        except BaseException:
            for name in ('core', 'server', 'bus'):
                print(f'{name}:\n{(directory / (name + ".log")).read_text()}', flush=True)
            raise
        finally:
            for child in reversed(children):
                child.terminate()
                try: child.wait(timeout=5)
                except subprocess.TimeoutExpired: child.kill(); child.wait()
            for log in logs: log.close()
            sub.close(0); pub.close(0); context.term()
