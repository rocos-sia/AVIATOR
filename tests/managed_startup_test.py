"""Mock Rokae idle feedback: no SDK/hardware, startup enable is emulated; no SDK or physical motion."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import yaml
import zmq
from robot_state_machine_process_test import port


def main():
    bus, core, root = sys.argv[1:]
    root = Path(root)
    directory = Path(tempfile.mkdtemp(prefix="aviator-idle-startup-"))
    print(f"Idle startup logs: {directory}", flush=True)
    config = yaml.safe_load((root / 'config/system.yaml').read_text())
    # Only Core's kinematic initialization uses this file; no hardware backend is instantiated.
    config['robot'] = str(root / 'config/robot.yaml')
    config['core_hand'] = {'enabled': False}
    endpoints = [f'tcp://127.0.0.1:{port()}' for _ in range(3)]
    assert len(set(endpoints)) == 3
    config['bus'] = dict(publish=endpoints[0], subscribe=endpoints[1])
    config['manipulator_service'] = endpoints[2]
    path = directory / 'system.yaml'
    path.write_text(yaml.safe_dump(config))
    context = zmq.Context()
    stop = threading.Event()
    ready = threading.Event()
    stale = threading.Event()
    failures = []
    operations = []
    posture = json.loads((root / 'config/posture.json').read_text())
    import math
    q = [math.radians(v) for v in posture['left_home_deg'] + posture['right_home_deg']]
    q[0] += .05  # Start away from home: startup must hold here rather than home automatically.
    session = str(uuid.uuid4())
    clock = socket.gethostname() + '-' + Path('/proc/sys/kernel/random/boot_id').read_text().strip()

    def device():
        rep = context.socket(zmq.REP)
        pub = context.socket(zmq.PUB)
        rep.bind(endpoints[2]); pub.connect(endpoints[0]); ready.set()
        sequence = 0
        enabled = False
        try:
            while not stop.is_set():
                if rep.poll(5):
                    req = rep.recv_json()
                    operation = req['operation']
                    operations.append(operation)
                    assert operation in ('describe', 'authorize', 'enable', 'stop'), f'Unexpected device action: {operation}'
                    if operation == 'enable': enabled = True
                    result = dict(config_id=config['config_id'], server_session=session, backend='rokae',
                                  q=q, target=q, speed=[1.] * 14,
                                  control_epoch=str(uuid.uuid4()))
                    rep.send_json(dict(msg_type='ServiceReply', request_id=req['request_id'],
                                       client_session_id=req['client_session_id'], status='COMPLETED', result=result))
                now = time.monotonic_ns() // 1000
                sequence += 1
                msg = dict(msg_type='ArmState', version='1.0', sequence=sequence,
                           timestamp=time.time_ns() // 1000, sample_mono_us=(now - (1000000 if stale.is_set() else 0)) if enabled else 0, valid=enabled,
                           clock_id=clock, publisher_id='manipulator', session_id=session,
                           config_id=config['config_id'], status_mono_us=now - (1000000 if stale.is_set() else 0),
                           software_lock=False, wheel_reference=dict(angle=0., displacement=0.),
                           execution=dict(trajectory_id=0, tick=0, target=q, fault=False, error='', stopping=False),
                           arms={side: dict(joint_position=q[7 * i:7 * (i + 1)], joint_velocity=[0.] * 7 if enabled else None, enabled=enabled)
                                 for i, side in enumerate(('left', 'right'))})
                pub.send_multipart([b'arm.state', json.dumps(msg).encode()])
                stop.wait(.005)
        except BaseException as exc:
            failures.append(exc)
        finally:
            rep.close(0); pub.close(0)

    sub = context.socket(zmq.SUB)
    sub.subscribe('flight.state'); sub.connect(endpoints[1])
    children, logs = [], []
    thread = threading.Thread(target=device)

    def start(name, args):
        log = (directory / (name + '.log')).open('w'); logs.append(log)
        child = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=log, stderr=log,
                                 text=True, env={**os.environ, 'LD_BIND_NOW': '1'})
        children.append(child)
        return child

    def state(expected, require_valid=False):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert not failures, failures
            if sub.poll(100):
                message = json.loads(sub.recv_multipart()[1])
                if message['system']['state'] == expected:
                    if require_valid:
                        assert message['valid'], 'idle Core status cannot be discovered by Gateway'
                        assert message['freshness']['arm']['valid'], 'READY requires feedback after enable'
                    return message['system']
            assert process.poll() is None, 'Core exited'
        raise AssertionError(f'Missing state {expected}; inspect logs')

    try:
        start('bus', [bus, '--input', endpoints[0], '--output', endpoints[1],
                      '--lock-file', str(directory / 'bus.lock')])
        thread.start(); assert ready.wait(2)
        process = start('core', [core, '--config', str(path), '--console'])
        assert state('READY', require_valid=True)['ready']
        assert operations == ['describe', 'authorize', 'enable'], operations
        # After READY, stale RT feedback must revoke readiness even if status messages arrive.
        stale.set()
        state('SAFE')
        process.stdin.write('GRASP_WHEEL\n'); process.stdin.flush()
        time.sleep(.1)
        assert operations.count('enable') == 1, operations
        process.stdin.write('quit\n'); process.stdin.flush()
        assert process.wait(timeout=5) == 0
        assert not failures, failures
        print('PASS: startup enables then waits in READY away from home; stale feedback trips SAFE (mock device only)', flush=True)
    finally:
        stop.set()
        if thread.ident is not None: thread.join()
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
                try: child.wait(timeout=5)
                except subprocess.TimeoutExpired: child.kill(); child.wait()
        for log in logs: log.close()
        sub.close(0); context.term()


if __name__ == '__main__':
    main()
