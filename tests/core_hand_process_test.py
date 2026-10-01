"""Real ZMQ + production RemoteLink; fake arm service/hand device, no hardware."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import uuid

import zmq


def port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def run(driver, bus, mode):
    with tempfile.TemporaryDirectory(prefix='core-hand-process-') as directory:
        root = Path(directory)
        endpoints = [f'tcp://127.0.0.1:{port()}' for _ in range(3)]
        assert len(set(endpoints)) == 3
        config = root / 'system.yaml'
        config.write_text(f'''robot: unused.yaml
config_id: test
bus:
  publish: {endpoints[0]}
  subscribe: {endpoints[1]}
manipulator_service: {endpoints[2]}
command_timeout_ms: 100
origin_timeout_ms: 100
core_hand:
  enabled: true
  completion_timeout_ms: 700
  close:
    left: [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]
    right: [0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
''')
        ctx = zmq.Context()
        pub, sub, rep = [ctx.socket(t) for t in (zmq.PUB, zmq.SUB, zmq.REP)]
        pub.connect(endpoints[0])
        sub.connect(endpoints[1])
        sub.subscribe(b'hand.command')
        sub.subscribe(b'flight.state')
        sub.subscribe(b'arm.command')
        rep.bind(endpoints[2])
        server = str(uuid.uuid4())
        node = str(uuid.uuid4())
        clock = socket.gethostname() + '-' + Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        seq = 0
        enabled = locked = False
        ack = dict(publisher_id='', session_id='', sequence=0, sample_mono_us=0)
        targets = dict(left=[1]*6, right=[1]*6)
        actual = dict(left=[0.2]*6, right=[0.2]*6)
        last_change = next_state = locked_at = last_arm = 0
        first_open = close_seen = unlock_seen = aggregate_seen = False
        command_times, operations, captured = [], [], []
        trajectory_id = cursor = reported_cursor = 0
        trajectory_at = 0
        partial_seen = False
        stalled_targets = []
        bus_process = subprocess.Popen([bus, '--input', endpoints[0], '--output', endpoints[1],
                                       '--lock-file', str(root / 'bus.lock')], stdout=subprocess.DEVNULL)
        child = None
        try:
            time.sleep(.2)
            child = subprocess.Popen([driver, str(config), mode], stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True,
                                     env={**os.environ, 'LD_BIND_NOW': '1'})
            until = time.monotonic() + 12
            def header(kind, publisher, session):
                return dict(msg_type=kind, version='1.0', timestamp=time.time_ns()//1000,
                            sample_mono_us=time.monotonic_ns()//1000, sequence=seq,
                            clock_id=clock, publisher_id=publisher, session_id=session, valid=True)
            while child.poll() is None and time.monotonic() < until:
                now = time.monotonic()
                if rep.poll(0):
                    request = rep.recv_json()
                    op = request['operation']
                    operations.append(op)
                    result = {}
                    if op == 'describe':
                        result = dict(config_id='test', server_session=server, backend='rokae',
                                      q=[0]*14, target=[0]*14, speed=[1]*14)
                    elif op == 'authorize': result = dict(control_epoch=str(uuid.uuid4()))
                    elif op == 'enable':
                        assert actual == dict(left=[1]*6, right=[1]*6), 'arm enabled before actual opening'
                        enabled = True
                        result = dict(target=[0]*14)
                    elif op == 'disable': enabled = False
                    elif op == 'lock':
                        assert close_seen, 'software lock before physical command'
                        locked = True
                        locked_at = now
                    elif op == 'unlock':
                        assert actual == dict(left=[1]*6, right=[1]*6), 'software unlock before actual opening'
                        locked = False
                    elif op == 'stop': result = dict(target=[0]*14)
                    else: raise AssertionError(op)
                    rep.send_json(dict(msg_type='ServiceReply', request_id=request['request_id'],
                                       client_session_id=request['client_session_id'], status='COMPLETED', result=result))
                while sub.poll(0):
                    topic, data = sub.recv_multipart()
                    message = json.loads(data)
                    if topic == b'arm.command':
                        last_arm = now
                        if mode == 'synchronized' and message['total_ticks'] == 1000:
                            if not trajectory_at:
                                trajectory_at = now
                                trajectory_id = message['trajectory_id']
                        continue
                    if topic == b'flight.state':
                        if message['freshness']['hand']['valid']:
                            assert message['hands']['left']['position_source'] == 'angle_act_register'
                            aggregate_seen = True
                        continue
                    captured.append(message)
                    stamp = time.monotonic_ns()//1000
                    assert message['mode'] == 'NORMALIZED_POSITION' and message['valid']
                    # Freshness is guaranteed at publication; transport can deliver a near-
                    # deadline message after expiry (the hand node then rejects it).
                    assert stamp >= message['sample_mono_us']
                    assert 0 <= message['sample_mono_us'] - message['origin']['sample_mono_us'] < 100000, 'invalid publication heartbeat age'
                    assert message['origin']['session_id'] == message['session_id']
                    assert message['sequence'] > ack['sequence']
                    command_times.append(now)
                    incoming = {side: message['hands'][side]['drive_position_normalized'] for side in targets}
                    if not first_open or incoming != targets:
                        last_change = now
                    targets = incoming
                    if mode == 'synchronized' and trajectory_at and not unlock_seen:
                        if targets == dict(left=[1]*6, right=[1]*6) and close_seen:
                            unlock_seen = True
                        else:
                            for side, initial, final, start in (('left', 1, .9, 300), ('right', 1, .3, 100)):
                                u = max(0, (reported_cursor-start)/(1000-start))
                                progress = u*u*u*(10+u*(-15+6*u))
                                # The hand target must never advance beyond the reported arm cursor.
                                assert targets[side][0] >= initial + progress*(final-initial) - 1e-9, (reported_cursor, targets)
                            partial_seen |= .9 < targets['left'][0] < 1
                            close_seen |= targets['left'][0] == .9
                            if cursor == 400 and now - trajectory_at > .48:
                                stalled_targets.append(dict(targets))
                    elif targets['left'][0] == .9:
                        assert targets['right'][0] == .3
                        close_seen = True
                    else:
                        assert targets == dict(left=[1]*6, right=[1]*6)
                        if close_seen: unlock_seen = True
                        first_open = True
                    ack = {key: message[key] for key in ack}
                if first_open and now - last_change > .15:
                    actual = {key: list(value) for key, value in targets.items()}
                if now >= next_state:
                    next_state = now + .02
                    seq += 1
                    if trajectory_at:
                        elapsed = now - trajectory_at
                        # Freeze the arm at 400 ms for 300 ms while hand traffic stays healthy.
                        cursor = min(1000, int(1000 * (elapsed if elapsed < .4 else .4 if elapsed < .7 else elapsed-.3)))
                        reported_cursor = cursor
                    m = header('ArmState', 'manipulator', server)
                    m.update(config_id='test', arms={side: dict(joint_position=[0]*7, joint_velocity=[0]*7,
                             enabled=enabled) for side in ('left','right')},
                             execution=dict(trajectory_id=trajectory_id, tick=cursor, target=[0]*14, fault=False, error='', stopping=False),
                             software_lock=locked, wheel_reference=dict(angle=0, displacement=0))
                    if mode == 'arm_failure' and locked_at and now - locked_at > .1:
                        del m['execution']  # A broken arm-state body terminates the arm IO loop.
                    if mode != 'no_arm':
                        pub.send_multipart([b'arm.state', json.dumps(m).encode()])
                    # Synthetic manipulator hand.state must never supply the physical hand feedback.
                    synthetic = header('HandState', 'manipulator', server)
                    synthetic['hands'] = dict(left=dict(position_source='synthetic'), right={})
                    pub.send_multipart([b'hand.state', json.dumps(synthetic).encode()])
                    if mode != 'missing' and not (mode == 'dropped' and locked_at and now - locked_at > .1):
                        m = header('HandState', 'inspire_hand', node)
                        owned = dict(ack)
                        if mode == 'foreign': owned.update(publisher_id='manual', session_id=str(uuid.uuid4()))
                        m.update(feedback_only=mode == 'readonly', command_valid=first_open,
                                 accepted_command=owned,
                                 hands={side: dict(valid=True, sample_mono_us=m['sample_mono_us'],
                                                  drive_position_normalized=actual[side],
                                                  position_source='angle_act_register') for side in targets})
                        pub.send_multipart([b'hand.state', json.dumps(m).encode()])
                time.sleep(.002)
            if child.poll() is None:
                child.kill()
                raise AssertionError('driver timeout')
            output = child.communicate()[0]
            if mode == 'no_arm':
                assert child.returncode != 0 and 'No arm.state via bus' in output, output
                assert not command_times, 'constructor failure published hand targets'
                print('no_arm: constructor failure cleaned up both IO threads', flush=True)
                return
            assert child.returncode == 0, output
            if mode == 'synchronized':
                assert partial_seen and close_seen and unlock_seen and aggregate_seen
                assert cursor == 1000 and len(stalled_targets) >= 5
                assert all(v == stalled_targets[0] for v in stalled_targets), 'hand advanced while arm cursor was frozen'
                assert operations == ['describe','authorize','enable','lock','unlock','disable'], operations
            elif mode in ('normal', 'isolation'):
                assert first_open and close_seen and unlock_seen and aggregate_seen
                assert operations == ['describe','authorize','enable','lock','stop','unlock','disable'], operations
                max_gap = max(b-a for a,b in zip(command_times,command_times[1:]))
                assert len(command_times) >= 40 and max_gap < .1
                print(f'{mode}: maximum observed hand command gap {max_gap * 1000:.1f} ms', flush=True)
                assert len({m['control_epoch'] for m in captured}) == 1
            elif mode in ('dropped', 'revoke', 'heartbeat', 'arm_failure'):
                assert close_seen and not unlock_seen
                assert time.monotonic() - command_times[-1] > .25, 'hand target kept alive after fault/revocation'
                assert time.monotonic() - last_arm > .25, 'arm windows continued after fault/revocation'
                if mode == 'revoke':
                    assert operations.count('stop') == 1, 'idle revocation did not request device-local stop'
            else:
                assert 'enable' not in operations and 'lock' not in operations, operations
            print(mode + ': ' + output.strip(), flush=True)
        finally:
            if child and child.poll() is None: child.kill(); child.wait()
            bus_process.terminate(); bus_process.wait(timeout=3)
            for sock in (pub,sub,rep): sock.close(0)
            ctx.term()


if __name__ == '__main__':
    for scenario in ('synchronized', 'normal', 'missing', 'readonly', 'foreign', 'dropped', 'revoke', 'isolation', 'heartbeat', 'arm_failure', 'no_arm'):
        run(*sys.argv[1:], scenario)
