"""RH56FTP ZMQ test peer; positions/thermal recovery are simulated, no serial IO."""
import json
from pathlib import Path
import socket
import threading
import time
import uuid

import zmq


class FakeRH56FTPHand:
    def __init__(self, context, endpoints, hand_config, directory, latched_error=False):
        self.context, self.endpoints = context, endpoints
        self.open = hand_config['open']
        self.close = hand_config['close']
        self.directory = directory
        self.latched_error = latched_error
        self.recovered_feedback_fresh = False
        self.release_arm_commands = []
        self.clock = socket.gethostname() + '-' + Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.session = str(uuid.uuid4())
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.fault_published = threading.Event()
        self.error = None
        self.actual = {side: list(pose) for side, pose in self.open.items()}
        self.fault_code = 0
        self.first_open = self.opened_invalid = self.recovered = None
        self.open_commands = []
        self.states = []
        self.thread = threading.Thread(target=self._run, name='fake-rh56ftp-hand')

    def start(self):
        self.thread.start()

    def finish(self):
        self.stop.set()
        self.thread.join(timeout=5)
        assert not self.thread.is_alive(), 'hand test peer failed to stop'
        self.check()

    def check(self):
        if self.error:
            raise AssertionError('hand test peer failed') from self.error

    def inject(self, code):
        self.check()
        with self.lock:
            assert all(abs(a-b) <= .03 for side in self.close
                       for a, b in zip(self.actual[side], self.close[side])), 'hand was not closed before fault'
            self.fault_code = code
        assert self.fault_published.wait(2), 'fault state was not published'

    def verify_release(self):
        self.check()
        with self.lock:
            assert self.first_open is not None, 'no open command while valid=false'
            assert self.opened_invalid is not None, 'simulated hand did not open while valid=false'
            assert self.recovered is not None, 'release completed before feedback recovered'
            commands = list(self.open_commands)
            states = list(self.states)
            assert 'SAFE' not in states and 'ERROR' not in states, states
            assert self.recovered_feedback_fresh, 'latched error rejected recovered measurements in Core'
            arm_gaps = [b-a for a, b in zip(self.release_arm_commands, self.release_arm_commands[1:])]
            assert arm_gaps and max(arm_gaps) < .1, 'arm.command stopped during release'
            assert len(commands) >= 60, 'open publication stopped during invalid feedback'
            assert all(state == 'RELEASING' for _, state in commands), commands
            gaps = [b[0]-a[0] for a, b in zip(commands, commands[1:])]
            max_gap = max(gaps)
            assert max_gap < .1, f'open command gap {max_gap:.3f}s exceeded watchdog budget'
            assert commands[-1][0] - commands[0][0] > 1.8, 'invalid scenario was too short'
            assert self.first_open < self.opened_invalid < self.recovered
            assert all(abs(a-b) <= .03 for side in self.open
                       for a, b in zip(self.actual[side], self.open[side]))
            assert 'RELEASING' in states and 'STANDBY' in states, states
            result = dict(error_code=self.fault_code, publisher_id='rh56ftp_hand',
                          invalid_open_commands=len(commands), max_command_gap_ms=round(max_gap*1000, 2),
                          opened_after_ms=round((self.opened_invalid-self.first_open)*1000, 2),
                          invalid_duration_ms=round((self.recovered-self.first_open)*1000, 2),
                          opened_while_invalid=True, final_state=states[-1],
                          hold_control_error_latched=self.latched_error,
                          recovered_feedback_fresh=self.recovered_feedback_fresh,
                          max_arm_command_gap_ms=round(max(arm_gaps)*1000, 2))
        (self.directory / 'hand-fault-result.json').write_text(json.dumps(result, indent=2) + '\n')
        print('PASS: RELEASING hand fault ' + json.dumps(result), flush=True)

    def _run(self):
        pub, sub = self.context.socket(zmq.PUB), self.context.socket(zmq.SUB)
        pub.connect(self.endpoints[0])
        sub.connect(self.endpoints[1])
        sub.subscribe(b'hand.command')
        sub.subscribe(b'arm.command')
        sub.subscribe(b'flight.state')
        ack = None
        targets = {side: list(pose) for side, pose in self.open.items()}
        system_state = None
        sequence = 0
        next_state = 0
        try:
            with (self.directory / 'hand-zmq.jsonl').open('w') as trace:
                while not self.stop.is_set():
                    while sub.poll(0):
                        topic, payload = sub.recv_multipart()
                        message = json.loads(payload)
                        now = time.monotonic()
                        if topic == b'flight.state':
                            state = message.get('system', {}).get('state')
                            if state != system_state:
                                system_state = state
                                with self.lock:
                                    self.states.append(state)
                                trace.write(json.dumps(dict(topic='flight.state', time=now, state=state)) + '\n')
                            with self.lock:
                                if self.recovered is not None and message.get('freshness', {}).get('hand', {}).get('valid'):
                                    self.recovered_feedback_fresh = True
                            continue
                        if topic == b'arm.command':
                            if system_state == 'RELEASING':
                                with self.lock:
                                    self.release_arm_commands.append(now)
                            continue
                        assert message['valid'] and message['mode'] == 'NORMALIZED_POSITION'
                        assert message['publisher_id'] == 'aviator_core'
                        assert ack is None or (message['session_id'] == ack['session_id'] and
                                               message['sequence'] > ack['sequence'])
                        stamp = time.monotonic_ns() // 1000
                        assert 0 <= stamp-message['sample_mono_us'] < 100000, 'stale hand command'
                        assert 0 <= message['sample_mono_us']-message['origin']['sample_mono_us'] < 100000
                        targets = {side: message['hands'][side]['drive_position_normalized'] for side in self.open}
                        with self.lock:
                            if self.fault_code and self.recovered is None and targets == self.open:
                                if self.first_open is None:
                                    self.first_open = now
                                self.open_commands.append((now, system_state))
                        ack = {key: message[key] for key in
                               ('publisher_id', 'session_id', 'sequence', 'sample_mono_us', 'control_epoch')}
                        trace.write(json.dumps(dict(topic='hand.command', time=now, message=message)) + '\n')
                    now = time.monotonic()
                    if now >= next_state:
                        next_state = now + .02
                        stamp = time.monotonic_ns() // 1000
                        sequence += 1
                        with self.lock:
                            invalid = bool(self.fault_code and self.recovered is None)
                            elapsed = None if self.first_open is None else now-self.first_open
                            # Fault prevents motion for 800 ms; the drive then recovers,
                            # but its reported validity stays false until 2 seconds.
                            blocked = invalid and (elapsed is None or elapsed < .8)
                            code = self.fault_code if blocked else 0
                            if not blocked:
                                for side in self.actual:
                                    self.actual[side] = [a + max(-.08, min(.08, b-a))
                                                         for a, b in zip(self.actual[side], targets[side])]
                            if invalid and targets == self.open and all(abs(a-b) <= .03 for side in self.open
                                                                       for a, b in zip(self.actual[side], self.open[side])):
                                if self.opened_invalid is None:
                                    self.opened_invalid = now
                            if invalid and elapsed is not None and elapsed >= 2:
                                self.recovered = now
                                invalid = False
                            hands = {}
                            for side in self.actual:
                                errors = [0, code, 0, 0, 0, 0]
                                hands[side] = dict(valid=not invalid, status='ERROR' if code else 'ACTIVE',
                                    sample_mono_us=stamp, position_source='angle_act_register', feedback_available=True,
                                    sample_time_basis='host_modbus_read', feedback_age_ms=0,
                                    drive_position_normalized=list(self.actual[side]),
                                    drive_position_raw=[round(v*1000) for v in self.actual[side]],
                                    commanded_drive_position_normalized=list(targets[side]),
                                    requested_drive_position_normalized=list(targets[side]),
                                    error_code=code, error_codes=errors, status_codes=[0]*6,
                                    current=[0, 1800 if code == 4 else 0, 0, 0, 0, 0],
                                    temperature=[30, 85 if code == 2 else 30, 30, 30, 30, 30],
                                    closing_hold_active=[False]*6, closing_hold_position_normalized=[None]*6)
                        reported_code = self.fault_code if self.latched_error else code
                        message = dict(msg_type='HandState', version='1.0', publisher_id='rh56ftp_hand',
                            session_id=self.session, sequence=sequence, sample_mono_us=stamp,
                            timestamp=time.time_ns()//1000, clock_id=self.clock, valid=not invalid,
                            feedback_only=False, command_valid=ack is not None, accepted_command=ack,
                            hold_control_error=('overcurrent' if reported_code == 4 else 'overtemperature') if reported_code else '',
                            hands=hands)
                        pub.send_multipart([b'hand.state', json.dumps(message).encode()])
                        trace.write(json.dumps(dict(topic='hand.state', time=now, message=message)) + '\n')
                        if invalid:
                            self.fault_published.set()
                    self.stop.wait(.002)
        except BaseException as error:
            self.error = error
        finally:
            pub.close(0)
            sub.close(0)
