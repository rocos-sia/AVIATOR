"""Bus reception/publication never waits for SDK I/O.

One policy owner preserves the RH56 hold/fault rules. Left/right writes run in
parallel, with a reader per port. Each port's transport serializes transactions.
Receipt freshness and completed device ACKs are deliberately separate clocks.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import copy, deepcopy
from dataclasses import replace
import json
import math
import signal
import threading

from rh56ftp_node import CommandGuard, Logger, decode_command, monotonic_us


class Scheduler:
    def __init__(self, node, settings, *, error_observer=None):
        self.node = node
        self.periods = {}
        self.error_observer = error_observer
        rates = [("position_write_hz", 20), ("position_read_hz", 10), ("telemetry_read_hz", 5)]
        if 'error_read_hz' in settings:
            rates.append(('error_read_hz', 20))
        for name, default in rates:
            value = settings.get(name, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= 100:
                raise ValueError(f"{name} must be in (0,100]")
            self.periods[name] = round(1_000_000 / value)
        self.condition = threading.Condition()
        self.received_guard = CommandGuard()
        self.pending = self.revocation = None
        self.revoked_stamp = 0
        self.stop = threading.Event()
        self.read_stop = threading.Event()
        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="hand-write")
        self.readers = []
        self.worker = None
        self.failure = None
        self.received = self.rejected = self.coalesced = 0
        self.state_sequence = 0
        self.next_diagnostic_us = 0
        self.feedback_wait_started_us = None
        # Hooks affect only nodes using this runner; Modbus retains its loop.
        node._write_targets = self.write_targets
        self.publish_policy()

    def write_targets(self, targets):
        if self.node.feedback_only:
            return
        # Preserve the original barrier: configure both hands before moving
        # either. All futures must finish even when one port fails.
        for operation in (self.node._write_settings, self.node._write_angles):
            jobs = [self.pool.submit(operation, side, targets[side])
                    for side, link in self.node.links.items() if link is not None]
            errors = []
            for job in jobs:
                try:
                    job.result()
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise errors[0]
        self.node.last_write_error = ""

    def receive(self, topic, payload):
        now = monotonic_us()
        self.received += 1
        try:
            command = decode_command(topic, payload)
            with self.condition:
                candidate = replace(self.received_guard)
                candidate.accept(command, now, self.node.command_timeout_us, self.node.clock_id)
                if self.node.feedback_only:
                    return False
                self.received_guard = candidate
                item = (topic, payload, command)
                if not command['valid']:
                    # Revocations cannot be coalesced away by a following target.
                    self.revocation = item
                    self.pending = None
                    self.revoked_stamp = command['sample_mono_us']
                else:
                    self.coalesced += self.pending is not None
                    self.pending = item
                self.condition.notify()
            return True
        except Exception as exc:
            self.rejected += 1
            self.node.diag.emit('reject command', reason=f'{type(exc).__name__}: {exc}')
            return False

    def publish_policy(self):
        # Immutable policy snapshot: bus publication must never hold a lock
        # while the policy owner is blocked in a device write.
        view = copy(self.node)
        for name in ('guard', 'last_targets', 'requested_targets', 'closing_holds',
                     'fault_channels', '_fault_open_sides', 'write_stats', 'paused'):
            setattr(view, name, deepcopy(getattr(self.node, name)))
        self.view = view

    def make_state(self):
        view = copy(self.view)
        view.snapshots = dict(self.node.snapshots)
        self.state_sequence += 1
        view.state_sequence = self.state_sequence - 1
        state = view.make_state()
        with self.condition:
            # Receipt alone never advances accepted_command. A revocation
            # invalidates a still-running old write; ordinary silence does not.
            if view.guard.sample_mono_us < self.revoked_stamp:
                state['command_valid'] = False
        state['scheduler'] = dict(received=self.received, rejected=self.rejected,
                                  coalesced=self.coalesced,
                                  **{key: 1_000_000 / period for key, period in self.periods.items()})
        view.command_received = self.received
        view.command_rejected += self.rejected
        view._next_diagnostic_us = self.next_diagnostic_us
        view._feedback_wait_started_us = self.feedback_wait_started_us
        view.state_sent = self.node.state_sent
        view.state_dropped = self.node.state_dropped
        view.last_state_sent_us = self.node.last_state_sent_us
        view.max_state_gap_ms = self.node.max_state_gap_ms
        view.report_diagnostics(state)
        self.next_diagnostic_us = view._next_diagnostic_us
        self.feedback_wait_started_us = view._feedback_wait_started_us
        return state

    def start(self):
        Logger.info('xCoreSDK_hand: 独立收发与设备 I/O；左右手并行；'
                    + '，'.join(f'{key}={1_000_000 / period:g}' for key, period in self.periods.items())
                    + ' Hz（频率上限）；ACK 仅在所需写入完成后更新')
        self.node.connect()
        for side, link in self.node.read_links.items():
            if link is not None:
                reader = threading.Thread(target=self.read_loop, args=(side, link),
                                          name=f'hand-read-{side}')
                self.readers.append(reader)
                reader.start()
        self.worker = threading.Thread(target=self.control_loop, name='hand-policy')
        self.worker.start()

    def read_loop(self, side, link):
        periods = {'position': self.periods['position_read_hz'],
                   'telemetry': self.periods['telemetry_read_hz']}
        if 'error_read_hz' in self.periods:
            periods = {'errors': self.periods['error_read_hz'], **periods}
        due = dict.fromkeys(periods, 0)
        data, stamps, errors = {}, {}, {}
        while not self.read_stop.is_set():
            kind = min(due, key=due.get)
            now = monotonic_us()
            if due[kind] > now:
                self.read_stop.wait((due[kind] - now) / 1_000_000)
                continue
            before = self.node.snapshots[side]
            started = monotonic_us()
            try:
                operation = getattr(link, 'read_' + kind, None)
                if operation is None:  # In-memory fake HandLink; no SDK traffic.
                    full = link.read_state()
                    fields = (('angle',) if kind == 'position' else ('err',) if kind == 'errors'
                              else ('force', 'current', 'err', 'status', 'temp'))
                    update = {key: full[key] for key in fields}
                else:
                    update = operation()
                for values in update.values():
                    if not isinstance(values, list) or len(values) != 6 or any(
                            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                        raise ValueError('Invalid hand feedback values')
                if kind == 'errors' and (set(update) != {'err'} or any(
                        type(v) is not int or not 0 <= v <= 255 for v in update['err'])):
                    raise ValueError('Invalid six-byte fault register response')
                if kind == 'telemetry' and 'errors' in periods:
                    update.pop('err', None)  # Dedicated samples own the fault codes.
                data.update(update)
                stamps[kind] = started
                errors.pop(kind, None)
                stamp = min(stamps.values()) if len(stamps) == len(periods) else None
                snapshot = replace(before, data=dict(data) if stamp else None,
                                   sample_mono_us=stamp, reads=before.reads + 1,
                                   error='; '.join(errors.values()),
                                   last_success_finished_us=monotonic_us())
                if before.error and not snapshot.error:
                    self.node.diag.emit('feedback_recovered', side=side)
            except Exception as exc:
                errors[kind] = str(exc)
                snapshot = replace(before, error='; '.join(errors.values()), errors=before.errors + 1)
                self.node.diag.emit('feedback_read_failed', key=f'read:{side}:{kind}',
                                    side=side, reason=str(exc), duration_ms=(monotonic_us()-started)/1000)
            if kind == 'errors' and self.error_observer is not None:
                # Observe every actual read, including short faults between UI refreshes.
                self.error_observer(side, None if kind in errors else update['err'],
                                    errors.get(kind, ''))
            finished = monotonic_us()
            self.node.snapshots[side] = replace(snapshot, read_started_us=started,
                                                read_finished_us=finished,
                                                read_duration_ms=(finished-started)/1000)
            # No catch-up bursts. These are rate ceilings under a slow SDK.
            due[kind] = max(started + periods[kind], finished)

    def supervise(self):
        self.node.supervise()

    def control_loop(self):
        node = self.node
        try:
            if not node.feedback_only:
                try:
                    node._safe_pose(force_open=True)
                except Exception as exc:
                    node.diag.emit('startup_safe_pose_failed', reason=str(exc))
            self.publish_policy()
            next_write = next_supervise = 0
            while not self.stop.is_set():
                item = None
                with self.condition:
                    if self.revocation is not None:
                        item, self.revocation = self.revocation, None
                    elif self.pending is not None and monotonic_us() >= next_write:
                        item, self.pending = self.pending, None
                if item is not None:
                    topic, payload, command = item
                    now = monotonic_us()
                    try:
                        # Never replay an expired queued motion. A queued revoke
                        # still needs its safe action even if its ACK is too old.
                        if not command['valid'] and now-command['sample_mono_us'] >= node.command_timeout_us:
                            node._safe_pose()
                        else:
                            node.handle_command(topic, payload, now)
                    except Exception as exc:
                        # handle_command logs its own failures. The direct safe
                        # action for an expired revoke needs the same visibility.
                        if not command['valid']:
                            node.diag.emit('safe_pose_failed', reason=str(exc))
                    next_write = max(now + self.periods['position_write_hz'], monotonic_us())
                if not node.feedback_only and monotonic_us() >= next_supervise:
                    self.supervise()
                    next_supervise = monotonic_us() + self.periods['position_write_hz']
                self.publish_policy()
                with self.condition:
                    if not self.stop.is_set():
                        # Predicate avoids missing a notification during I/O.
                        self.condition.wait_for(lambda: self.stop.is_set() or self.revocation is not None or
                                                (self.pending is not None and monotonic_us() >= next_write),
                                                timeout=0.005)
        except Exception as exc:
            self.failure = exc
        finally:
            self.read_stop.set()
            for reader in self.readers:
                reader.join()
            if not node.feedback_only:
                try:
                    node._safe_pose(force_open=True)
                except Exception as exc:
                    node.diag.emit('shutdown_safe_pose_failed', reason=str(exc))
            self.publish_policy()

    def close(self):
        self.stop.set()
        self.read_stop.set()
        with self.condition:
            self.condition.notify_all()
        if self.worker:
            self.worker.join()
        for reader in self.readers:
            reader.join()
        self.pool.shutdown(wait=True)
        self.node.close()  # Adapter only: never disconnectFromRobot().


def run_scheduled(node, endpoint, state_endpoint, state_hz, settings):
    import zmq
    scheduler = Scheduler(node, settings)
    context = zmq.Context()
    sub, pub = context.socket(zmq.SUB), context.socket(zmq.PUB)
    sub.setsockopt(zmq.RCVHWM, 32)
    sub.setsockopt(zmq.SUBSCRIBE, b'hand.command')
    pub.setsockopt(zmq.SNDHWM, 8)
    sub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.LINGER, 0)
    sub.connect(endpoint)
    pub.connect(state_endpoint)
    stopped = threading.Event()
    def stop(*_):
        stopped.set()
    old_int = signal.signal(signal.SIGINT, stop)
    old_term = signal.signal(signal.SIGTERM, stop)
    try:
        scheduler.start()
        period = round(1_000_000 / state_hz)
        due = 0
        while not stopped.is_set():
            if scheduler.failure is not None:
                raise RuntimeError(f'Hand scheduler failed: {scheduler.failure}')
            # Bound reception work so a busy publisher cannot starve state.
            for _ in range(64):
                if not sub.poll(0):
                    break
                frames = sub.recv_multipart()
                if len(frames) == 2:
                    try:
                        scheduler.receive(frames[0].decode('ascii'), frames[1])
                    except UnicodeDecodeError:
                        pass
            now = monotonic_us()
            if now >= due:
                state = scheduler.make_state()
                try:
                    pub.send_multipart([b'hand.state', json.dumps(state, allow_nan=False).encode()], zmq.NOBLOCK)
                    sent = monotonic_us()
                    if node.last_state_sent_us:
                        node.max_state_gap_ms = max(node.max_state_gap_ms, (sent-node.last_state_sent_us)/1000)
                    node.last_state_sent_us = sent
                    node.state_sent += 1
                except zmq.Again:
                    node.state_dropped += 1
                due = now + period
            stopped.wait(0.001)
    finally:
        scheduler.close()
        sub.close()
        pub.close()
        context.term()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
    return 0
