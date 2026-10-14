"""Offline regression tests with real I/O delays; no SDK or robot connection."""
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
from main import SdkHandNode
from scheduler import Scheduler
from rh56ftp_node import monotonic_us, normalized_to_raw, DEFAULT_SAFE_POSE, canonical_to_rh


class DelayedLink:
    host, port = 'offline', None

    def __init__(self):
        self.lock = threading.Lock()
        self.angle = [1000] * 6
        self.writes = []
        self.reads = {'position': [], 'telemetry': []}
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release.set()
        self.fail = False

    def connect(self): pass
    def close(self): pass
    def write_mode_set(self, values): pass
    def write_speed_set(self, values): pass
    def write_force_set(self, values): pass

    def write_angle_set(self, values):
        with self.lock:
            self.entered.set()
            if not self.release.wait(3):
                raise RuntimeError('offline test timed out')
            time.sleep(.003)
            if self.fail:
                raise RuntimeError('injected write failure')
            self.writes.append((monotonic_us(), list(values)))
            self.angle = [old if value is None or value == -1 else value
                          for old, value in zip(self.angle, values)]

    def read_position(self):
        with self.lock:
            self.reads['position'].append(monotonic_us())
            return {'angle': list(self.angle)}

    def read_telemetry(self):
        with self.lock:
            self.reads['telemetry'].append(monotonic_us())
            return {key: [0]*6 for key in ('force', 'current', 'err', 'status', 'temp')}


class Tests(unittest.TestCase):
    def setUp(self):
        self.schedulers = []
        self.seq = 0

    def tearDown(self):
        for scheduler in self.schedulers:
            for link in scheduler.node.links.values():
                if link:
                    link.release.set()
            scheduler.close()
            self.assertIsNone(scheduler.failure)

    def create(self, settings=None, **kwargs):
        links = {side: DelayedLink() for side in ('left', 'right')}
        node = SdkHandNode(links, **kwargs)
        node.diag = Mock()
        scheduler = Scheduler(node, settings or {})
        self.schedulers.append(scheduler)
        scheduler.start()
        self.until(lambda: scheduler.view._safe_applied and scheduler.make_state()['valid'])
        for link in links.values():
            link.writes.clear()
            link.entered.clear()
        return scheduler, links

    def send(self, scheduler, target=.2, valid=True, stamp=None):
        self.seq += 1
        now = stamp or monotonic_us()
        command = dict(msg_type='HandCommand', version='1.0', sequence=self.seq,
                       timestamp=1, sample_mono_us=now, clock_id=scheduler.node.clock_id,
                       publisher_id='test', session_id='11111111-1111-4111-8111-111111111111',
                       control_epoch='22222222-2222-4222-8222-222222222222', valid=valid,
                       mode='NORMALIZED_POSITION',
                       origin=dict(publisher_id='test', session_id='11111111-1111-4111-8111-111111111111',
                                   sequence=self.seq, sample_mono_us=now, clock_id=scheduler.node.clock_id),
                       hands={side: dict(drive_position_normalized=[target]*6) for side in ('left','right')})
        return scheduler.receive('hand.command', json.dumps(command))

    def until(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.002)
        self.fail('condition did not become true')

    def feed(self, scheduler, duration, target=.3):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.assertTrue(self.send(scheduler, target))
            time.sleep(.02)

    def test_slow_write_does_not_block_reception_state_or_other_hand(self):
        scheduler, links = self.create()
        links['left'].release.clear()
        self.assertTrue(self.send(scheduler, .2))
        self.assertTrue(links['left'].entered.wait(1))
        self.until(lambda: bool(links['right'].writes))  # independent port
        for i in range(12):  # exceed the old 100 ms watchdog while I/O is busy
            started = time.monotonic()
            self.assertTrue(self.send(scheduler, .3 + i/100))
            state = scheduler.make_state()
            self.assertIsNone(state['accepted_command'])  # left not complete
            self.assertLess(time.monotonic()-started, .05)
            time.sleep(.02)
        links['left'].release.set()
        self.feed(scheduler, .25, .41)
        state = scheduler.make_state()
        self.assertTrue(state['command_valid'])
        self.assertGreater(state['accepted_command']['sequence'], 10)
        self.assertGreater(scheduler.coalesced, 5)
        self.assertEqual(links['left'].angle, [410]*6)
        # No protective open inserted merely because a write took >100 ms.
        self.assertFalse(any(call.args[0] == 'command_watchdog_expired'
                             for call in scheduler.node.diag.emit.call_args_list))
        self.assertFalse(any(300 in values for _, values in links['left'].writes))
        # The same watchdog still reacts to an actually missing source.
        self.until(lambda: scheduler.view._safe_applied)
        self.assertFalse(scheduler.make_state()['command_valid'])

    def test_failed_write_never_acknowledges_then_recovers(self):
        scheduler, links = self.create()
        links['left'].fail = True
        self.feed(scheduler, .16)
        self.assertIsNone(scheduler.make_state()['accepted_command'])
        links['left'].fail = False
        self.feed(scheduler, .2)
        self.assertIsNotNone(scheduler.make_state()['accepted_command'])
        self.assertTrue(scheduler.make_state()['command_valid'])
        previous = scheduler.received_guard.last_sequence
        self.assertFalse(self.send(scheduler, stamp=monotonic_us()-200_000))
        self.assertEqual(scheduler.received_guard.last_sequence, previous)

    def test_rates_are_applied_independently_without_catchup_bursts(self):
        scheduler, links = self.create({'position_write_hz': 10, 'position_read_hz': 8,
                                        'telemetry_read_hz': 4}, command_timeout_ms=200)
        for link in links.values():
            for values in link.reads.values(): values.clear()
        for i in range(35):
            self.send(scheduler, .2 + i/100)
            time.sleep(.02)
        for link in links.values():
            self.assertGreaterEqual(len(link.writes), 5)
            self.assertLessEqual(len(link.writes), 8)
            self.assertGreaterEqual(len(link.reads['position']), 4)
            self.assertLessEqual(len(link.reads['position']), 7)
            self.assertGreaterEqual(len(link.reads['telemetry']), 2)
            self.assertLessEqual(len(link.reads['telemetry']), 4)
            for values, period in (([t for t,_ in link.writes], 100_000),
                                   (link.reads['position'], 125_000),
                                   (link.reads['telemetry'], 250_000)):
                self.assertTrue(all(b-a >= period*.9 for a,b in zip(values, values[1:])))

    def test_revocation_not_coalesced_away(self):
        scheduler, links = self.create()
        links['left'].release.clear()
        self.send(scheduler, .2)
        self.assertTrue(links['left'].entered.wait(1))
        self.send(scheduler, valid=False)
        self.feed(scheduler, .16, .4)  # revoke itself expires while first write blocks
        links['left'].release.set()
        self.feed(scheduler, .25, .4)
        safe = canonical_to_rh(normalized_to_raw(list(DEFAULT_SAFE_POSE)))
        writes = [values for _, values in links['left'].writes]
        self.assertIn(safe, writes)
        self.assertEqual(links['left'].angle, [400]*6)
        self.assertTrue(scheduler.make_state()['command_valid'])

    def test_slow_hold_transition_does_not_reset_hold(self):
        scheduler, links = self.create(closing_hold_ms=150)
        self.feed(scheduler, .12, .3)
        links['left'].release.clear()
        links['left'].entered.clear()
        self.feed(scheduler, .35, .3)
        self.assertTrue(links['left'].entered.is_set())
        links['left'].release.set()
        self.feed(scheduler, .2, .3)
        self.assertTrue(all(c.held_raw is not None for c in scheduler.view.closing_holds['left']))
        self.assertTrue(scheduler.make_state()['command_valid'])

    def test_invalid_frequency_rejected(self):
        for value in (0, -1, float('nan'), True, 101):
            with self.assertRaises(ValueError):
                Scheduler(SdkHandNode({'left': DelayedLink(), 'right': None}),
                          {'position_write_hz': value})


if __name__ == '__main__':
    unittest.main()
