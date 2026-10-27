"""Modbus pause policy, device write contract, and streaming watchdog regressions."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, call, patch

from rh56ftp_node import WholeHandPauseNode, Snapshot, canonical_to_rh, main
from fake_rh56ftp_hand import FakeHandLink, FakeRh56FtpNode


class Link:
    def __init__(self):
        self.events = []
        self.fail_pause = False
        self.fail_angles = False

    def write_angle_set(self, values):
        if self.fail_angles:
            raise RuntimeError('angle failed')
        if -1 in values:
            raise AssertionError('pause marker escaped into angle registers')
        self.events.append(('angle', list(values)))

    def write_pause(self):
        if self.fail_pause:
            raise RuntimeError('pause failed')
        self.events.append(('pause', 1))

    def clear_error(self):
        self.events.append(('clear_error', 1))

    def write_speed_set(self, values):
        self.events.append(('speed', list(values)))

    def write_force_set(self, values):
        self.events.append(('force', list(values)))

    def write_mode_set(self, values):
        self.events.append(('mode', list(values)))


class PauseTests(unittest.TestCase):
    def setUp(self):
        self.links = {s: Link() for s in ('left', 'right')}
        self.node = WholeHandPauseNode(self.links, clock_id='test', closing_hold_ms=200,
                                      hold_speed=0, hold_force=0)
        self.node.diag = Mock()
        self.now = 1_000_000
        self.sample()

    def sample(self, left=None, errors=None):
        for side in self.links:
            data = dict(angle=canonical_to_rh(left if side == 'left' and left else [400]*6),
                        **{key: [0]*6 for key in ('force', 'current', 'err', 'status', 'temp')})
            if errors and side == 'left':
                data['err'] = errors
            self.node.snapshots[side] = Snapshot(data, self.now)

    def send(self, action='one', target=.4, valid=True, hold=True):
        n = self.node.guard.last_sequence + 1
        message = dict(msg_type='HandCommand', version='1.0', sequence=n, timestamp=1,
                       sample_mono_us=self.now, clock_id='test', publisher_id='test', session_id='test',
                       control_epoch='22222222-2222-4222-8222-222222222222', valid=valid,
                       mode='NORMALIZED_POSITION', hold_after_motion=hold,
                       origin=dict(publisher_id='test', session_id='test', sequence=n,
                                   sample_mono_us=self.now, clock_id='test'),
                       hands={s: dict(drive_position_normalized=[target]*6) for s in self.links})
        if action is not None:
            message['action_id'] = action
        self.node.handle_command('hand.command', json.dumps(message), self.now)

    def settle(self, action='one', target=.4):
        self.send(action, target)
        for _ in range(11):
            self.now += 20000
            self.sample()
            self.send(action, target)
        self.assertTrue(all(self.node.paused.values()))

    def test_six_channels_required_and_hands_pause_independently(self):
        self.send()
        for i in range(21):
            self.now += 20000
            self.sample(left=[400]*5 + [380 if i % 2 else 420])
            self.send()
        state = self.node.make_state(self.now)['hands']
        self.assertFalse(state['left']['paused'])
        self.assertEqual(state['left']['stop_criteria_met'], [True]*5 + [False])
        self.assertEqual(state['left']['closing_hold_active'], [False]*6)
        self.assertTrue(state['right']['paused'])
        self.assertEqual(self.links['right'].events.count(('pause', 1)), 1)
        self.assertNotIn(('pause', 1), self.links['left'].events)

    def test_pause_once_without_hold_settings_and_repeat_does_not_resume(self):
        self.send()
        original = {s: list(link.events) for s, link in self.links.items()}
        self.settle()
        for side, link in self.links.items():
            self.assertEqual(link.events, original[side] + [('pause', 1)])
        self.now += 20000; self.sample(); self.send()
        for side, link in self.links.items():
            self.assertEqual(link.events, original[side] + [('pause', 1)])
        self.assertEqual(self.node.make_state(self.now)['accepted_command']['action_id'], 'one')
        self.now += 20000; self.sample(); self.send('two')
        self.assertFalse(any(self.node.paused.values()))
        for link in self.links.values():
            self.assertEqual(link.events[-1], ('angle', [400]*6))

    def test_release_never_auto_pauses_even_if_stable_or_blocked(self):
        for target in (.4, .9):
            with self.subTest(target=target):
                self.setUp(); self.settle()
                before = {side: len(link.events) for side, link in self.links.items()}
                self.now += 20000; self.sample(); self.send('release', target, hold=False)
                for _ in range(25):
                    self.now += 20000; self.sample(); self.send('release', target, hold=False)
                self.assertFalse(any(self.node.paused.values()))
                for side, link in self.links.items():
                    self.assertEqual(link.events[before[side]:], [('angle', [round(target*1000)]*6)] * 10)
                    self.assertEqual(self.node.release_writes_remaining[side], 0)
                    self.assertEqual(self.node.make_state(self.now)['hands'][side]['stop_criteria_met'], [False]*6)
                # Device faults must still pause an open action.
                self.now += 20000; self.sample(errors=[4, 0, 0, 0, 0, 0]); self.node.supervise(self.now)
                self.assertTrue(all(self.node.paused.values()))

    def test_release_burst_retries_failed_frame_and_new_action_gets_ten(self):
        self.send('release', .9, hold=False)
        for _ in range(3):
            self.now += 20000; self.sample(); self.send('release', .9, hold=False)
        self.links['right'].fail_angles = True
        self.now += 20000; self.sample()
        sequence = self.node.guard.last_sequence
        with self.assertRaisesRegex(RuntimeError, 'angle failed'):
            self.send('release', .9, hold=False)
        self.assertEqual(self.node.guard.last_sequence, sequence)
        self.assertEqual(self.node.release_writes_remaining, dict(left=5, right=6))
        self.links['right'].fail_angles = False
        for _ in range(20):
            self.now += 20000; self.sample(); self.send('release', .9, hold=False)
        for link in self.links.values():
            self.assertEqual(link.events.count(('angle', [900]*6)), 10)
        self.now += 20000; self.sample(); self.send('another-release', .9, hold=False)
        self.assertEqual(self.node.release_writes_remaining, dict(left=9, right=9))
        for _ in range(20):
            self.now += 20000; self.sample(); self.send('another-release', .9, hold=False)
        for link in self.links.values():
            self.assertEqual(link.events.count(('angle', [900]*6)), 20)

    def test_new_grasp_and_fault_cancel_unfinished_release(self):
        self.send('release', .9, hold=False)
        self.now += 20000; self.sample(); self.send('grasp', .4)
        self.assertEqual(self.node.release_writes_remaining, dict(left=0, right=0))
        for _ in range(15):
            self.now += 20000; self.sample(); self.send('grasp', .4)
        for link in self.links.values():
            self.assertEqual(link.events.count(('angle', [900]*6)), 1)
        self.now += 20000; self.sample(); self.send('release-2', .9, hold=False)
        self.now += 20000; self.sample(errors=[4, 0, 0, 0, 0, 0]); self.node.supervise(self.now)
        self.assertEqual(self.node.release_writes_remaining, dict(left=0, right=0))
        self.assertTrue(all(self.node.paused.values()))

    def test_invalid_hold_flag_cannot_resume_or_change_ack(self):
        self.settle()
        sequence = self.node.guard.last_sequence
        before = {side: list(link.events) for side, link in self.links.items()}
        for flag in ('false', 0, None):
            with self.subTest(flag=flag), self.assertRaisesRegex(ValueError, 'hold_after_motion'):
                self.send('release', .9, hold=flag)
        self.assertEqual(self.node.guard.last_sequence, sequence)
        self.assertEqual(before, {side: link.events for side, link in self.links.items()})

    def test_new_target_resumes_legacy_client_without_action_id(self):
        self.settle(action=None)
        self.now += 20000; self.sample(); self.send(None, .3)
        self.assertFalse(any(self.node.paused.values()))
        for link in self.links.values():
            self.assertEqual(link.events[-1], ('angle', [300]*6))

    def test_failed_pause_retries_without_resetting_window_or_advancing_ack(self):
        for failed in ('left', 'right'):
            with self.subTest(failed=failed):
                self.setUp()
                self.send()
                for _ in range(9):
                    self.now += 20000; self.sample(); self.send()
                sequence = self.node.guard.last_sequence
                self.links[failed].fail_pause = True
                self.now += 20000; self.sample()
                with self.assertRaisesRegex(RuntimeError, 'pause failed'):
                    self.send()
                self.assertEqual(self.node.guard.last_sequence, sequence)
                for side in self.links:
                    self.assertEqual(self.node.paused[side], side != failed)
                self.links[failed].fail_pause = False
                self.now += 20000; self.sample(); self.send()
                self.assertTrue(all(self.node.paused.values()))
                for link in self.links.values():
                    self.assertEqual(link.events.count(('pause', 1)), 1)

    def test_invalid_or_replayed_request_cannot_resume(self):
        self.settle()
        previous = {s: list(link.events) for s, link in self.links.items()}
        with self.assertRaisesRegex(ValueError, 'action_id'):
            self.send(action=5)
        with self.assertRaises(ValueError):
            self.node.handle_command('hand.command', json.dumps(self.node.last_command), self.now + 20000)
        self.assertTrue(all(self.node.paused.values()))
        self.assertEqual(previous, {s: link.events for s, link in self.links.items()})

    def test_faults_pause_without_minus_one_and_timeout_still_opens_when_healthy(self):
        self.settle()
        self.now += 120000; self.sample(); self.node.supervise(self.now)
        self.assertFalse(self.node.command_valid)
        self.assertFalse(any(self.node.paused.values()))
        for link in self.links.values():
            self.assertEqual(link.events[-1], ('angle', [1000]*5 + [500]))
        self.now += 20000; self.sample(); self.send('two')
        self.now += 20000; self.sample(errors=[4, 0, 0, 0, 0, 0]); self.node.supervise(self.now)
        self.assertTrue(all(self.node.paused.values()))

    def test_clear_three_times_then_error_without_fourth_attempt(self):
        self.send()
        for attempt in range(1, 4):
            self.now += 210000
            self.sample(errors=[4]*6)
            self.send()
            self.assertEqual(self.node.error_recovery['left']['attempts'], attempt)
            self.assertEqual(self.node.make_state(self.now)['hands']['left']['status'], 'RECOVERING')
            self.assertFalse(self.node.make_state(self.now)['command_valid'])
            self.assertTrue(all(self.node.paused.values()))
        self.now += 210000; self.sample(errors=[2]*6); self.send('new-action')
        self.assertEqual(self.node.make_state(self.now)['hands']['left']['status'], 'ERROR')
        for _ in range(5):
            self.now += 210000; self.sample(errors=[2]*6); self.send()
        self.assertEqual(self.links['left'].events.count(('clear_error', 1)), 3)
        self.assertNotIn(('clear_error', 1), self.links['right'].events)
        self.assertLess(self.links['left'].events.index(('pause', 1)),
                        self.links['left'].events.index(('clear_error', 1)))

    def test_clear_success_requires_new_feedback_and_new_episode_gets_budget(self):
        self.send()
        self.now += 20000; self.sample(errors=[4,0,0,0,0,0]); self.send()
        stamp = self.node.snapshots['left'].sample_mono_us
        self.node.snapshots['left'].data['err'] = [0]*6
        self.now += 20000; self.send()
        self.assertEqual(self.node.error_recovery['left']['phase'], 'clearing')
        self.assertEqual(self.node.snapshots['left'].sample_mono_us, stamp)
        self.now += 20000; self.sample(); self.send()
        self.assertEqual(self.node.error_recovery['left']['phase'], 'recovered')
        self.assertTrue(self.node.make_state(self.now)['command_valid'])
        self.assertFalse(any(self.node.paused.values()))
        self.now += 20000; self.sample(errors=[4]*6); self.send()
        self.assertEqual(self.node.error_recovery['left']['attempts'], 1)

    def test_hands_recover_independently_and_second_attempt_can_succeed(self):
        self.sample(errors=[4]*6)
        self.node.snapshots['right'].data['err'] = [2]*6
        self.node.supervise(self.now)
        self.now += 20000; self.sample(errors=[4]*6); self.node.supervise(self.now)
        self.assertEqual(self.node.error_recovery['right']['phase'], 'recovered')
        self.now += 210000; self.sample(errors=[4]*6); self.node.supervise(self.now)
        self.assertEqual(self.node.error_recovery['left']['attempts'], 2)
        self.now += 20000; self.sample(); self.node.supervise(self.now)
        self.assertEqual(self.node.error_recovery['left']['phase'], 'recovered')
        self.now += 210000; self.sample(); self.node.supervise(self.now)
        self.assertEqual(self.links['left'].events.count(('clear_error', 1)), 2)
        self.assertEqual(self.links['right'].events.count(('clear_error', 1)), 1)

    def test_clear_write_errors_and_missing_feedback_are_bounded(self):
        for write_error in (False, True):
            with self.subTest(write_error=write_error):
                self.setUp()
                operation = Mock(side_effect=RuntimeError('clear failed') if write_error else None)
                self.links['left'].clear_error = operation
                self.sample(errors=[4]*6)
                self.node.supervise(self.now)
                for _ in range(5):
                    self.now += 210000
                    self.node.snapshots['left'].error = 'read failed'
                    self.node.supervise(self.now)
                self.assertEqual(operation.call_count, 3)
                self.assertEqual(self.node.make_state(self.now)['hands']['left']['status'], 'ERROR')

    def test_feedback_only_never_clears(self):
        self.node.feedback_only = True
        self.sample(errors=[4]*6)
        self.node.supervise(self.now)
        self.assertFalse(any(link.events for link in self.links.values()))
        self.assertEqual(self.node.make_state(self.now)['hands']['left']['status'], 'ERROR')

    def test_cli_selects_pause_policy(self):
        with patch('rh56ftp_node.load_handlink', return_value=('fake', 6000, Mock())), \
                patch('rh56ftp_node.run_node', return_value=0) as run, patch('sys.stderr'):
            main([])
        self.assertIsInstance(run.call_args.args[0], WholeHandPauseNode)
        self.assertTrue(issubclass(FakeRh56FtpNode, WholeHandPauseNode))

    def test_fake_pause_stops_all_axes_and_new_position_resumes(self):
        now = [0.0]
        link = FakeHandLink(clock=lambda: now[0])
        link.connect(); link.write_angle_set([0]*6)
        now[0] = .1; link.write_pause()
        stopped = link.read_state()['angle']
        now[0] = 1
        self.assertEqual(link.read_state()['angle'], stopped)
        link.write_angle_set([0]*6)
        now[0] = 2
        self.assertEqual(link.read_state()['angle'], [0]*6)

    def test_modbus_pause_register_and_error(self):
        path = Path(__file__).parents[2] / 'third_party/RH56FTP/python/pendant/handlink.py'
        spec = importlib.util.spec_from_file_location('pause_handlink', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'pymodbus': Mock(), 'pymodbus.client': Mock()}):
            spec.loader.exec_module(module)
        link = module.HandLink(); link._cli = Mock()
        link._cli.write_registers.return_value.isError.return_value = False
        link.write_pause()
        self.assertEqual(link._cli.write_registers.call_args_list,
                         [call(address=1634, values=[1], device_id=1)])
        link.clear_error()
        self.assertEqual(link._cli.write_registers.call_args,
                         call(address=1004, values=[1], device_id=1))
        link._cli.write_registers.return_value.isError.return_value = True
        with self.assertRaises(module.HandError):
            link.write_pause()


if __name__ == '__main__':
    unittest.main()
