"""Deterministic whole-hand criteria and protection tests; no hardware."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0, str(Path(__file__).resolve().parent))
from main import SdkHandNode
from test_scheduler import DelayedLink
from rh56ftp_node import Snapshot, canonical_to_rh


class Tests(unittest.TestCase):
    def setUp(self):
        self.links = {s: DelayedLink() for s in ('left', 'right')}
        self.node = SdkHandNode(self.links, clock_id='test', closing_hold_ms=200)
        self.node.diag = Mock()
        self.now = 1_000_000
        self.sample()

    def sample(self, left=None, right=None, errors=None):
        for side, positions in (('left', left), ('right', right)):
            data = dict(angle=canonical_to_rh(positions or [400]*6),
                        **{key: [0]*6 for key in ('force','current','err','status','temp')})
            if errors and side in errors: data['err'] = errors[side]
            self.node.snapshots[side] = Snapshot(data, self.now)

    def command(self, action='one', valid=True, target=.4):
        n = self.node.guard.last_sequence + 1
        cmd = dict(msg_type='HandCommand', version='1.0', sequence=n, timestamp=1,
            sample_mono_us=self.now, clock_id='test', publisher_id='test', session_id='test',
            control_epoch='22222222-2222-4222-8222-222222222222', valid=valid, mode='NORMALIZED_POSITION', action_id=action,
            origin=dict(publisher_id='test', session_id='test', sequence=n,
                        sample_mono_us=self.now, clock_id='test'),
            hands={s: dict(drive_position_normalized=[target]*6) for s in ('left','right')})
        self.node.handle_command('hand.command', json.dumps(cmd), self.now)

    def test_all_six_required_and_hands_pause_independently(self):
        self.command()
        for i in range(1, 21):
            self.now += 20000
            self.sample(left=[400]*5 + [380 if i % 2 else 420])
            self.node.supervise(self.now)
        self.assertEqual(self.links['left'].pauses, 0)
        self.assertEqual(self.links['right'].pauses, 1)
        state = self.node.make_state(self.now)
        self.assertEqual(state['hands']['left']['stop_criteria_met'], [True]*5 + [False])
        self.assertEqual(state['hands']['left']['closing_hold_active'], [False]*6)
        for _ in range(12):
            self.now += 20000; self.sample(); self.node.supervise(self.now)
        self.assertEqual(self.links['left'].pauses, 1)
        self.assertEqual(self.links['right'].pauses, 1)
        self.assertEqual(self.node.guard.last_sequence, 1)
        for link in self.links.values():
            self.assertFalse(any(-1 in values for _,values in link.writes))

    def test_interpolated_target_preserves_same_direction_residual_timer(self):
        self.sample(left=[800]*6, right=[800]*6)
        self.command()
        for i in range(1, 11):
            self.now += 20000
            self.sample(left=[800]*6, right=[800]*6)
            self.command(target=.4-i*.01)
        self.assertTrue(all(self.node.paused.values()))

    def test_disabled_hand_faults_do_not_affect_selected_hand(self):
        unused = self.links['right']
        self.node.links['right'] = None
        self.command()
        for _ in range(15):
            self.now += 20000
            self.sample(errors={'right': [4]*6})
            self.node.supervise(self.now)
        self.assertEqual(self.links['left'].pauses, 1)
        self.assertFalse(unused.writes)
        self.assertFalse(self.node.hold_control_error)
        self.assertTrue(self.node.make_state(self.now)['valid'])

    def test_feedback_loss_pauses_and_recovery_needs_new_action(self):
        self.command()
        self.now += 600000
        self.node.supervise(self.now)
        self.assertFalse(self.node.command_valid)
        self.assertTrue(all(self.node.paused.values()))
        writes = sum(len(link.writes) for link in self.links.values())
        self.now += 20000; self.sample(); self.node.supervise(self.now)
        self.assertEqual(sum(len(link.writes) for link in self.links.values()), writes)
        with self.assertRaisesRegex(ValueError, 'new action_id'):
            self.command('one')
        self.command('two')
        self.assertFalse(any(self.node.paused.values()))

    def test_device_fault_pause_never_writes_minus_one(self):
        self.command()
        self.now += 20000
        self.sample(errors={'left': [4,0,0,0,0,0]})
        self.node.supervise(self.now)
        self.assertFalse(self.node.command_valid)
        self.assertTrue(all(self.node.paused.values()))
        for link in self.links.values():
            self.assertFalse(any(-1 in values for _,values in link.writes))


if __name__ == '__main__': unittest.main()
