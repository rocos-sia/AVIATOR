"""Exercise the UI's controller against fake devices only."""
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import patch
import yaml
from grip_demo import DemoController
from fault_journal import FaultJournal, describe_error


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        (root / 'hand.yaml').write_text(yaml.safe_dump(dict(closing_hold_ms=150,
            threshold=10, motion_rate=20, speed={'grasp':800}, force={'grasp':2000})))
        system = dict(xcore_sdk_hand={'config':'hand.yaml'}, core_hand={
            'open':{s:[.5,1,1,1,1,1] for s in ('left','right')},
            'close':{s:[.5,0,0,0,0,0] for s in ('left','right')}})
        (root / 'system.yaml').write_text(yaml.safe_dump(system))
        self.demo = DemoController(root / 'system.yaml', root / 'must-not-load.so', fake=True,
                                   log_file=root / 'faults.log')

    def tearDown(self):
        self.demo.close()
        self.temp.cleanup()

    def until(self, predicate):
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if predicate(): return
            time.sleep(.01)
        self.fail('timed out')

    def ready(self):
        self.until(lambda: all(s.make_state()['valid'] and s.view._safe_applied
                              for s in self.demo.schedulers.values()))

    def test_selected_side_only_and_clicks(self):
        for side in ('left','right'):
            self.demo.connect(side); self.ready()
            self.assertEqual(set(self.demo.schedulers), {side})
            node = self.demo.schedulers[side].node
            self.assertIsNone(node.links['right' if side == 'left' else 'left'])
            self.demo.action('close')
            self.until(lambda: self.demo.status()['hands'][side]['paused'])
            first = node.action_id
            self.demo.action('close')
            self.until(lambda: node.action_id != first)
            self.until(lambda: self.demo.status()['hands'][side]['paused'])
            self.demo.action('open')
            self.until(lambda: self.demo.status()['hands'][side]['drive_position_normalized'] == [.5,1,1,1,1,1])
            self.demo.close()

    def test_50ms_fault_samples_capture_transient_without_ui_polling(self):
        self.demo.connect('left'); self.ready()
        self.assertFalse(self.demo.faults.path.exists())
        link = self.demo.schedulers['left'].node.links['left']
        times = []
        def read_errors():
            times.append(time.monotonic())
            return {'err': [0,0,0,0,0,5 if len(times) == 2 else 0]}
        with patch.object(link, 'read_errors', side_effect=read_errors, create=True):
            time.sleep(.36)  # No UI/status consumer; don't lose a single 50 ms fault.
        self.assertGreaterEqual(len(times), 6)
        self.assertLessEqual(len(times), 8)
        self.assertTrue(all(.045 <= b-a < .12 for a,b in zip(times, times[1:])), times)
        history, error = self.demo.faults.snapshot()
        self.assertFalse(error)
        self.assertEqual(len(history), 1)
        self.assertEqual((history[0]['side'], history[0]['channel']), ('左手', '拇指侧摆'))
        self.assertEqual(history[0]['description'], '堵转、过流')
        self.assertTrue(history[0]['time'].endswith('+08:00'))
        self.assertIn('故障码=5 (0x05)', self.demo.faults.path.read_text())
        self.assertEqual(set(self.demo.schedulers), {'left'})

    def test_dual_hand_fault_isolation(self):
        self.demo.connect('both'); self.ready()
        right = self.demo.schedulers['right'].node.links['right']
        read = right.read_state
        def faulty():
            state = read(); state['err'] = [4]*6; return state
        with patch.object(right, 'read_state', side_effect=faulty):
            self.until(lambda: bool(self.demo.schedulers['right'].view.hold_control_error))
            self.demo.action('close')
            self.until(lambda: self.demo.status()['hands']['left']['paused'])
            self.assertFalse(self.demo.schedulers['left'].view.hold_control_error)
            self.assertTrue(self.demo.status()['hands']['left']['command_valid'])
            self.assertTrue(self.demo.status()['hands']['right']['diagnostic'])


class JournalTests(unittest.TestCase):
    def test_only_faults_logged_with_combinations_unknown_bits_and_history(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'faults.log'
            journal = FaultJournal(path)
            journal.observe('right', [0]*6)
            self.assertFalse(path.exists())
            journal.observe('right', [31,32,0,0,0,0])
            journal.observe('right', [0]*6)
            journal.observe('right', [31,32,0,0,0,0])
            events, error = journal.snapshot()
            self.assertEqual(len(events), 4)  # Each nonzero sample is retained.
            self.assertFalse(error)
            self.assertIn('未知故障位 0x20', events[0]['description'])
            self.assertEqual(events[1]['channel'], '小指')
            self.assertEqual(describe_error(31), '堵转、过温、过流、电机异常、通信故障')
            self.assertEqual(len(path.read_text().splitlines()), 4)
            journal.observe('left', None, 'SDK timeout')
            self.assertIn('故障码读取失败', journal.snapshot()[0][-1]['description'])
            self.assertIsNone(journal.snapshot()[0][-1]['code'])

    def test_disk_failure_keeps_ui_history(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = FaultJournal(directory)  # directory cannot be opened as a log file
            journal.observe('left', [1,0,0,0,0,0])
            events, error = journal.snapshot()
            self.assertEqual(len(events), 1)
            self.assertIn('日志写入失败', error)


if __name__ == '__main__': unittest.main()
