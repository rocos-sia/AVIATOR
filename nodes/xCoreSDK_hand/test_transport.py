import ctypes
from pathlib import Path
import sys
import threading
import time
import unittest
import tempfile
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from transport import HandLink, SdkConnection
from main import SdkHandNode
from rh56ftp_node import Rh56FtpNode
import rh56ftp_node
import main as sdk_node

FAKE_LIBRARY = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else None

class Connection:
    host = 'offline-controller'
    def __init__(self):
        self.sent = []
        self.bad = False
        self.closed = False
    def connect(self): pass
    def transfer(self, tx, count):
        self.sent.append(tx)
        self.assert_frame(tx)
        data = bytes([1]) if tx[4] == 0x12 else bytes(range(count - 8))
        rx = bytes([0x90,0xeb,tx[2],len(data)+3,*tx[4:7]]) + data
        return rx + bytes([(sum(rx[2:]) + self.bad) & 255])
    def assert_frame(self, tx):
        assert tx[:2] == b'\xeb\x90' and tx[-1] == sum(tx[2:-1]) & 255
        assert len(tx) == tx[3] + 5

class Tests(unittest.TestCase):
    def test_frames_and_same_handlink_contract(self):
        c = Connection(); h = HandLink(c, 2); h.connect()
        s = h.read_state()
        self.assertEqual(s['angle'], [256,770,1284,1798,2312,2826])
        self.assertEqual(s['current'], [3340,3854,4368,4882,5396,5910])
        self.assertEqual(s['err'], list(range(24,30)))
        self.assertEqual(s['temp'], list(range(36,42)))
        c.sent.clear(); h.write_angle_set([None,-1,None,42,None,None])
        self.assertEqual(len(c.sent),2)
        self.assertEqual(c.sent[0][5:9], bytes([0xd0,5,255,255]))
        self.assertEqual(c.sent[1][5:9], bytes([0xd4,5,42,0]))
        h.write_mode_set([0]*6); h.write_speed_set([500]*6); h.write_force_set([1000]*6)
        c.bad=True
        with self.assertRaisesRegex(RuntimeError,'register=1546'): h.read_state()
        c.bad=False
        self.assertEqual(len(h.read_state()['angle']),6)
        h.close()
        self.assertFalse(c.closed)
        with self.assertRaisesRegex(RuntimeError,'closed'): h.read_state()

    def test_fault_read_is_six_bytes_at_1606(self):
        c = Connection(); h = HandLink(c, 2); h.connect()
        self.assertEqual(h.read_errors(), {'err': [0,1,2,3,4,5]})
        self.assertEqual(c.sent[0][4:8], bytes([0x11, 0x46, 0x06, 6]))
        c.bad = True
        with self.assertRaisesRegex(RuntimeError, 'register=1606'):
            h.read_errors()

    def test_shared_feedback_and_safety_with_event_policy(self):
        from policy import EventHandNode
        for method in ('handle_command', 'read_states'):
            self.assertIs(getattr(SdkHandNode,method),getattr(Rh56FtpNode,method))
        self.assertIs(SdkHandNode.supervise, EventHandNode.supervise)

    def test_pause_is_single_byte_register_1634(self):
        c = Connection(); h = HandLink(c, 2); h.connect()
        h.write_pause()
        self.assertEqual(c.sent, [bytes.fromhex('eb 90 02 04 12 62 06 01 81')])
        c.bad = True
        with self.assertRaisesRegex(RuntimeError, 'register=1634'):
            h.write_pause()

    def test_shared_configuration_and_overrides_without_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            hand = root / 'rh56ftp_hand.yaml'
            hand.write_text('speed: {grasp: 321, hold: 123}\nforce: {grasp: 987, hold: 98}\n'
                            'threshold: 17\nclosing_hold_ms: 8000\ncommand_timeout_ms: 120\n'
                            'feedback_timeout_ms: 600\nstate_period_ms: 80\n'
                            'position_write_hz: 20\nposition_read_hz: 10\ntelemetry_read_hz: 5\n'
                            'right_host: modbus-right\nleft_host: modbus-left\nleft_id: 2\n')
            (root / 'robot.yaml').write_text('rokae: {left_ip: sdk-left, right_ip: sdk-right}\n')
            system = root / 'system.yaml'
            system.write_text('robot: robot.yaml\nbus: {publish: tcp://127.0.0.1:5555, '
                              'subscribe: tcp://127.0.0.1:5556}\n'
                              'xcore_sdk_hand: {config: rh56ftp_hand.yaml}\n')
            captured = []
            def capture(node, sub, pub, hz, settings=None):
                captured.append((node, sub, pub, hz))
                return 0
            with patch.object(rh56ftp_node, 'run_node', side_effect=capture), \
                 patch.object(sdk_node, 'run_scheduled', side_effect=capture), \
                 patch.object(sdk_node, 'prepare_sdk_directory'):
                rh56ftp_node.main(['--config',str(hand),'--speed','456'], simulated=True)
                sdk_node.main(['--system-config',str(system),'--sdk-library','unused.so','--speed','456'])
                sdk_node.main(['--system-config',str(system),'--fake-hand','--speed','456'])
            reference = captured[0][0]
            for node, sub, pub, hz in captured:
                for name in ('speed','force','hold_speed','hold_force','closing_hold_us',
                             'closing_motion_raw','command_timeout_us','feedback_timeout_us'):
                    self.assertEqual(getattr(node,name),getattr(reference,name),name)
                self.assertEqual(node.speed,456)
                self.assertEqual(hz,12.5)
            real = captured[1][0]
            self.assertEqual(real.links['left'].host,'sdk-left')
            self.assertEqual(real.links['left'].hand_id,2)
            self.assertIs(real.links,real.read_links)
            self.assertEqual(real.make_state()['hands']['left']['sample_time_basis'],'host_rs485_read')

    @unittest.skipUnless(FAKE_LIBRARY,'requires compiled fake SDK transport')
    def test_native_api_and_process_owned_sdk_lifetime(self):
        c = SdkConnection(FAKE_LIBRARY,'offline',50)
        h = HandLink(c); h.connect(); h.connect()
        h.write_speed_set([500]*6)
        self.assertEqual(h.read_state()['angle'],[0]*6)
        h.close()  # fake SDK destructor aborts if it is ever invoked
        self.assertTrue(c.handle)
        # Same per-port queue supports reads/writes from separate threads.
        h.connect()
        errors=[]
        def work():
            try:
                for _ in range(10): h.read_state(); h.write_angle_set([400]*6)
            except Exception as e: errors.append(e)
        workers=[threading.Thread(target=work) for _ in range(2)]
        for t in workers:t.start()
        for t in workers:t.join(2);self.assertFalse(t.is_alive())
        self.assertFalse(errors)

if __name__ == '__main__': unittest.main()
