#!/usr/bin/env python3
"""Offline framing/statistics and real PTY tests; never open configured hardware."""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import shutil
import signal
import struct
import subprocess
import tempfile
import termios
import time
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rs422_loss', ROOT / 'scripts/test_rs422_loss.py')
loss = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loss)


def frame(counter, operation=0, roll=0, pitch=0):
    data = bytearray(b'\xeb\x90' + bytes(30))
    data[2], data[3] = counter & 255, operation
    struct.pack_into('<hh', data, 4, roll, pitch)
    data[31] = sum(data[2:31]) & 255
    return bytes(data)


class ReceiverTest(unittest.TestCase):
    def test_noise_split_header_bad_checksum_and_recovery(self):
        receiver = loss.Receiver()
        bad = bytearray(frame(255))
        bad[-1] ^= 1
        data = b'noise' + frame(254) + bad + frame(0) + frame(1)[:17]
        for i, byte in enumerate(data):
            receiver.feed(bytes([byte]), i / 1000)
        result = receiver.summary(4, 1)
        self.assertEqual(result['valid_frames'], 2)
        self.assertEqual(result['checksum_failures'], 1)
        self.assertEqual(result['discarded_bytes'], 37)
        self.assertEqual(result['trailing_partial_bytes'], 17)
        self.assertEqual(result['counter_missing_estimate'], 1)
        receiver.feed(frame(1)[17:], .2)
        self.assertEqual(receiver.valid_frames, 3)

    def test_wrap_gaps_duplicate_and_restart_ambiguity(self):
        receiver = loss.Receiver()
        for i, value in enumerate((254, 255, 0, 1, 4, 4, 2, 3)):
            receiver.feed(frame(value), i * .0125)
        self.assertEqual(receiver.counter_missing_estimate, 2)
        self.assertEqual(receiver.same_counter_events, 1)
        self.assertEqual(receiver.ambiguous_events, 1)
        receiver.feed(frame(4), 4)  # Even delta=1 cannot exclude complete lost wraps.
        self.assertEqual(receiver.ambiguous_events, 2)
        self.assertEqual(receiver.counter_missing_estimate, 2)

    def test_business_validation_does_not_hide_received_envelopes(self):
        receiver = loss.Receiver()
        receiver.feed(frame(0, operation=0x70) + frame(1, roll=1001) + frame(2), 0)
        result = receiver.summary(2, 1)
        self.assertEqual(result['valid_frames'], 3)
        self.assertEqual(result['semantic_invalid_frames'], 2)
        self.assertEqual(result['count_shortfall'], 0)
        self.assertEqual(result['count_excess'], 1)
        self.assertEqual(result['reception_percent'], 150)

    def test_bounded_events_and_buffer(self):
        receiver = loss.Receiver()
        receiver.feed(b'garbage' * 10000 + frame(3) * 1200, 0)
        result = receiver.summary(1200, 1)
        self.assertEqual(result['omitted_events'], 199)
        self.assertEqual(len(result['events']), 1000)
        self.assertEqual(len(receiver.buffer), 0)


class ProcessTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='aviator rs422 test ')
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.resource = self.directory / 'source'
        (self.resource / 'scripts').mkdir(parents=True)
        (self.resource / 'config').mkdir()
        for name in ('test_rs422_loss.py', 'test_rs422_loss.sh'):
            shutil.copy2(ROOT / 'scripts' / name, self.resource / 'scripts' / name)
        self.config = yaml.safe_load((ROOT / 'config/flight-rs422.yaml').read_text())
        self.config_path = self.resource / 'config/flight-rs422.yaml'
        self.config['rs422'].update(channel_a='/missing/serial-a', channel_b='/missing/serial-b')
        self.save_config()
        self.report = self.directory / 'result.json'

    def save_config(self):
        self.config_path.write_text(yaml.safe_dump(self.config))

    def command(self, *args):
        return [str(self.resource / 'scripts/test_rs422_loss.sh'), *args]

    def make_ports(self):
        ports = [pty.openpty(), pty.openpty()]
        self.port_fds = {fd for pair in ports for fd in pair}
        def close_ports():
            for fd in self.port_fds:
                os.close(fd)
        self.addCleanup(close_ports)
        for name, (master, slave) in zip(('a', 'b'), ports):
            self.config['rs422']['channel_' + name] = os.ttyname(slave)
            os.set_blocking(master, False)
        self.save_config()
        return ports

    def start(self):
        self.output_path = self.directory / 'output.log'
        output = self.output_path.open('w')
        self.addCleanup(output.close)
        process = subprocess.Popen(self.command('--report', str(self.report)), cwd='/tmp',
                                   stdout=output, stderr=subprocess.STDOUT)
        def cleanup():
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        self.addCleanup(cleanup)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            text = self.output_path.read_text()
            if '接收就绪' in text:
                return process
            if process.poll() is not None:
                self.fail(text)
            time.sleep(.01)
        self.fail('receiver did not become ready')

    def send(self, fd, data):
        deadline = time.monotonic() + 10
        while data:
            self.assertLess(time.monotonic(), deadline, 'PTY write timed out')
            if select.select([], [fd], [], .1)[1]:
                try:
                    size = os.write(fd, data[:4096])
                    data = data[size:]
                except BlockingIOError:
                    pass

    def test_10000_frames_two_ports_loss_and_ctrl_c(self):
        (master_a, slave_a), (master_b, _) = self.make_ports()
        original = termios.tcgetattr(slave_a)
        process = self.start()
        missing = {0, 101, 255, 256, 9999}
        for start in range(0, 10000, 100):
            a, b = bytearray(), bytearray()
            for index in range(start, start + 100):
                packet = bytearray(frame(index))
                b.extend(packet)
                if index not in missing:
                    if index == 500:
                        packet[-1] ^= 1
                    a.extend(packet)
            self.send(master_a, a)
            self.send(master_b, b)
        time.sleep(.25)  # Longer than the configured 50ms; must not stop or reset counters.
        self.assertIsNone(process.poll())  # B already reached --expected, also not a stop condition.
        process.send_signal(signal.SIGINT)
        self.assertEqual(process.wait(timeout=5), 0, self.output_path.read_text())
        report = json.loads(self.report.read_text())
        a, b = report['channels']['A'], report['channels']['B']
        self.assertEqual((a['valid_frames'], a['count_shortfall']), (9994, 6))
        self.assertEqual(a['counter_missing_estimate'], 4)  # Leading/trailing losses require total count.
        self.assertEqual(a['checksum_failures'], 1)
        self.assertEqual((b['valid_frames'], b['count_shortfall']), (10000, 0))
        self.assertEqual(b['counter_missing_estimate'], 0)
        self.assertEqual(report['stop_reason'], 'SIGINT')
        self.assertIsNone(report['error'])
        self.assertEqual(termios.tcgetattr(slave_a), original)
        # The read-only test must never reply on the serial connection.
        self.assertFalse(select.select([master_a, master_b], [], [], 0)[0])

    def test_gateway_lock_conflict_changes_neither_port(self):
        (_, slave_a), (_, slave_b) = self.make_ports()
        original = termios.tcgetattr(slave_a)
        held = os.open(os.ttyname(slave_b), os.O_RDWR | os.O_NOCTTY)
        self.addCleanup(os.close, held)
        fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = subprocess.run(self.command('--report', str(self.report)), capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 1)
        self.assertIn('串口已被占用', result.stderr)
        self.assertEqual(termios.tcgetattr(slave_a), original)
        self.assertFalse(json.loads(self.report.read_text())['ready'])
        # A's lock was released after B failed.
        probe = loss.SerialPort(os.ttyname(slave_a)).open()
        probe.close()

    def test_serial_settings_match_gateway_8e1_without_flow_control(self):
        (_, slave), _ = self.make_ports()
        original = termios.tcgetattr(slave)
        original[0] |= termios.IXON | termios.IXOFF | termios.IXANY | termios.IGNPAR
        original[2] |= loss.CMSPAR | termios.PARODD | termios.CSTOPB | termios.CRTSCTS
        port = loss.SerialPort(os.ttyname(slave))
        port.fd = slave
        with patch.object(termios, 'tcgetattr', return_value=original), \
                patch.object(termios, 'tcsetattr') as setattrs:
            port.configure()
        attrs = setattrs.call_args.args[2]
        self.assertEqual(attrs[4:6], [termios.B115200, termios.B115200])
        self.assertEqual(attrs[2] & termios.CSIZE, termios.CS8)
        self.assertTrue(attrs[2] & termios.PARENB)
        self.assertFalse(attrs[2] & (loss.CMSPAR | termios.PARODD | termios.CSTOPB | termios.CRTSCTS))
        self.assertTrue(attrs[0] & termios.INPCK)
        self.assertFalse(attrs[0] & (termios.IXON | termios.IXOFF | termios.IXANY | termios.IGNPAR))

    def test_disconnect_preserves_partial_report(self):
        (master_a, _), _ = self.make_ports()
        process = self.start()
        self.send(master_a, frame(12))
        time.sleep(.1)
        os.close(master_a)
        self.port_fds.remove(master_a)
        self.assertEqual(process.wait(timeout=5), 1, self.output_path.read_text())
        report = json.loads(self.report.read_text())
        self.assertEqual(report['channels']['A']['valid_frames'], 1)
        self.assertEqual(report['stop_reason'], 'error')

    def test_dry_run_no_device_access_and_invalid_configuration(self):
        with patch.object(loss.os, 'open', side_effect=AssertionError('opened device')), \
                patch.object(loss, 'CONFIG', self.config_path):
            self.assertEqual(loss.main(['--dry-run', '--report', str(self.report)]), 0)
        self.assertFalse(self.report.exists())
        for key, value in (('profile', 'old'), ('channel_a', 'relative'), ('receive_timeout_ms', 5)):
            with self.subTest(key=key):
                modified = yaml.safe_load(self.config_path.read_text())
                modified['rs422'][key] = value
                self.config_path.write_text(yaml.safe_dump(modified))
                with self.assertRaises(ValueError):
                    loss.load_config(self.config_path)
                self.save_config()
        with self.config_path.open('a') as stream:
            stream.write('\nrs422: {}\n')
        with self.assertRaisesRegex(ValueError, '重复'):
            loss.load_config(self.config_path)

    def test_deployed_launcher_uses_bundled_python_and_field_config(self):
        prefix = self.directory / 'installed'
        installed = prefix / 'share/aviator'
        shutil.copytree(self.resource, installed)
        (prefix / 'release-version.json').write_text('{}')
        executable = prefix / 'cache/python/bin/python'
        executable.parent.mkdir(parents=True)
        # A forwarding stub verifies selection of the deployed interpreter.
        executable.write_text('#!/bin/sh\necho BUNDLED_PYTHON\nexec /usr/bin/python3 "$@"\n')
        executable.chmod(0o755)
        result = subprocess.run([str(installed / 'scripts/test_rs422_loss.sh'), '--dry-run'],
                                cwd='/tmp', capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('BUNDLED_PYTHON', result.stdout)
        self.assertIn(str(installed / 'config/flight-rs422.yaml'), result.stdout)
        self.assertIn('/missing/serial-a', result.stdout)
        executable.unlink()
        result = subprocess.run([str(installed / 'scripts/test_rs422_loss.sh'), '--dry-run'],
                                capture_output=True, text=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('部署 Python 不存在', result.stderr)


if __name__ == '__main__':
    unittest.main()
