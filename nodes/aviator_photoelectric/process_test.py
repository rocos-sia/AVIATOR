"""Exercise the real node through a PTY and private ZMQ endpoint; no hardware."""
import copy
import os
import pathlib
import pty
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import yaml
import zmq

BINARY, CONFIG = sys.argv[1:3]
sys.argv = sys.argv[:1]


def frame(payload):
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return bytes(payload) + crc.to_bytes(2, "little")


class NodeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = pathlib.Path(self.temp.name)
        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"photoelectric.state")
        port = self.sub.bind_to_random_port("tcp://127.0.0.1")
        self.cfg = yaml.safe_load(pathlib.Path(CONFIG).read_text())
        self.link = self.directory / "device"
        self.master, self.slave = pty.openpty()
        self.link.symlink_to(os.ttyname(self.slave))
        self.cfg["serial"]["device"] = str(self.link)
        self.cfg["publish_endpoint"] = f"tcp://127.0.0.1:{port}"
        self.cfg["modbus"]["reconnect_interval_ms"] = 150
        self.mode = "zero"
        self.requests = []
        self.errors = []
        self.running = True
        self.proc = None
        self.thread = threading.Thread(target=self.device, daemon=True)
        self.thread.start()

    def tearDown(self):
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                self.fail("node did not stop promptly")
        self.running = False
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)
        self.sub.close(linger=0)
        self.ctx.term()
        self.temp.cleanup()

    def device(self):
        buffer = b""
        while self.running:
            try:
                if not select.select([self.master], [], [], 0.02)[0]:
                    continue
                buffer += os.read(self.master, 1024)
                while len(buffer) >= 8:
                    request, buffer = buffer[:8], buffer[8:]
                    self.requests.append(request)
                    if request != bytes.fromhex("01 02 00 00 00 01 B9 CA"):
                        self.errors.append(f"unexpected request: {request.hex()}")
                        continue
                    mode = self.mode
                    if mode == "silent":
                        continue
                    response = frame([1, 2, 1, int(mode in ("one", "fragment"))])
                    if mode == "crc":
                        response = response[:-1] + bytes([response[-1] ^ 1])
                    elif mode == "address":
                        response = frame([2, 2, 1, 1])
                    elif mode == "function":
                        response = frame([1, 1, 1, 1])
                    elif mode == "exception":
                        response = frame([1, 0x82, 2])
                    elif mode == "length":
                        response = frame([1, 2, 2, 1, 0])
                    if mode == "fragment":
                        os.write(self.master, response[:2])
                        time.sleep(0.005)
                        os.write(self.master, response[2:])
                    else:
                        os.write(self.master, response)
            except OSError:
                time.sleep(0.01)

    def start(self):
        path = self.directory / "config.yaml"
        path.write_text(yaml.safe_dump(self.cfg))
        self.proc = subprocess.Popen([BINARY, "--config", str(path)],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def receive(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.sub.poll(100):
                topic, payload = self.sub.recv_multipart()
                import json
                msg = json.loads(payload)
                self.assertEqual(topic, b"photoelectric.state")
                self.assertEqual(msg["msg_type"], "PhotoelectricState")
                self.assertIn(msg["value"], (0, 1))
                self.assertEqual(msg["channel"], "X1")
                if predicate(msg):
                    return msg
            self.assertIsNone(self.proc.poll(), "node exited unexpectedly")
        self.fail("no matching publication")

    def test_signal_faults_and_recovery(self):
        self.start()
        initial = self.receive(lambda m: m["valid"] and m["value"] == 0)
        messages = [self.receive(lambda m: m["valid"] and m["value"] == 0) for _ in range(8)]
        self.assertGreater(messages[-1]["sequence"], initial["sequence"])
        self.assertGreater(messages[-1]["sample_mono_us"], initial["sample_mono_us"])
        gaps = [b["timestamp"] - a["timestamp"] for a, b in zip(messages, messages[1:])]
        self.assertTrue(all(25000 < gap < 100000 for gap in gaps), gaps)
        self.mode = "fragment"
        self.receive(lambda m: m["valid"] and m["value"] == 1)
        for mode, error in [("crc", "CRC"), ("address", "address/function"),
                            ("function", "address/function"), ("exception", "exception 2"),
                            ("length", "length"), ("silent", "timeout")]:
            self.mode = mode
            failed = self.receive(lambda m: not m["valid"] and error in m["status"])
            later = self.receive(lambda m: not m["valid"] and m["sequence"] >= failed["sequence"] + 3)
            self.assertEqual(later["sample_mono_us"], failed["sample_mono_us"])
            self.assertEqual(later["value"], 1)
            self.mode = "one"
            recovered = self.receive(lambda m: m["valid"] and m["sample_mono_us"] > failed["sample_mono_us"])
            self.assertEqual(recovered["value"], 1)
        self.mode = "zero"
        self.receive(lambda m: m["valid"] and m["value"] == 0)
        self.assertFalse(self.errors, self.errors)
        self.assertGreater(len(self.requests), 10)
        self.proc.terminate()
        self.assertEqual(self.proc.wait(timeout=2), 0)

    def test_missing_device_and_reconnect(self):
        self.link.unlink()
        self.start()
        missing = self.receive(lambda m: not m["valid"] and m["status"].startswith("open:"))
        self.assertEqual(missing["sample_mono_us"], 0)
        self.link.symlink_to(os.ttyname(self.slave))
        self.receive(lambda m: m["valid"])
        # Replace the adapter with a new PTY while retaining its configured path.
        self.running = False
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)
        self.master, self.slave = pty.openpty()
        self.receive(lambda m: not m["valid"])
        self.link.unlink()
        self.link.symlink_to(os.ttyname(self.slave))
        self.mode = "one"
        self.running = True
        self.thread = threading.Thread(target=self.device, daemon=True)
        self.thread.start()
        self.receive(lambda m: m["valid"] and m["value"] == 1)

    def test_invert(self):
        self.cfg["invert"] = True
        self.start()
        self.receive(lambda m: m["valid"] and m["value"] == 1)
        self.mode = "one"
        self.receive(lambda m: m["valid"] and m["value"] == 0)

    def test_invalid_config(self):
        for section, key, value in [("serial", "baud_rate", 12345), ("serial", "data_bits", 7),
                                     ("modbus", "slave_address", 0), ("modbus", "input_address", -1),
                                     (None, "poll_hz", 0), (None, "stale_timeout_ms", 10),
                                     (None, "unknown", 5)]:
            cfg = copy.deepcopy(self.cfg)
            (cfg[section] if section else cfg)[key] = value
            path = self.directory / "invalid.yaml"
            path.write_text(yaml.safe_dump(cfg))
            result = subprocess.run([BINARY, "--config", str(path), "--check-config"],
                                    capture_output=True, timeout=2)
            self.assertNotEqual(result.returncode, 0, (key, value))


if __name__ == "__main__":
    unittest.main()
