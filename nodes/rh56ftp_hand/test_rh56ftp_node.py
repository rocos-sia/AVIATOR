import json
import importlib.util
import multiprocessing
import subprocess
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).with_name("rh56ftp_node.py")
spec = importlib.util.spec_from_file_location("rh56ftp_node", SCRIPT)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def command(node, sequence=1, valid=True):
    mono = node.clock_now
    return {
        "msg_type": "HandCommand", "version": "1.0", "sequence": sequence,
        "timestamp": 1, "sample_mono_us": mono, "clock_id": node.clock_id,
        "publisher_id": "test", "session_id": "11111111-1111-4111-8111-111111111111",
        "control_epoch": "22222222-2222-4222-8222-222222222222", "valid": valid,
        "mode": "NORMALIZED_POSITION",
        "origin": {"publisher_id": "test", "session_id": "11111111-1111-4111-8111-111111111111",
                    "sequence": sequence, "sample_mono_us": mono, "clock_id": node.clock_id},
        "hands": {"left": {"drive_position_normalized": [0, .1, .2, .3, .4, .5]},
                  "right": {"drive_position_normalized": [1, .9, .8, .7, .6, .5]}},
    }


class FakeLink:
    def __init__(self, *_args):
        self.writes = []
        self.setting_writes = []
        self.events = []
        self.fail_writes = False
        self.fail_force = False

    def connect(self):
        pass

    def close(self):
        pass

    def write_angle_set(self, values):
        if self.fail_writes:
            raise RuntimeError("write failed")
        self.writes.append(list(values))
        self.events.append(("angle", list(values)))

    def write_speed_set(self, values):
        self.setting_writes.append(("speed", list(values)))
        self.events.append(("speed", list(values)))

    def write_force_set(self, values):
        if self.fail_force:
            raise RuntimeError("force write failed")
        self.setting_writes.append(("force", list(values)))
        self.events.append(("force", list(values)))

    def read_state(self):
        return {"angle": [10, 20, 30, 40, 50, 60], "force": [1] * 6,
                "current": [2] * 6, "err": [0] * 6, "status": [3] * 6,
                "temp": [35] * 6}


def private_bus(connection):
    import zmq
    context = zmq.Context()
    ingress = context.socket(zmq.XSUB)
    egress = context.socket(zmq.XPUB)
    input_port = ingress.bind_to_random_port("tcp://127.0.0.1")
    output_port = egress.bind_to_random_port("tcp://127.0.0.1")
    connection.send((f"tcp://127.0.0.1:{input_port}", f"tcp://127.0.0.1:{output_port}"))
    connection.close()
    zmq.proxy(ingress, egress)


def private_node(ingress, egress):
    # Exercise CLI endpoint semantics and the real node loop without hardware.
    with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)):
        module.main(["--right-host", "fake-right", "--left-host", "fake-left",
                     "--endpoint", egress, "--state-endpoint", ingress])


class NodeTests(unittest.TestCase):
    def make_node(self):
        links = {"left": FakeLink(), "right": FakeLink()}
        node = module.Rh56FtpNode(links, clock_id="test-clock",
                                   session_id="33333333-3333-4333-8333-333333333333")
        node.clock_now = 1000000
        return node, links

    def test_reorder_and_command_write(self):
        node, links = self.make_node()
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertEqual(links["right"].writes[-1], [500, 600, 700, 800, 900, 1000])
        self.assertEqual(links["left"].writes[-1], [500, 400, 300, 200, 100, 0])

    def test_default_settings_precede_startup_pose_and_are_cached(self):
        node, links = self.make_node()
        node.connect()
        node._safe_pose()
        for link in links.values():
            self.assertEqual(link.events, [("speed", [500] * 6),
                                           ("force", [500] * 6),
                                           ("angle", [1000] * 6)])
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        node.supervise(now=node.clock_now + node.command_timeout_us)
        node._safe_pose()  # Exit pose uses the same settings.
        for link in links.values():
            self.assertEqual(len(link.setting_writes), 2)

    def test_setting_failure_blocks_angles_and_retries(self):
        node, links = self.make_node()
        links["right"].fail_force = True
        with self.assertRaisesRegex(RuntimeError, "right speed/force setup failed"):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertFalse(node.guard.authorized)
        self.assertFalse(node.command_valid)
        for link in links.values():
            self.assertEqual(link.writes, [])
        links["right"].fail_force = False
        node.supervise(now=node.clock_now)
        for link in links.values():
            self.assertEqual(link.writes, [[1000] * 6])
            self.assertIn(("force", [500] * 6), link.setting_writes)

    def test_angle_write_failure_reapplies_settings(self):
        node, links = self.make_node()
        node._safe_pose()
        links["left"].fail_writes = True
        with self.assertRaisesRegex(RuntimeError, "write failed"):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        links["left"].fail_writes = False
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertEqual(len(links["left"].setting_writes), 4)
        self.assertEqual(len(links["right"].setting_writes), 2)

    def test_feedback_only_does_not_write_registers(self):
        links = {"left": FakeLink(), "right": FakeLink()}
        node = module.Rh56FtpNode(links, feedback_only=True, clock_id="test-clock")
        node.clock_now = 1000000
        node.connect()
        self.assertFalse(node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now))
        node.supervise(now=node.clock_now)
        node.read_states(now=node.clock_now)
        node.close()
        for link in links.values():
            self.assertEqual(link.events, [])

    def test_custom_cli_settings_on_single_hand(self):
        with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                patch.object(module, "run_node", return_value=0) as run:
            self.assertEqual(module.main(["--right-host", "", "--left-host", "fake-left",
                                          "--speed", "250", "--force", "1200"]), 0)
        node = run.call_args.args[0]
        node._safe_pose()
        self.assertIsNone(node.links["right"])
        self.assertEqual(node.links["left"].setting_writes,
                         [("speed", [250] * 6), ("force", [1200] * 6)])

    def test_invalid_settings_rejected_before_opening_links(self):
        for option, value in (("--speed", "-1"), ("--speed", "1001"),
                              ("--force", "-1"), ("--force", "3001"), ("--force", "1.5")):
            with self.subTest(option=option, value=value), \
                    patch.object(module, "load_handlink") as load, \
                    patch("sys.stderr"), self.assertRaises(SystemExit):
                module.main([option, value])
            load.assert_not_called()
        for settings in ({"speed": True}, {"speed": 1.5}, {"force": 3001}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                module.Rh56FtpNode({"left": None, "right": FakeLink()}, **settings)

    def test_state_has_diagnostics_without_touch(self):
        node, _links = self.make_node()
        node.read_states(now=1000000)
        state = node.make_state(now=1000000)
        self.assertTrue(state["valid"])
        self.assertEqual(state["hands"]["right"]["drive_position_raw"], [60, 50, 40, 30, 20, 10])
        self.assertEqual(state["hands"]["right"]["temperature"], [35] * 6)
        self.assertNotIn("touch", json.dumps(state).lower())

    def test_grasp_setpoint_maps_to_opening_counts(self):
        node, links = self.make_node()
        msg = command(node)
        msg["mode"] = "GRASP_SETPOINT"
        msg["hands"] = {side: {"grasp": {"closure": .25}} for side in ("left", "right")}
        node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertEqual(links["right"].writes[-1], [750] * 6)

    def test_invalidates_on_timeout(self):
        node, links = self.make_node()
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertFalse(node.command_valid)
        self.assertEqual(links["left"].writes[-1], [1000] * 6)

    def test_failed_safe_pose_is_retried(self):
        node, links = self.make_node()
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        links["left"].fail_writes = True
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertTrue(node.command_valid)
        links["left"].fail_writes = False
        node.supervise(now=node.clock_now + node.command_timeout_us + 1)
        self.assertFalse(node.command_valid)

    def test_default_endpoints_follow_bus_direction(self):
        with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                patch.object(module, "run_node", return_value=0) as run:
            self.assertEqual(module.main([]), 0)
        _node, subscribe_endpoint, publish_endpoint, _hz = run.call_args.args
        self.assertEqual(subscribe_endpoint, "tcp://127.0.0.1:5556")
        self.assertEqual(publish_endpoint, "tcp://127.0.0.1:5555")

    @unittest.skipUnless(importlib.util.find_spec("zmq"), "pyzmq required for private bus integration")
    def test_manual_publisher_ack_through_private_bus(self):
        import zmq
        processes = multiprocessing.get_context("spawn")
        receiver, sender = processes.Pipe(duplex=False)
        bus = processes.Process(target=private_bus, args=(sender,))
        node = None
        publisher = None
        context = zmq.Context()
        observer = context.socket(zmq.SUB)
        observer.subscribe(b"hand.state")
        bus.start()
        sender.close()
        try:
            self.assertTrue(receiver.poll(3), "private bus did not start")
            ingress, egress = receiver.recv()
            observer.connect(egress)
            node = processes.Process(target=private_node, args=(ingress, egress))
            node.start()
            self.assertTrue(observer.poll(3000), "node state did not reach bus egress")
            observer.recv_multipart()
            script = SCRIPT.parent.parent / "aviator_hand" / "hand_command.py"
            publisher = subprocess.Popen(
                [sys.executable, str(script), "--pose", "half", "--duration", "0.6",
                 "--endpoint", ingress, "--state-endpoint", egress],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            accepted = []
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and publisher.poll() is None:
                if observer.poll(50):
                    state = json.loads(observer.recv_multipart()[1])
                    if state["command_valid"]:
                        accepted.append(state)
            stdout, stderr = publisher.communicate(timeout=1)
            self.assertEqual(publisher.returncode, 0, stderr)
            self.assertIn("已接受", stdout)
            self.assertTrue(accepted, "node never acknowledged a command through the bus")
            for state in accepted:
                self.assertEqual(state["accepted_command"]["publisher_id"], "hand_manual_test")
                for side in module.SIDES:
                    self.assertEqual(state["hands"][side]["commanded_drive_position_normalized"], [.5] * 6)
        finally:
            if publisher is not None and publisher.poll() is None:
                publisher.kill()
                publisher.communicate()
            for process in (node, bus):
                if process is not None:
                    if process.is_alive():
                        process.terminate()
                    process.join(timeout=3)
                    if process.is_alive():
                        process.kill()
                        process.join(timeout=1)
            receiver.close()
            observer.close(0)
            context.term()


if __name__ == "__main__":
    unittest.main()
