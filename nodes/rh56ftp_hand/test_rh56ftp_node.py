import json
import io
import importlib.util
import multiprocessing
import subprocess
import sys
import time
import unittest
from dataclasses import replace
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
    def test_text_instance_restart_keeps_sequence_and_epoch_checks(self):
        node, _ = self.make_node()
        first = command(node, sequence=10)
        guard = module.CommandGuard()
        guard.accept(first, node.clock_now, node.command_timeout_us, node.clock_id)
        node.clock_now += 1
        restarted = command(node)
        restarted["session_id"] = "restarted-core"
        restarted["origin"]["session_id"] = "unconfigured-origin"
        module.decode_command("hand.command", json.dumps(restarted))
        guard.accept(restarted, node.clock_now, node.command_timeout_us, node.clock_id)
        with self.assertRaises(ValueError):
            guard.accept(restarted, node.clock_now, node.command_timeout_us, node.clock_id)
        with self.assertRaises(ValueError):
            guard.accept(first, node.clock_now, node.command_timeout_us, node.clock_id)
        changed = dict(restarted, sequence=2, control_epoch="another-epoch")
        with self.assertRaises(PermissionError):
            guard.accept(changed, node.clock_now, node.command_timeout_us, node.clock_id)

    def make_node(self):
        links = {"left": FakeLink(), "right": FakeLink()}
        node = module.Rh56FtpNode(links, clock_id="test-clock",
                                   session_id="33333333-3333-4333-8333-333333333333")
        node.clock_now = 1000000
        return node, links

    def drive_closing(self, node, *, duration_ms=5100, target=None, position=None, feedback=None):
        target = target or [0] * 6
        start = node.clock_now
        for elapsed_ms in range(0, duration_ms + 1, 20):
            node.clock_now = start + elapsed_ms * 1000
            if elapsed_ms % 60 == 0:
                for side in module.SIDES:
                    previous = node.snapshots[side]
                    available = feedback is None or feedback(elapsed_ms)
                    values = position(elapsed_ms, side) if position else [400] * 6
                    data = FakeLink().read_state()
                    data["angle"] = module.canonical_to_rh(values)
                    node.snapshots[side] = replace(
                        previous, data=data if available else previous.data,
                        sample_mono_us=node.clock_now if available else previous.sample_mono_us,
                        error="" if available else "read failed",
                        errors=previous.errors + int(not available),
                        reads=previous.reads + int(available))
            msg = command(node, sequence=node.guard.last_sequence + 1)
            for side in module.SIDES:
                msg["hands"][side]["drive_position_normalized"] = list(target)
            node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)

    def test_closing_hold_only_affects_five_bending_channels_and_keeps_ack_fresh(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=4980)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        # Continue fresh commands and feedback until the full window is covered.
        self.drive_closing(node, duration_ms=120)
        self.assertEqual(links["left"].writes[-1], [400] * 5 + [0])
        self.assertEqual(node.guard.last_sequence, node.command_accepted)
        self.assertEqual(node.guard.sample_mono_us, node.clock_now)
        self.assertTrue(node.command_valid)
        state = node.make_state(now=node.clock_now)
        for side in module.SIDES:
            hand = state["hands"][side]
            self.assertEqual(hand["requested_drive_position_normalized"], [0] * 6)
            self.assertEqual(hand["commanded_drive_position_normalized"], [0] + [.4] * 5)
            self.assertEqual(hand["closing_hold_active"], [False] + [True] * 5)
            self.assertEqual(hand["drive_position_normalized"], [.4] * 6)
        # A new position reading must not make a latched target drift.
        self.drive_closing(node, duration_ms=60, position=lambda _ms, _side: [450] * 6)
        self.assertEqual(links["left"].writes[-1], [400] * 5 + [0])

    def test_opening_and_new_targets_release_hold_immediately(self):
        node, links = self.make_node()
        self.drive_closing(node)
        self.drive_closing(node, duration_ms=0, target=[1] * 6)
        self.assertEqual(links["left"].writes[-1], [1000] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
        # A new, tighter target starts a new observation window too.
        self.drive_closing(node, target=[.2] * 6)
        self.assertEqual(links["left"].writes[-1], [400] * 5 + [200])
        self.drive_closing(node, duration_ms=0, target=[.1] * 6)
        self.assertEqual(links["left"].writes[-1], [100] * 6)

    def test_moving_fingers_continue_while_stalled_fingers_hold_independently(self):
        node, links = self.make_node()
        self.drive_closing(node, position=lambda ms, side: (
            [400, 400, 700 - ms // 20, 400, 400, 400] if side == "left" else [400] * 6))
        self.assertEqual(links["left"].writes[-1], [400, 400, 400, 0, 400, 0])
        self.assertEqual(links["right"].writes[-1], [400] * 5 + [0])

    def test_motion_window_uses_range_not_only_start_end_displacement(self):
        node, links = self.make_node()
        self.drive_closing(node, position=lambda ms, _side: [400 if ms < 2000 or ms > 3000 else 500] * 6)
        self.assertEqual(links["left"].writes[-1], [0] * 6)

    def test_reached_targets_and_opening_never_trigger_hold(self):
        for target in ([.4] * 6, [1] * 6):
            with self.subTest(target=target):
                node, links = self.make_node()
                self.drive_closing(node, target=target)
                self.assertEqual(links["left"].writes[-1], module.normalized_to_raw(target))
                self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_failed_feedback_resets_stall_window_and_missing_samples_cannot_trigger_hold(self):
        node, links = self.make_node()
        self.drive_closing(node, feedback=lambda ms: ms != 3000)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        self.drive_closing(node, duration_ms=3000)
        self.assertEqual(links["left"].writes[-1], [400] * 5 + [0])
        # Repeating the same snapshot cannot substitute for five seconds of measurements.
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=0)
        start = node.clock_now
        for ms in range(20, 5120, 20):
            node.clock_now = start + ms * 1000
            msg = command(node, sequence=node.guard.last_sequence + 1)
            for side in module.SIDES:
                msg["hands"][side]["drive_position_normalized"] = [0] * 6
            node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertEqual(links["left"].writes[-1], [0] * 6)

    def test_failed_write_does_not_commit_new_hold_and_watchdog_clears_hold(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=4980)
        node.clock_now += 60000
        links["left"].fail_writes = True
        with patch("sys.stderr", io.StringIO()), self.assertRaises(RuntimeError):
            self.drive_closing(node, duration_ms=0)
        self.assertIsNone(node.closing_holds["left"][1].held_raw)
        links["left"].fail_writes = False
        self.drive_closing(node, duration_ms=60)
        self.assertIsNotNone(node.closing_holds["left"][1].held_raw)
        with patch("sys.stderr", io.StringIO()):
            node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertEqual(links["left"].writes[-1], [1000] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_command_gap_and_changed_target_do_not_accumulate_old_stall_time(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=4980)
        node.clock_now += node.command_timeout_us
        self.drive_closing(node, duration_ms=120)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        self.drive_closing(node, duration_ms=4980, target=[.1] * 6)
        self.assertEqual(links["left"].writes[-1], [100] * 6)

    def test_grasp_mode_also_holds_bending_channels(self):
        node, links = self.make_node()
        self.drive_closing(node)
        msg = command(node, sequence=node.guard.last_sequence + 1)
        msg["mode"] = "GRASP_SETPOINT"
        msg["hands"] = {side: {"grasp": {"closure": 1}} for side in module.SIDES}
        node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertEqual(links["left"].writes[-1], [400] * 5 + [0])

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

    def test_state_capture_cannot_mix_new_measurement_with_older_envelope_time(self):
        node, _ = self.make_node()
        node.read_states(now=node.clock_now)
        old = node.snapshots["left"]

        def concurrently_finish_read():
            data = dict(old.data, angle=[800] * 6)
            node.snapshots["left"] = replace(old, data=data, sample_mono_us=node.clock_now + 1)
            return node.clock_now

        with patch.object(module, "monotonic_us", side_effect=concurrently_finish_read):
            state = node.make_state()
        self.assertTrue(state["valid"])
        self.assertEqual(state["hands"]["left"]["drive_position_raw"], [60, 50, 40, 30, 20, 10])
        self.assertEqual(state["hands"]["left"]["sample_mono_us"], state["sample_mono_us"])

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
        self.assertAlmostEqual(_hz, 1000 / 60)

    def test_hold_and_feedback_cli_validation(self):
        for option, value in (("--closing-hold-ms", "0"), ("--closing-motion-raw", "-1"),
                              ("--closing-motion-raw", "1000"), ("--state-hz", "nan"),
                              ("--state-hz", "0"), ("--state-hz", "101")):
            with self.subTest(option=option, value=value), patch.object(module, "load_handlink") as load, \
                    patch("sys.stderr"), self.assertRaises(SystemExit):
                module.main([option, value])
            load.assert_not_called()
        with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                patch.object(module, "run_node", return_value=0) as run:
            module.main(["--closing-hold-ms", "6000", "--closing-motion-raw", "5", "--state-hz", "20"])
        self.assertEqual(run.call_args.args[0].closing_hold_us, 6000000)
        self.assertEqual(run.call_args.args[0].closing_motion_raw, 5)
        self.assertEqual(run.call_args.args[3], 20)

    @staticmethod
    def diagnostic_records(output):
        return [json.loads(line.split("rh56ftp_hand: ", 1)[1])
                for line in output.getvalue().splitlines() if line.startswith("rh56ftp_hand: {")]

    def test_feedback_diagnostics_identify_stale_side_and_recovery(self):
        node, links = self.make_node()
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=node.clock_now):
            node.read_states(now=node.clock_now)
            with patch.object(links["left"], "read_state", side_effect=RuntimeError("left TCP timeout")):
                node.read_states(now=node.clock_now + node.feedback_timeout_us)
            state = node.make_state(now=node.clock_now + node.feedback_timeout_us)
            node.report_diagnostics(state, now=node.clock_now + node.feedback_timeout_us)
            node.read_states(now=node.clock_now + node.feedback_timeout_us + 1)
        records = self.diagnostic_records(output)
        failure = next(r for r in records if r["event"] == "feedback_read_failed")
        self.assertEqual(failure["side"], "left")
        self.assertIn("left TCP timeout", failure["reason"])
        health = next(r for r in records if r["event"] == "health")
        self.assertFalse(health["state_valid"])
        self.assertFalse(health["hands"]["left"]["valid"])
        self.assertEqual(health["hands"]["left"]["invalid_reason"], "feedback_timestamp_expired_or_future")
        self.assertEqual(health["hands"]["left"]["feedback_age_ms"], 500)
        self.assertTrue(health["hands"]["right"]["valid"])
        self.assertEqual(node.snapshots["left"].error, "")
        self.assertFalse(any(r["event"] == "feedback_read_recovered" for r in records))

    def test_failed_write_diagnostic_preserves_previous_ack(self):
        node, links = self.make_node()
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=node.clock_now):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
            links["right"].fail_writes = True
            with self.assertRaisesRegex(RuntimeError, "write failed"):
                node.handle_command("hand.command", json.dumps(command(node, sequence=2)), now=node.clock_now)
            node.read_states(now=node.clock_now)
            node.report_diagnostics(node.make_state(now=node.clock_now), now=node.clock_now)
        records = self.diagnostic_records(output)
        failure = next(r for r in records if r["event"] == "modbus_write_failed")
        self.assertEqual((failure["side"], failure["phase"]), ("right", "angle"))
        rejection = next(r for r in records if r["event"] == "reject command")
        self.assertEqual((rejection["sequence"], rejection["accepted_sequence"]), (2, 1))
        health = next(r for r in records if r["event"] == "health")
        self.assertEqual((health["received"], health["accepted"], health["rejected"]), (2, 1, 1))
        self.assertEqual(health["hands"]["right"]["write"]["errors"], 1)
        self.assertEqual(node.guard.last_sequence, 1)

    def test_slow_write_diagnostic_shows_ack_command_age(self):
        node, links = self.make_node()
        clock = [node.clock_now]
        output = io.StringIO()
        write = links["left"].write_angle_set

        def slow_write(values):
            clock[0] += 60000
            write(values)

        with patch("sys.stderr", output), patch.object(module, "monotonic_us", side_effect=lambda: clock[0]), \
                patch.object(links["left"], "write_angle_set", side_effect=slow_write):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
            node.read_states(now=clock[0])
            node.report_diagnostics(node.make_state(now=clock[0]), now=clock[0])
        records = self.diagnostic_records(output)
        slow = next(r for r in records if r["event"] == "modbus_write_slow")
        self.assertEqual((slow["side"], slow["phase"], slow["duration_ms"]), ("left", "angle", 60))
        health = next(r for r in records if r["event"] == "health")
        self.assertEqual(health["ack_command_age_ms"], 60)
        self.assertEqual(health["ack_write_age_ms"], 0)

    def test_repeated_diagnostics_are_throttled_with_suppressed_count(self):
        log = module.DiagnosticLog()
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=1000000):
            for _ in range(3):
                log.emit("feedback_read_failed", side="left", reason="timeout")
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=2000000):
            log.emit("feedback_read_failed", side="left", reason="timeout")
        records = self.diagnostic_records(output)
        self.assertEqual(len(records), 2)
        self.assertEqual(records[-1]["suppressed"], 2)

    def test_normal_operation_and_recovery_are_silent(self):
        node, links = self.make_node()
        output = io.StringIO()
        with patch("sys.stderr", output), patch("sys.stdout", output), \
                patch.object(module, "monotonic_us", return_value=node.clock_now):
            node.connect()
            node._safe_pose()
            node.read_states(now=node.clock_now)
            node.report_diagnostics(node.make_state(now=node.clock_now), now=node.clock_now)
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
            node.report_diagnostics(node.make_state(now=node.clock_now), now=node.clock_now + 2000000)
            node._safe_pose()
            node.close()
        self.assertEqual(output.getvalue(), "")
        with patch.object(links["left"], "read_state", side_effect=RuntimeError("read failed")), \
                patch("sys.stderr", io.StringIO()):
            node.read_states(now=node.clock_now)
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=node.clock_now):
            node.read_states(now=node.clock_now)
            node.report_diagnostics(node.make_state(now=node.clock_now), now=node.clock_now)
        self.assertEqual(output.getvalue(), "")

    def test_initial_feedback_wait_is_silent_until_timeout(self):
        node, _links = self.make_node()
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=node.clock_now):
            node.report_diagnostics(node.make_state(now=node.clock_now), now=node.clock_now)
            pending = node.clock_now + node.feedback_timeout_us - 1
            node.report_diagnostics(node.make_state(now=pending), now=pending)
            self.assertEqual(output.getvalue(), "")
            expired = pending + 1
            node.report_diagnostics(node.make_state(now=expired), now=expired)
        records = self.diagnostic_records(output)
        self.assertTrue(any(r["event"] == "feedback_status_changed" for r in records))
        self.assertTrue(any(r["event"] == "health" for r in records))

    def test_poll_timing_distinguishes_wait_overrun_from_slow_previous_cycle(self):
        for finish, next_start, expected_gap, expected_idle, expected_lag in (
                (1010000, 1610000, 610, 600, 510),
                (1600000, 1600000, 600, 0, 0)):
            node, _links = self.make_node()
            output = io.StringIO()
            with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=next_start):
                node.reader_poll_started(100000, now=1000000)
                node.reader_poll_finished(100000, now=finish)
                node.reader_poll_started(100000, now=next_start)
            event = next(r for r in self.diagnostic_records(output) if r["event"] == "feedback_poll_gap")
            self.assertEqual(event["start_gap_ms"], expected_gap)
            self.assertEqual(event["idle_gap_ms"], expected_idle)
            self.assertEqual(event["wait_overrun_ms"], expected_lag)
            self.assertEqual(event["cycle_duration_ms"], (finish - 1000000) / 1000)

    def test_normal_poll_timing_is_silent_and_exposes_current_wait(self):
        node, _links = self.make_node()
        output = io.StringIO()
        with patch("sys.stderr", output):
            node.reader_poll_started(100000, now=1000000)
            node.reader_poll_finished(100000, now=1010000)
            node.reader_poll_started(100000, now=1110000)
            node.reader_poll_finished(100000, now=1120000)
            details = node.reader_diagnostics(1720000)
        self.assertEqual(output.getvalue(), "")
        self.assertEqual(details["phase"], "waiting")
        self.assertEqual(details["current_wait_overrun_ms"], 510)
        self.assertEqual(details["cycle_finish_age_ms"], 600)
        self.assertEqual(details["max_wait_overrun_ms"], 10)

    def test_each_hand_uses_own_read_start_after_delayed_first_read(self):
        node, links = self.make_node()
        clock = [1000000]
        calls = [0]
        output = io.StringIO()

        def mono():
            calls[0] += 1
            if calls[0] == 1:  # Delayed before the first hand begins its read.
                clock[0] += 510000
            return clock[0]

        read_left, read_right = links["left"].read_state, links["right"].read_state

        def left():
            clock[0] += 4000
            return read_left()

        def right():
            clock[0] += 6000
            return read_right()

        with patch("sys.stderr", output), patch.object(module, "monotonic_us", side_effect=mono), \
                patch.object(links["left"], "read_state", side_effect=left), \
                patch.object(links["right"], "read_state", side_effect=right):
            node.read_states()
            state = node.make_state(now=clock[0])
            node.report_diagnostics(state, now=clock[0])
        self.assertTrue(state["valid"])
        records = self.diagnostic_records(output)
        self.assertEqual(records, [])
        for side, duration, stamp, age in (("left", 4, 1510000, 10), ("right", 6, 1514000, 6)):
            hand = state["hands"][side]
            self.assertEqual(node.snapshots[side].read_duration_ms, duration)
            self.assertEqual(node.snapshots[side].sample_timestamp_offset_ms, duration)
            self.assertEqual(hand["feedback_age_ms"], age)
            self.assertEqual(hand["sample_mono_us"], stamp)

    def test_slow_first_hand_does_not_expire_new_second_hand_sample(self):
        node, links = self.make_node()
        clock = [1000000]
        read_left = links["left"].read_state

        def slow_left():
            clock[0] += 510000
            return read_left()

        with patch("sys.stderr", io.StringIO()), \
                patch.object(module, "monotonic_us", side_effect=lambda: clock[0]), \
                patch.object(links["left"], "read_state", side_effect=slow_left):
            node.read_states()
        state = node.make_state(now=clock[0])
        self.assertFalse(state["hands"]["left"]["valid"])
        self.assertTrue(state["hands"]["right"]["valid"])
        self.assertEqual(state["hands"]["right"]["feedback_age_ms"], 0)

    def test_feedback_loop_subtracts_read_time_and_skips_wait_after_overrun(self):
        node, _ = self.make_node()
        clock = [1000000]
        starts, waits = [], []
        durations = iter([24000, 90000, 12000])

        class Stop:
            def is_set(self):
                return len(waits) == 3

            def wait(self, seconds):
                waits.append(seconds)
                clock[0] += round(seconds * 1000000)

        def read():
            starts.append(clock[0])
            clock[0] += next(durations)

        with patch("sys.stderr", io.StringIO()), \
                patch.object(module, "monotonic_us", side_effect=lambda: clock[0]), \
                patch.object(node, "read_states", side_effect=read):
            module.feedback_read_loop(node, Stop(), 60000)
        self.assertEqual(starts, [1000000, 1060000, 1150000])
        self.assertEqual(waits, [.036, 0, .048])

    def test_diagnostic_interval_cli_validation(self):
        with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                patch.object(module, "run_node", return_value=0) as run:
            module.main(["--diagnostic-interval-s", "0"])
        self.assertEqual(run.call_args.args[0].diagnostic_interval_us, 0)
        for value in ("-1", "nan", "inf"):
            with patch.object(module, "load_handlink") as load, patch("sys.stderr"), self.assertRaises(SystemExit):
                module.main(["--diagnostic-interval-s", value])
            load.assert_not_called()

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
