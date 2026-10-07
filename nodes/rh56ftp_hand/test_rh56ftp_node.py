import json
import io
import importlib.util
import multiprocessing
import subprocess
import sys
import time
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, call, patch


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
        self.fail_mode = False

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

    def write_mode_set(self, values):
        if self.fail_mode:
            raise RuntimeError("mode write failed")
        self.setting_writes.append(("mode", list(values)))
        self.events.append(("mode", list(values)))

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


class ConsoleLogTests(unittest.TestCase):
    def test_default_text_hides_health_and_reports_faults_with_repeat_count(self):
        log = module.DiagnosticLog()
        output = io.StringIO()
        clock = [1000000]
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", side_effect=lambda: clock[0]):
            log.emit("health", throttle_us=0, hands={"left": {"valid": True}})
            self.assertEqual(output.getvalue(), "")
            for offset in (0, 1000000, 9000000, 10000000):
                clock[0] = 1000000 + offset
                log.emit("feedback_read_failed", side="left", reason="TCP timeout", host="192.168.11.210", port=6000)
            # A different error must not wait for the repeat interval.
            log.emit("feedback_read_failed", side="left", reason="connection reset")
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 3)
        self.assertIn("左手 读取反馈失败", lines[0])
        self.assertIn("[error]", lines[0])
        self.assertIn("192.168.11.210:6000", lines[0])
        self.assertIn("期间省略同类日志=2", lines[1])
        self.assertIn("connection reset", lines[2])
        self.assertNotIn('"event"', output.getvalue())

    def test_device_error_and_holding_are_readable(self):
        log = module.DiagnosticLog()
        output = io.StringIO()
        with patch("sys.stderr", output):
            log.emit("device_error", side="right", channels=[{
                "channel_name": "thumb_rotation", "code": 37,
                "error_names": ["stall", "overcurrent"], "unknown_bits": 32,
                "current_raw": -128, "force_raw": 301, "temperature_c": 40,
                "actual_raw": 483, "requested_raw": 500, "applied_raw": -1}], command_valid=True)
            log.emit("hold_phase_changed", side="left", channel_name="index", phase="holding",
                     actual_raw=400, requested_raw=0, threshold=25, closing_age_ms=10000)
        text = output.getvalue()
        for expected in ("右手 设备故障", "拇指侧摆: 堵转、过流、未知故障位 32", "电流=-128",
                         "左手食指 满足抓握完成判据，切换到保持状态", "位置误差阈值=25", "偏差持续(ms)=10000", "[info]", "[error]",
                         "当前位置=483", "最近请求位置=500", "最近成功写入目标=-1", "控制状态=有有效指令"):
            self.assertIn(expected, text)

    def test_feedback_failure_and_recovery_print_once(self):
        node, links = NodeTests().make_node()
        node.diag = module.DiagnosticLog()
        output = io.StringIO()
        with patch("sys.stderr", output):
            node.read_states(now=node.clock_now)
            node.report_diagnostics(node.make_state(node.clock_now), node.clock_now)
            self.assertEqual(output.getvalue(), "")
            with patch.object(links["left"], "read_state", side_effect=RuntimeError("TCP timeout")):
                node.read_states(now=node.clock_now)
            node.report_diagnostics(node.make_state(node.clock_now), node.clock_now)
            node.report_diagnostics(node.make_state(node.clock_now), node.clock_now)
            node.read_states(now=node.clock_now + 1)
            node.report_diagnostics(node.make_state(node.clock_now + 1), node.clock_now + 1)
            node.report_diagnostics(node.make_state(node.clock_now + 1), node.clock_now + 1)
        text = output.getvalue()
        self.assertEqual(text.count("左手 反馈异常"), 1)
        self.assertEqual(text.count("左手 反馈已恢复"), 1)
        self.assertNotIn("health", text)

    def test_cli_default_and_json_opt_in(self):
        for args, expected in (([], "text"), (["--log-format", "json"], "json")):
            with patch.object(module, "load_handlink", return_value=("right", 6000, FakeLink)), \
                    patch.object(module, "run_node", return_value=0) as run:
                module.main(args)
            self.assertEqual(run.call_args.args[0].diag.log_format, expected)


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
                                   session_id="33333333-3333-4333-8333-333333333333", log_format="json")
        node.clock_now = 1000000
        return node, links

    def drive_closing(self, node, *, duration_ms=10100, target=None, position=None, feedback=None):
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

    def test_closing_hold_affects_all_six_channels_and_keeps_ack_fresh(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=9960)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        # Continue fresh commands and feedback until the full window is covered.
        self.drive_closing(node, duration_ms=120)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)
        self.assertEqual(node.guard.last_sequence, node.command_accepted)
        self.assertEqual(node.guard.sample_mono_us, node.clock_now)
        self.assertTrue(node.command_valid)
        state = node.make_state(now=node.clock_now)
        for side in module.SIDES:
            hand = state["hands"][side]
            self.assertEqual(hand["requested_drive_position_normalized"], [0] * 6)
            self.assertEqual(hand["commanded_drive_position_normalized"], [.4] * 6)
            self.assertEqual(hand["closing_hold_active"], [True] * 6)
            self.assertEqual(hand["closing_hold_position_normalized"], [.4] * 6)
            self.assertEqual(hand["drive_position_normalized"], [.4] * 6)
        # A new position reading must not make a latched target drift.
        before = len(links["left"].writes)
        self.drive_closing(node, duration_ms=60, position=lambda _ms, _side: [450] * 6)
        self.assertEqual(len(links["left"].writes), before)

    def test_stopped_fingers_only_read_and_ack_until_opening(self):
        node, links = self.make_node()
        self.drive_closing(node, target=[0] + [.2] * 5)
        counts = {side: len(link.events) for side, link in links.items()}
        last_write = node.last_ack_write_us
        reads = node.snapshots["left"].reads
        for target in (.1, .3, .4):
            self.drive_closing(node, duration_ms=1200, target=[0] + [target] * 5)
        self.assertGreater(node.snapshots["left"].reads, reads)
        self.assertEqual(node.last_ack_write_us, last_write)
        self.assertEqual(node.guard.sample_mono_us, node.clock_now)
        self.assertTrue(node.command_valid)
        for side, link in links.items():
            self.assertEqual(len(link.events), counts[side])
        # Only one opening channel is written; stopped neighbours stay untouched.
        self.drive_closing(node, duration_ms=0, target=[0, .6, .4, .4, .4, .4])
        self.assertEqual(links["left"].writes[-1], [None, None, None, None, 600, None])

    def test_only_opening_beyond_stopped_position_releases_hold(self):
        node, links = self.make_node()
        self.drive_closing(node)
        self.drive_closing(node, duration_ms=0, target=[1] * 6)
        self.assertEqual(links["left"].writes[-1], [1000] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
        # Changing a target toward the obstruction must not restart any channel.
        self.drive_closing(node, target=[.2] * 6)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)
        before = len(links["left"].writes)
        self.drive_closing(node, duration_ms=0, target=[.1] * 6)
        self.assertEqual(len(links["left"].writes), before)
        self.drive_closing(node, duration_ms=0, target=[.3] * 6)
        self.assertEqual(len(links["left"].writes), before)
        self.assertTrue(all(c.held_raw == 400 for c in node.closing_holds["left"][1:]))
        self.drive_closing(node, duration_ms=0, target=[.6] * 6)
        self.assertEqual(links["left"].writes[-1], [600] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_reached_finger_continues_while_residual_fingers_hold_independently(self):
        node, links = self.make_node()
        self.drive_closing(node, position=lambda ms, side: (
            [400, 400, max(0, 700 - ms // 10), 400, 400, 400] if side == "left" else [400] * 6))
        self.assertEqual(links["left"].writes[-1], [-1, -1, -1, None, -1, -1])
        self.assertEqual(links["right"].writes[-1], [-1] * 6)
        self.assertEqual(links["left"].setting_writes[-2:],
                         [("speed", [100, 100, 100, 500, 100, 100]),
                          ("force", [100, 100, 100, 1000, 100, 100])])

    def test_hold_limits_are_cached_and_motion_restores_grasp_limits(self):
        node, links = self.make_node()
        node.hold_speed, node.hold_force = 80, 90
        self.drive_closing(node)
        for link in links.values():
            self.assertEqual(link.events[-3:],
                             [("speed", [80] * 6),
                              ("force", [90] * 6),
                              ("angle", [-1] * 6)])
        before = {side: len(link.events) for side, link in links.items()}
        self.drive_closing(node, duration_ms=120)
        self.assertEqual({side: len(link.events) for side, link in links.items()}, before)
        self.drive_closing(node, duration_ms=0, target=[0.6] * 6)
        for link in links.values():
            self.assertEqual(link.events[-3:], [("speed", [500] * 6),
                                                ("force", [1000] * 6), ("angle", [600] * 6)])

    def test_hold_limit_failure_does_not_commit_hold_or_ack_and_retries(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=9960)
        sequence = node.guard.last_sequence
        links["left"].fail_force = True
        with self.assertRaisesRegex(RuntimeError, "force write failed"):
            self.drive_closing(node, duration_ms=120)
        self.assertTrue(all(c.held_raw is None for c in node.closing_holds["left"]))
        self.assertEqual(node.guard.last_sequence, sequence + 3)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        links["left"].fail_force = False
        self.drive_closing(node, duration_ms=0)
        self.assertEqual(links["left"].events[-3:], [("speed", [100] * 6),
                                                   ("force", [100] * 6),
                                                   ("angle", [-1] * 6)])

    def test_persistent_residual_holds_without_position_stability_requirement(self):
        node, links = self.make_node()
        self.drive_closing(node, position=lambda ms, _side: [400 if ms < 2000 or ms > 3000 else 500] * 6)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)

    def test_reached_stable_targets_hold_and_new_targets_restore_motion(self):
        for new_target in (.2, .6):
            with self.subTest(new_target=new_target):
                node, links = self.make_node()
                position = lambda ms, _side: [396 if (ms // 60) % 2 else 404] * 6
                self.drive_closing(node, duration_ms=9960, target=[.4] * 6, position=position)
                self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
                self.drive_closing(node, duration_ms=120, target=[.4] * 6, position=position)
                for side, link in links.items():
                    self.assertEqual(link.writes[-1], [-1] * 6)
                    self.assertTrue(all(c.held_at_target and c.hold_reason == "position_stable"
                                        for c in node.closing_holds[side]))
                before = {side: len(link.events) for side, link in links.items()}
                self.drive_closing(node, duration_ms=120, target=[.4] * 6)
                self.assertEqual({side: len(link.events) for side, link in links.items()}, before)
                self.drive_closing(node, duration_ms=0, target=[new_target] * 6)
                for side, link in links.items():
                    self.assertFalse(any(c.held_raw is not None for c in node.closing_holds[side]))
                    self.assertEqual(link.events[-3:], [("speed", [500] * 6),
                        ("force", [1000] * 6), ("angle", [round(new_target * 1000)] * 6)])

    def test_failed_feedback_resets_stall_window_and_missing_samples_cannot_trigger_hold(self):
        node, links = self.make_node()
        self.drive_closing(node, feedback=lambda ms: ms != 3000)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        self.drive_closing(node, duration_ms=3000)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)
        # Repeating the same snapshot cannot substitute for ten seconds of measurements.
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=0)
        start = node.clock_now
        for ms in range(20, 10120, 20):
            node.clock_now = start + ms * 1000
            msg = command(node, sequence=node.guard.last_sequence + 1)
            for side in module.SIDES:
                msg["hands"][side]["drive_position_normalized"] = [0] * 6
            node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertEqual(links["left"].writes[-1], [0] * 6)

    def test_failed_write_does_not_commit_new_hold_and_watchdog_clears_hold(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=9960)
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
        self.assertEqual(links["left"].writes[-1], [1000, 1000, 1000, 1000, 1000, 500])
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_command_gap_resets_but_changing_closing_target_keeps_stall_window(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=9960)
        node.clock_now += node.command_timeout_us
        self.drive_closing(node, duration_ms=120)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        self.drive_closing(node, duration_ms=9960, target=[.1] * 6)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)

    def test_stability_threshold_is_strict_and_uses_full_window(self):
        node, _ = self.make_node()
        # Every adjacent step is smaller than ten, but cumulative motion is ten.
        self.drive_closing(node, duration_ms=10020, target=[.4] * 6,
                           position=lambda ms, _side: [400 + min(ms // 1000, 10)] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
        node, _ = self.make_node()
        self.drive_closing(node, duration_ms=12000, target=[.4] * 6,
                           position=lambda ms, _side: [395 if (ms // 60) % 2 else 405] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
        node, _ = self.make_node()
        node.closing_motion_raw = 0
        self.drive_closing(node, target=[.4] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_stability_feedback_failure_and_target_change_restart_window(self):
        for change in ("read_failure", "command_gap", "target_change"):
            with self.subTest(change=change):
                node, _ = self.make_node()
                self.drive_closing(node, duration_ms=9960, target=[.4] * 6)
                if change == "read_failure":
                    self.drive_closing(node, duration_ms=0, target=[.4] * 6, feedback=lambda _ms: False)
                elif change == "command_gap":
                    node.clock_now += node.command_timeout_us
                target = [.401 if change == "target_change" else .4] * 6
                node.clock_now += 20000
                self.drive_closing(node, duration_ms=9960, target=target)
                self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
                self.drive_closing(node, duration_ms=60, target=target)
                self.assertTrue(all(c.held_raw == 400 for c in node.closing_holds["left"]))

    def test_feedback_gap_resets_timer_despite_continuous_commands(self):
        node, _ = self.make_node()
        self.drive_closing(node, duration_ms=9960)
        # Commands remain fresh, but no new measurements arrive beyond the feedback timeout.
        old_stamp = node.clock_now
        for _ in range(26):
            node.clock_now += 20000
            msg = command(node, sequence=node.guard.last_sequence + 1)
            for side in module.SIDES:
                msg["hands"][side]["drive_position_normalized"] = [0] * 6
            node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertGreaterEqual(node.clock_now - old_stamp, node.feedback_timeout_us)
        self.drive_closing(node, duration_ms=9960)
        self.assertIsNone(node.closing_holds["left"][1].held_raw)
        self.drive_closing(node, duration_ms=60)
        self.assertEqual(node.closing_holds["left"][1].held_raw, 400)

    def test_grasp_mode_also_holds_all_channels(self):
        node, links = self.make_node()
        self.drive_closing(node)
        msg = command(node, sequence=node.guard.last_sequence + 1)
        msg["mode"] = "GRASP_SETPOINT"
        msg["hands"] = {side: {"grasp": {"closure": 1}} for side in module.SIDES}
        node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)

    def test_rotation_holds_in_both_directions_and_only_reversing_releases(self):
        for target, blocked_targets, release in ((0, (.1, .3, .4), .6), (.8, (.9, .5, .4), .2)):
            with self.subTest(target=target):
                node, links = self.make_node()
                self.drive_closing(node, duration_ms=9960, target=[target] + [.4] * 5)
                self.assertIsNone(node.closing_holds["left"][0].held_raw)
                self.drive_closing(node, duration_ms=120, target=[target] + [.4] * 5)
                for side, link in links.items():
                    self.assertEqual(link.events[-3:], [
                        ("speed", [100] * 6), ("force", [100] * 6),
                        ("angle", [-1] * 6)])
                    hand = node.make_state(node.clock_now)["hands"][side]
                    self.assertEqual(hand["closing_hold_active"], [True] * 6)
                    self.assertEqual(hand["closing_hold_position_normalized"], [.4] * 6)
                before = {side: len(link.events) for side, link in links.items()}
                for value in blocked_targets:
                    self.drive_closing(node, duration_ms=120, target=[value] + [.4] * 5,
                                       position=lambda _ms, _side: [450] + [400] * 5)
                # Missing feedback must not unlatch a stopped rotation, either.
                self.drive_closing(node, duration_ms=600, target=[target] + [.4] * 5,
                                   feedback=lambda _ms: False)
                self.assertEqual({side: len(link.events) for side, link in links.items()}, before)
                self.drive_closing(node, duration_ms=0, target=[release] + [.4] * 5)
                for side, link in links.items():
                    self.assertIsNone(node.closing_holds[side][0].held_raw)
                    self.assertEqual(link.events[-3:], [
                        ("speed", [100] * 5 + [500]), ("force", [100] * 5 + [1000]),
                        ("angle", [None] * 5 + [round(release * 1000)])])

    def test_rotation_reversing_residual_restarts_full_observation_window(self):
        for by_feedback in (False, True):
            with self.subTest(by_feedback=by_feedback):
                node, _ = self.make_node()
                target = [.2] + [.4] * 5
                self.drive_closing(node, duration_ms=9960, target=target)
                node.clock_now += 20000
                # Reverse via a new target or via feedback crossing the unchanged target.
                if not by_feedback:
                    target[0] = .8
                position = lambda _ms, _side: [100 if by_feedback else 400] + [400] * 5
                self.drive_closing(node, duration_ms=9960, target=target, position=position)
                self.assertIsNone(node.closing_holds["left"][0].held_raw)
                self.drive_closing(node, duration_ms=60, target=target, position=position)
                self.assertEqual(node.closing_holds["left"][0].held_raw, 100 if by_feedback else 400)

    def test_residual_threshold_is_strict_in_both_directions(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                node, _ = self.make_node()
                # Movement spanning the threshold disables the stability branch.
                self.drive_closing(node, duration_ms=12000, target=[.4] * 6,
                    position=lambda ms, _side: [400 + sign * (0 if (ms // 60) % 2 else 10)] * 6)
                self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
                node, _ = self.make_node()
                self.drive_closing(node, duration_ms=9960, target=[.4] * 6,
                    position=lambda ms, _side: [400 + sign * (11 if (ms // 60) % 2 else 30)] * 6)
                self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
                self.drive_closing(node, duration_ms=120, target=[.4] * 6,
                    position=lambda ms, _side: [400 + sign * (11 if (ms // 60) % 2 else 30)] * 6)
                self.assertTrue(all(c.hold_reason == "position_error" for c in node.closing_holds["left"]))

    def test_stable_hold_write_failure_preserves_observation_and_ack_for_retry(self):
        node, links = self.make_node()
        node.closing_hold_us = 60000
        self.drive_closing(node, duration_ms=40, target=[.4] * 6)
        sequence = node.guard.last_sequence
        samples = node.closing_holds["left"][0].stability_samples
        node.clock_now += 20000
        links["left"].fail_force = True
        with self.assertRaisesRegex(RuntimeError, "force write failed"):
            self.drive_closing(node, duration_ms=0, target=[.4] * 6)
        self.assertEqual(node.guard.last_sequence, sequence)
        self.assertEqual(node.closing_holds["left"][0].stability_samples, samples)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))
        links["left"].fail_force = False
        self.drive_closing(node, duration_ms=0, target=[.4] * 6)
        self.assertEqual(links["left"].writes[-1], [-1] * 6)
        self.assertTrue(all(c.hold_reason == "position_stable" for c in node.closing_holds["left"]))

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
            self.assertEqual(link.events, [("mode", [0] * 6),
                                           ("speed", [500] * 6),
                                           ("force", [1000] * 6),
                                           ("angle", [1000, 1000, 1000, 1000, 1000, 500])])
        node.read_states(now=node.clock_now)
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        node.supervise(now=node.clock_now + node.command_timeout_us)
        node._safe_pose(force_open=True)  # Exit pose uses the same settings.
        for link in links.values():
            self.assertEqual(len(link.setting_writes), 3)

    def test_setting_failure_blocks_angles_and_retries(self):
        node, links = self.make_node()
        links["right"].fail_force = True
        with self.assertRaisesRegex(RuntimeError, "right speed/force/mode setup failed"):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertFalse(node.guard.authorized)
        self.assertFalse(node.command_valid)
        for link in links.values():
            self.assertEqual(link.writes, [])
        links["right"].fail_force = False
        node.supervise(now=node.clock_now)
        for link in links.values():
            self.assertEqual(link.writes, [[-1] * 6])  # A transport/settings failure does not prove six failed channels.
            self.assertIn(("force", [100] * 6), link.setting_writes)

    def test_angle_control_recovers_from_previous_force_mode(self):
        class ModeAwareLink(FakeLink):
            """Model unloaded angle tracking in manual §2.6.20 mode 0."""
            def __init__(self):
                super().__init__()
                self.modes = [1] * 6  # Device state left by the previous node.
                self.angles = [400] * 6

            def write_mode_set(self, values):
                super().write_mode_set(values)
                self.modes = list(values)

            def write_force_set(self, values):
                if self.modes != [0] * 6:
                    raise RuntimeError("force changed before restoring angle control")
                super().write_force_set(values)

            def write_angle_set(self, values):
                super().write_angle_set(values)
                for index, value in enumerate(values):
                    if self.modes[index] == 0 and value is not None and value >= 0:
                        self.angles[index] = value

            def read_state(self):
                return dict(super().read_state(), angle=list(self.angles))

        for mode in ("NORMALIZED_POSITION", "GRASP_SETPOINT"):
            with self.subTest(mode=mode):
                links = {side: ModeAwareLink() for side in module.SIDES}
                node = module.Rh56FtpNode(links, clock_id="test-clock", force=1200)
                node.clock_now = 1000000

                def assert_position(position, thumb_rotation=None):
                    node.read_states(now=node.clock_now)
                    state = node.make_state(now=node.clock_now)
                    for side in module.SIDES:
                        self.assertTrue(state["hands"][side]["valid"])
                        self.assertEqual(state["hands"][side]["drive_position_normalized"],
                                         [position if thumb_rotation is None else thumb_rotation] + [position] * 5)

                node.connect()
                node._safe_pose()
                assert_position(1, thumb_rotation=.5)
                for sequence, position in enumerate((.2, .8, .3), start=1):
                    msg = command(node, sequence=sequence)
                    msg["mode"] = mode
                    msg["hands"] = {
                        side: ({"drive_position_normalized": [position] * 6}
                               if mode == "NORMALIZED_POSITION" else
                               {"grasp": {"closure": 1 - position}})
                        for side in module.SIDES}
                    node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
                    assert_position(position)
                with patch("sys.stderr", io.StringIO()):
                    node.clock_now += node.command_timeout_us
                    node.supervise(now=node.clock_now)
                assert_position(1, thumb_rotation=.5)
                self.assertFalse(node.command_valid)
                for link in links.values():
                    self.assertEqual(link.setting_writes,
                                     [("mode", [0] * 6), ("speed", [500] * 6),
                                      ("force", [1200] * 6)])

    def test_angle_write_failure_reapplies_settings(self):
        node, links = self.make_node()
        node._safe_pose()
        links["left"].fail_writes = True
        with self.assertRaisesRegex(RuntimeError, "write failed"):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        links["left"].fail_writes = False
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertEqual(len(links["left"].setting_writes), 6)
        self.assertEqual(len(links["right"].setting_writes), 3)

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
                         [("mode", [0] * 6), ("speed", [250] * 6), ("force", [1200] * 6)])

    def test_mode_failure_prevents_motion_and_ack(self):
        node, links = self.make_node()
        links["right"].fail_mode = True
        with patch("sys.stderr", io.StringIO()), self.assertRaisesRegex(RuntimeError, "mode setup failed"):
            node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertFalse(node.guard.authorized)
        self.assertTrue(all(not link.writes for link in links.values()))
        links["right"].fail_mode = False
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        self.assertTrue(node.command_valid)

    def test_config_settings_and_cli_override_are_loaded_before_connections(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "hand.yaml"
            config.write_text("speed: 300\nforce: 750\n")
            with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                    patch.object(module, "run_node", return_value=0) as run:
                module.main(["--config", str(config)])
                self.assertEqual((run.call_args.args[0].speed, run.call_args.args[0].force), (300, 750))
                module.main(["--force", "900", "--config", str(config)])
                self.assertEqual(run.call_args.args[0].force, 900)
            for text in ("force: true", "force: 3001", "force: 1.5", "forse: 500", "[]", "force: ["):
                config.write_text(text)
                with patch.object(module, "load_handlink") as load, patch("sys.stderr"), self.assertRaises(SystemExit):
                    module.main(["--config", str(config)])
                load.assert_not_called()

    def test_repository_ips_threshold_and_cli_precedence(self):
        import yaml
        config = SCRIPT.parents[2] / "config/rh56ftp_hand.yaml"
        settings = yaml.safe_load(config.read_text())
        self.assertNotIn("hold_control", settings)
        factory = Mock(side_effect=FakeLink)
        with patch.object(module, "load_handlink", return_value=("fallback", 6000, factory)), \
                patch.object(module, "run_node", return_value=0) as run:
            module.main(["--config", str(config)])
            self.assertEqual([c.args[0] for c in factory.call_args_list],
                             [settings["right_host"], settings["left_host"]] * 2)
            node = run.call_args.args[0]
            self.assertEqual((node.speed, node.force, node.hold_speed, node.hold_force),
                             (500, 1000, 100, 100))
            self.assertEqual(node.closing_motion_raw, settings["threshold"])
            self.assertEqual(node.closing_hold_us, 10000000)
            factory.reset_mock()
            module.main(["--config", str(config), "--right-host", "override", "--left-host", "",
                         "--threshold", "31"])
            self.assertEqual([c.args[0] for c in factory.call_args_list], ["override"] * 2)
            self.assertIsNone(run.call_args.args[0].links["left"])
            self.assertEqual(run.call_args.args[0].closing_motion_raw, 31)

    def test_phase_settings_validation_and_cli_overrides(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "hand.yaml"
            config.write_text("speed: {grasp: 400, hold: 80}\nforce: {grasp: 900, hold: 70}\n")
            with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                    patch.object(module, "run_node", return_value=0) as run:
                module.main(["--config", str(config)])
                node = run.call_args.args[0]
                self.assertEqual((node.speed, node.force, node.hold_speed, node.hold_force),
                                 (400, 900, 80, 70))
                module.main(["--config", str(config), "--speed", "600", "--force", "1100",
                             "--hold-speed", "90", "--hold-force", "120"])
                node = run.call_args.args[0]
                self.assertEqual((node.speed, node.force, node.hold_speed, node.hold_force),
                                 (600, 1100, 90, 120))
            for text in ("speed: {hold: true}", "speed: {hold: 1001}", "force: {hold: 3001}",
                         "force: {grasp: -1}", "speed: {hold: 1.5}", "force: {typo: 100}"):
                config.write_text(text)
                with patch.object(module, "load_handlink") as load, patch("sys.stderr"), self.assertRaises(SystemExit):
                    module.main(["--config", str(config)])
                load.assert_not_called()

    def test_threshold_default_validation_and_retired_config(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "hand.yaml"
            config.write_text("speed: 500\n")
            with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                    patch.object(module, "run_node", return_value=0) as run:
                module.main(["--config", str(config)])
                self.assertEqual(run.call_args.args[0].closing_motion_raw, 10)
            for text in ("threshold: true", "threshold: -1", "threshold: 1000", "threshold: 1.5",
                         "threshold: [10]", "right_host: 123", "left_host: null", "hold_control: {}"):
                with self.subTest(text=text):
                    config.write_text(text)
                    with patch.object(module, "load_handlink") as load, patch("sys.stderr"), self.assertRaises(SystemExit):
                        module.main(["--config", str(config)])
                    load.assert_not_called()

    def test_modbus_mode_packing_and_sparse_angle_writes(self):
        path = SCRIPT.parents[2] / "third_party/RH56FTP/python/pendant/handlink.py"
        spec = importlib.util.spec_from_file_location("test_handlink", path)
        backend = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"pymodbus": Mock(), "pymodbus.client": Mock()}):
            spec.loader.exec_module(backend)
        link = backend.HandLink()
        link._cli = Mock()
        link._cli.write_registers.return_value.isError.return_value = False
        link.write_mode_set([1] * 6)
        link.write_angle_set([None, -1, -1, None, 800, None])
        link.write_angle_set([None] * 6)
        self.assertEqual(link._cli.write_registers.call_args_list, [
            call(address=1625, values=[0x0101] * 3, device_id=1),
            call(address=1488, values=[0xFFFF, 0xFFFF], device_id=1),
            call(address=1494, values=[800], device_id=1),
        ])

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

    def test_compact_state_fields_for_fresh_stale_and_unconfigured_hands(self):
        node, links = self.make_node()
        data = {key: [offset + i for i in range(6)] for key, offset in
                (("angle", 100), ("force", 200), ("current", 300),
                 ("err", 0), ("status", 10), ("temp", 30))}
        links["right"].read_state = lambda: data
        node.links["left"] = None
        states = [node.make_state(node.clock_now)]
        node.read_states(now=node.clock_now)
        states += [node.make_state(node.clock_now),
                   node.make_state(node.clock_now + node.feedback_timeout_us)]
        self.assertTrue(states[1]["valid"])  # Only configured hands participate.
        for state in states:
            for hand in state["hands"].values():
                for field in ("angle", "angle_raw", "err", "error", "status_code", "status_values",
                              "temp", "enabled", "joint_position", "joint_velocity"):
                    self.assertNotIn(field, hand)
                for field in ("drive_position_raw", "error_codes", "status_codes", "temperature"):
                    self.assertEqual(len(hand[field]), 6 if hand["feedback_available"] else 0)
        hand = states[1]["hands"]["right"]
        for source, field in (("angle", "drive_position_raw"), ("err", "error_codes"),
                              ("status", "status_codes"), ("temp", "temperature")):
            self.assertEqual(hand[field], list(reversed(data[source])))
        self.assertEqual(hand["error_code"], 5)

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
        node.read_states(now=node.clock_now)  # Healthy feedback retains the ordinary timeout safe pose.
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertFalse(node.command_valid)
        self.assertEqual(links["left"].writes[-1], [1000, 1000, 1000, 1000, 1000, 500])

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
                for line in output.getvalue().splitlines() if "rh56ftp_hand: {" in line]

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
        self.assertIn("RuntimeError: left TCP timeout", failure["traceback"])
        self.assertIn("duration_ms", failure)
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
            changed = command(node, sequence=2)
            changed["hands"]["right"]["drive_position_normalized"][0] = .8
            with self.assertRaisesRegex(RuntimeError, "write failed"):
                node.handle_command("hand.command", json.dumps(changed), now=node.clock_now)
            node.read_states(now=node.clock_now)
            node.report_diagnostics(node.make_state(now=node.clock_now), now=node.clock_now)
        records = self.diagnostic_records(output)
        failure = next(r for r in records if r["event"] == "modbus_write_failed")
        self.assertEqual((failure["side"], failure["phase"]), ("right", "angle"))
        self.assertIn("RuntimeError: write failed", failure["traceback"])
        self.assertEqual(failure["previous_target_raw"], [1000, 900, 800, 700, 600, 500])
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
        log = module.DiagnosticLog("json")
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
        records = self.diagnostic_records(output)
        self.assertEqual([r["event"] for r in records], ["configuration"] + ["setting_write"] * 4)
        self.assertEqual(records[0]["configured_mode"], 0)
        self.assertEqual(records[0]["closing_motion_raw"], 10)
        self.assertEqual(records[0]["closing_hold_ms"], 10000)
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


class ClosingFaultTests(unittest.TestCase):
    def make_node(self):
        links = {side: FakeLink() for side in module.SIDES}
        node = module.Rh56FtpNode(links, clock_id="test-clock", closing_hold_ms=60)
        node.clock_now = 1000000
        node.diag.emit = Mock()
        return node, links

    def tick(self, node, *, forces=None, currents=None, positions=None, target=None,
             temps=None, errors=None, refresh=True, valid=True):
        node.clock_now += 20000
        if refresh:
            for side in module.SIDES:
                data = dict(angle=positions or [400] * 6, force=forces or [100] * 6,
                            current=currents or [10] * 6, temp=temps or [35] * 6,
                            err=errors or [0] * 6, status=[3] * 6)
                node.snapshots[side] = module.Snapshot(
                    data={k: module.canonical_to_rh(v[side] if isinstance(v, dict) else v)
                          for k, v in data.items()}, sample_mono_us=node.clock_now)
        msg = command(node, sequence=node.guard.last_sequence + 1, valid=valid)
        for side in module.SIDES:
            # Rotation is already at target; bending channels are closing.
            msg["hands"][side]["drive_position_normalized"] = (
                target[side] if isinstance(target, dict) else target or [.4, 0, 0, 0, 0, 0])
        node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)


    def hold(self, node):
        for _ in range(4):
            self.tick(node)
        self.assertEqual(node.closing_holds["left"][1].phase, "holding")


    def test_partial_fault_holds_on_timeout_invalid_command_but_exit_opens(self):
        for path in ("watchdog", "invalid_command", "shutdown"):
            with self.subTest(path=path):
                node, links = self.make_node()
                self.hold(node)
                self.tick(node, errors=[0, 0, 32, 0, 0, 0])
                if path == "watchdog":
                    node.supervise(now=node.clock_now + node.command_timeout_us)
                elif path == "invalid_command":
                    self.tick(node, valid=False, errors=[0, 0, 32, 0, 0, 0])
                else:
                    node._safe_pose(force_open=True)  # Explicit normal shutdown.
                for side in module.SIDES:
                    expected = [500, 1000, 1000, 1000, 1000, 1000] if path == "shutdown" else [-1] * 6
                    self.assertEqual(node._angle_applied[side], expected)
                self.assertFalse(node.command_valid)
                self.assertIn("device error 32", node.hold_control_error)
                self.assertTrue(node.make_state(node.clock_now)["valid"])

    def test_device_fault_recovers_and_next_command_opens(self):
        for code in (2, 4, 16):  # Overtemperature, overcurrent, device communication error.
            with self.subTest(code=code):
                node, _ = self.make_node()
                self.tick(node, errors=[code, 0, 0, 0, 0, 0])
                fault = node.hold_control_error
                self.assertTrue(fault)
                opened = [.5, 1, 1, 1, 1, 1]
                self.tick(node, target=opened)
                state = node.make_state(node.clock_now)
                self.assertTrue(state["valid"])
                self.assertTrue(all(hand["valid"] for hand in state["hands"].values()))
                self.assertEqual(state["hold_control_error"], "")
                self.assertTrue(state["command_valid"])
                self.assertEqual(node._angle_applied, {side: [500, 1000, 1000, 1000, 1000, 1000]
                                                      for side in module.SIDES})
                self.assertTrue(all(c.phase == "tracking" for channels in node.closing_holds.values() for c in channels))
                self.assertEqual(sum(c.args[0] == "hold_control_recovered" for c in node.diag.emit.call_args_list), 1)
                for invalid in ("stale", "read_failed", "future", "bad_position"):
                    snapshot = node.snapshots["left"]
                    stamp = node.clock_now - node.feedback_timeout_us if invalid == "stale" else (
                        node.clock_now + 1 if invalid == "future" else node.clock_now)
                    node.snapshots["left"] = replace(snapshot, sample_mono_us=stamp,
                        error="TCP timeout" if invalid == "read_failed" else "",
                        data={**snapshot.data, "angle": [1001] * 6} if invalid == "bad_position" else snapshot.data)
                    self.assertFalse(node.make_state(node.clock_now)["valid"])
                    node.snapshots["left"] = snapshot
                    self.assertTrue(node.make_state(node.clock_now)["valid"])


    def test_fault_recovery_requires_healthy_feedback_from_all_configured_hands(self):
        for invalid in ("missing", "stale", "read_failed", "future", "bad_position", "other_fault"):
            with self.subTest(invalid=invalid):
                node, _ = self.make_node()
                self.tick(node, errors={"left": [4, 0, 0, 0, 0, 0], "right": [0] * 6})
                fault = node.hold_control_error
                node.clock_now += 20000
                healthy = {side: replace(snapshot, data={**snapshot.data, "err": [0] * 6},
                                          sample_mono_us=node.clock_now)
                           for side, snapshot in node.snapshots.items()}
                node.snapshots = dict(healthy)
                snapshot = healthy["right"]
                if invalid == "missing":
                    node.snapshots["right"] = module.Snapshot()
                elif invalid == "stale":
                    node.snapshots["right"] = replace(snapshot, sample_mono_us=node.clock_now-node.feedback_timeout_us)
                elif invalid == "future":
                    node.snapshots["right"] = replace(snapshot, sample_mono_us=node.clock_now+1)
                elif invalid == "read_failed":
                    node.snapshots["right"] = replace(snapshot, error="TCP timeout")
                else:
                    key, values = ("angle", [1001] * 6) if invalid == "bad_position" else ("err", [2] * 6)
                    node.snapshots["right"] = replace(snapshot, data={**snapshot.data, key: values})
                node.supervise(now=node.clock_now)
                self.assertTrue(node.hold_control_error)
                self.assertFalse(any(c.args[0] == "hold_control_recovered" for c in node.diag.emit.call_args_list))
                node.snapshots = healthy
                node.supervise(now=node.clock_now)
                self.assertEqual(node.make_state(node.clock_now)["hold_control_error"], "")
                self.assertTrue(node.make_state(node.clock_now)["valid"])
                self.assertEqual(node.diag.emit.call_args_list[-1].kwargs["previous_error"], fault)

    def test_supervision_recovers_without_replaying_old_targets_and_fault_can_recur(self):
        node, links = self.make_node()
        self.hold(node)
        self.tick(node, errors=[4] * 6)
        node.supervise(now=node.clock_now)
        writes = {side: len(link.writes) for side, link in links.items()}
        node.clock_now += 20000
        node.snapshots = {side: replace(snapshot, data={**snapshot.data, "err": [0] * 6},
                                        sample_mono_us=node.clock_now)
                         for side, snapshot in node.snapshots.items()}
        node.supervise(now=node.clock_now)
        self.assertEqual(node.hold_control_error, "")
        self.assertFalse(node.command_valid)
        self.assertFalse(node._fault_open_sides)
        self.assertEqual({side: len(link.writes) for side, link in links.items()}, writes)
        for _ in range(3):
            node.supervise(now=node.clock_now)
        self.assertEqual(sum(c.args[0] == "hold_control_recovered" for c in node.diag.emit.call_args_list), 1)
        self.tick(node, target=[.7] * 6)
        self.assertTrue(node.command_valid)
        self.assertEqual(node._angle_applied, {side: [700] * 6 for side in module.SIDES})
        self.assertTrue(all(c.phase == "tracking" for channels in node.closing_holds.values() for c in channels))
        self.tick(node, errors=[0, 2, 0, 0, 0, 0])
        self.assertIn("device error 2", node.hold_control_error)
        self.assertEqual(node._angle_applied, {side: [-1] * 6 for side in module.SIDES})
        self.assertFalse(node._fault_open_sides)

    def test_unconfigured_hand_does_not_prevent_recovery(self):
        node, _ = self.make_node()
        node.links["right"] = node.read_links["right"] = None
        self.tick(node, errors=[4, 0, 0, 0, 0, 0])
        node.snapshots["right"] = module.Snapshot()
        left = node.snapshots["left"]
        node.snapshots["left"] = replace(left, data={**left.data, "err": [0] * 6})
        node.supervise(now=node.clock_now)
        self.assertEqual(node.hold_control_error, "")
        self.assertTrue(node.make_state(node.clock_now)["valid"])

    def test_fault_hold_write_failure_is_retried(self):
        node, links = self.make_node()
        self.hold(node)
        self.tick(node, target=[.2] + [0] * 5)  # Resume reached rotation before fault stop.
        node.hold_control_error = "left[2]: device error 32"
        snapshot = node.snapshots["left"]
        node.snapshots["left"] = replace(snapshot,
            data={**snapshot.data, "err": module.canonical_to_rh([0, 0, 32, 0, 0, 0])})
        links["left"].fail_writes = True
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertFalse(node._safe_applied)
        links["left"].fail_writes = False
        node.supervise(now=node.clock_now + node.command_timeout_us + 1)
        self.assertTrue(node._safe_applied)
        self.assertEqual(node._angle_applied, {side: [-1] * 6 for side in module.SIDES})


    def test_feedback_loss_without_channel_fault_evidence_holds_on_timeout(self):
        for lost in ("stale", "read_failed"):
            with self.subTest(lost=lost):
                node, _ = self.make_node()
                self.hold(node)
                snapshot = node.snapshots["left"]
                node.snapshots["left"] = replace(snapshot,
                    sample_mono_us=node.clock_now - node.feedback_timeout_us if lost == "stale" else snapshot.sample_mono_us,
                    error="read failed" if lost == "read_failed" else "")
                node.supervise(now=node.clock_now + node.command_timeout_us)
                self.assertEqual(node._angle_applied, {side: [-1] * 6 for side in module.SIDES})
                self.assertFalse(node._fault_open_sides)


    def test_only_hand_with_six_faulty_channels_opens(self):
        for side, other in (("left", "right"), ("right", "left")):
            for count in (1, 5, 6):
                with self.subTest(side=side, count=count):
                    node, _ = self.make_node()
                    self.hold(node)
                    self.tick(node, errors={side: [5] * count + [0] * (6 - count), other: [0] * 6})
                    node.supervise(now=node.clock_now + node.command_timeout_us)
                    self.assertEqual(node._angle_applied[other], [-1] * 6)
                    self.assertEqual(node._angle_applied[side],
                                     [500, 1000, 1000, 1000, 1000, 1000] if count == 6 else [-1] * 6)
                    state = node.make_state(node.clock_now)["hands"][side]["fault_protection"]
                    self.assertEqual(state["fault_count"], count)
                    self.assertEqual(state["open_requested"], count == 6)


    def test_fault_counts_never_accumulate_across_samples_or_hands(self):
        node, _ = self.make_node()
        self.hold(node)
        for index in range(6):
            codes = [0] * 6
            codes[index] = 5
            self.tick(node, errors=codes)
            self.assertEqual(node._angle_applied, {side: [-1] * 6 for side in module.SIDES})
            self.assertEqual([len(node.fault_channels[s]) for s in module.SIDES], [1, 1])
        self.tick(node, errors={"left": [5, 5, 5, 0, 0, 0], "right": [0, 0, 0, 5, 5, 5]})
        self.assertFalse(node._fault_open_sides)
        self.assertTrue(all(c.phase == "failed" for channels in node.closing_holds.values() for c in channels))
        self.assertEqual([len(node.fault_channels[s]) for s in module.SIDES], [3, 3])


    def test_supervision_observes_new_six_channel_fault_without_commands(self):
        node, _ = self.make_node()
        self.hold(node)
        self.tick(node, errors={"left": [0, 5, 0, 0, 0, 0], "right": [0] * 6})
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertFalse(node.command_valid)
        self.assertTrue(node._safe_applied)
        node.clock_now += 200000
        old = node.snapshots["left"]
        node.snapshots["left"] = module.Snapshot(
            data={**old.data, "err": [5] * 6}, sample_mono_us=node.clock_now)
        node.supervise(now=node.clock_now)
        self.assertEqual(node._angle_applied["left"], [500, 1000, 1000, 1000, 1000, 1000])
        self.assertEqual(node._angle_applied["right"], [-1] * 6)
        # A new valid target can close again once all configured hands recover.
        self.tick(node)
        self.assertEqual(node.hold_control_error, "")
        self.assertFalse(node._fault_open_sides)
        self.assertEqual(node._angle_applied["left"], [400, 0, 0, 0, 0, 0])


    def test_stale_failed_read_and_future_feedback_cannot_authorize_opening(self):
        for invalid in ("stale", "read_failed", "future"):
            with self.subTest(invalid=invalid):
                node, _ = self.make_node()
                self.hold(node)
                self.tick(node, errors=[0, 5, 0, 0, 0, 0])
                old = node.snapshots["left"]
                stamp = node.clock_now - node.feedback_timeout_us if invalid == "stale" else (
                    node.clock_now + 1 if invalid == "future" else node.clock_now)
                node.snapshots["left"] = module.Snapshot(
                    data={**old.data, "err": [5] * 6}, sample_mono_us=stamp,
                    error="read failed" if invalid == "read_failed" else "")
                node.supervise(now=node.clock_now)
                self.assertEqual(node._angle_applied["left"], [-1] * 6)
                self.assertEqual(node.fault_channels["left"], {})
                self.assertFalse(node._fault_open_sides)


    def test_six_fault_open_write_failure_is_retried_without_opening_other_hand(self):
        node, links = self.make_node()
        self.hold(node)
        links["left"].fail_writes = True
        with self.assertRaises(RuntimeError):
            self.tick(node, errors={"left": [5] * 6, "right": [0] * 6})
        self.assertFalse(any(c.args[0] == "fault_safe_pose_applied" for c in node.diag.emit.call_args_list))
        links["left"].fail_writes = False
        node.supervise(now=node.clock_now)
        self.assertEqual(node._angle_applied["left"], [500, 1000, 1000, 1000, 1000, 1000])
        self.assertEqual(node._angle_applied["right"], [-1] * 6)


    def test_device_fault_is_still_reported_during_grasp_motion(self):
        node, _ = self.make_node()
        self.tick(node, errors=[0, 2, 0, 0, 0, 0])
        self.assertIn("device error 2", node.hold_control_error)


    def test_logged_startup_current_words_are_signed_and_transient(self):
        node, _ = self.make_node()
        opened = [0, 1, 1, 1, 1, 1]
        for raw, signed in ((65408, -128), (65367, -169), (65334, -202), (235, 235), (0, 0)):
            self.tick(node, target=opened, currents=[raw, 0, 0, 0, 0, 0], temps=[40] * 6)
            self.assertEqual(node.hold_control_error, "")
            state = node.make_state(node.clock_now)
            self.assertTrue(state["valid"])
            self.assertEqual(state["hands"]["right"]["current"][0], signed)
            self.assertEqual(state["hands"]["right"]["current_register_raw"][0], raw)


    def test_device_error_is_decoded_and_throttled(self):
        node, _ = self.make_node()
        node.diag = module.DiagnosticLog("json")
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=node.clock_now):
            self.tick(node, errors=[5, 0, 0, 0, 0, 0])
            for _ in range(3):
                node.report_diagnostics(node.make_state(node.clock_now), node.clock_now)
        records = [r for r in NodeTests.diagnostic_records(output) if r["event"] == "device_error"]
        self.assertEqual(len(records), 2)  # One per hand, not one per publish tick.
        channel = records[0]["channels"][0]
        self.assertEqual(channel["channel_name"], "thumb_rotation")
        self.assertEqual(channel["error_names"], ["stall", "overcurrent"])
        self.assertEqual((channel["current_raw"], channel["force_raw"], channel["temperature_c"]), (10, 100, 35))
        self.assertIn("[error]", output.getvalue())


    def test_normal_holding_has_no_periodic_health_output_in_either_format(self):
        node, _ = self.make_node()
        self.hold(node)
        for log_format in ("text", "json"):
            node.diag = module.DiagnosticLog(log_format)
            output = io.StringIO()
            with patch("sys.stderr", output):
                for _ in range(10):
                    self.tick(node)
                    node.report_diagnostics(node.make_state(node.clock_now), node.clock_now)
            self.assertEqual(output.getvalue(), "")


    def test_load_values_do_not_gate_hold_or_generate_unloading(self):
        node, links = self.make_node()
        for _ in range(4):
            self.tick(node, forces=[0] * 6, currents=[2500] * 6, temps=[144] * 6)
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual(node._angle_applied["left"], [-1] * 6)
        before = {side: len(link.writes) for side, link in links.items()}
        for _ in range(100):
            self.tick(node, forces=[4117] * 6, currents=[2500] * 6, temps=[144] * 6)
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual({side: len(link.writes) for side, link in links.items()}, before)



if __name__ == "__main__":
    unittest.main()
