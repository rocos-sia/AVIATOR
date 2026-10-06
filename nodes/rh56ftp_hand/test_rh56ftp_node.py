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

    def test_device_error_and_unloading_are_readable(self):
        log = module.DiagnosticLog()
        output = io.StringIO()
        with patch("sys.stderr", output):
            log.emit("device_error", side="right", channels=[{
                "channel_name": "thumb_rotation", "code": 37,
                "error_names": ["stall", "overcurrent"], "unknown_bits": 32,
                "current_raw": -128, "force_raw": 301, "temperature_c": 40,
                "actual_raw": 483, "requested_raw": 500, "applied_raw": -1}], command_valid=True)
            log.emit("hold_phase_changed", side="left", channel_name="index", phase="moving",
                     actual_raw=400, release_target_raw=405, steps=1, current_raw=128, force_raw=301,
                     limits={"current_limit": 100, "force_limit": 300})
            log.emit("hold_control_failed", side="left", channel_name="index",
                     reason="temperature limit reached", temperature_c=50, limits={"temperature_limit": 50})
        text = output.getvalue()
        for expected in ("右手 设备故障", "拇指侧摆: 堵转、过流、未知故障位 32", "电流=-128",
                         "左手食指 负载过高，开始减载", "减载目标=405", "电流上限=100",
                         "温度达到上限", "温度上限=50", "[warning]", "[error]",
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

    def test_hold_drift_console_shows_stop_target_and_throttles_repeats(self):
        log = module.DiagnosticLog()
        output = io.StringIO()
        with patch("sys.stderr", output), patch.object(module, "monotonic_us", return_value=1000000):
            for _ in range(3):
                log.emit("holding_position_drift", key="hold_drift:right:2", side="right", channel_name="index",
                         held_raw=218, actual_raw=800, hold_drift_raw=582, applied_raw=-1,
                         current_raw=0, force_raw=27, steps=0)
        self.assertEqual(len(output.getvalue().splitlines()), 1)
        for expected in ("[warning]", "右手食指 保持位置发生偏移", "记录保持位置=218", "当前位置=800",
                         "位置偏移=582", "最近成功写入目标=-1", "力=27", "减载步数=0"):
            self.assertIn(expected, output.getvalue())

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
        self.assertEqual(links["left"].writes[-1], [-1] * 5 + [None])
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
        # Changing a closing target must not restart a stopped bending finger.
        self.drive_closing(node, target=[.2] * 6)
        self.assertEqual(links["left"].writes[-1], [-1] * 5 + [None])
        self.drive_closing(node, duration_ms=0, target=[.1] * 6)
        self.assertEqual(links["left"].writes[-1], [None] * 5 + [100])
        self.drive_closing(node, duration_ms=0, target=[.3] * 6)
        self.assertEqual(links["left"].writes[-1], [None] * 5 + [300])
        self.assertTrue(all(c.held_raw == 400 for c in node.closing_holds["left"][1:]))
        self.drive_closing(node, duration_ms=0, target=[.6] * 6)
        self.assertEqual(links["left"].writes[-1], [600] * 6)
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_moving_fingers_continue_while_stalled_fingers_hold_independently(self):
        node, links = self.make_node()
        self.drive_closing(node, position=lambda ms, side: (
            [400, 400, 700 - ms // 20, 400, 400, 400] if side == "left" else [400] * 6))
        self.assertEqual(links["left"].writes[-1], [-1, -1, -1, None, -1, None])
        self.assertEqual(links["right"].writes[-1], [-1] * 5 + [None])

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
        self.assertEqual(links["left"].writes[-1], [-1] * 5 + [None])
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
        self.assertEqual(links["left"].writes[-1], [1000, 1000, 1000, 1000, 1000, 500])
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_command_gap_resets_but_changing_closing_target_keeps_stall_window(self):
        node, links = self.make_node()
        self.drive_closing(node, duration_ms=4980)
        node.clock_now += node.command_timeout_us
        self.drive_closing(node, duration_ms=120)
        self.assertEqual(links["left"].writes[-1], [0] * 6)
        self.drive_closing(node, duration_ms=4980, target=[.1] * 6)
        self.assertEqual(links["left"].writes[-1], [-1] * 5 + [None])

    def test_grasp_mode_also_holds_bending_channels(self):
        node, links = self.make_node()
        self.drive_closing(node)
        msg = command(node, sequence=node.guard.last_sequence + 1)
        msg["mode"] = "GRASP_SETPOINT"
        msg["hands"] = {side: {"grasp": {"closure": 1}} for side in module.SIDES}
        node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)
        self.assertEqual(links["left"].writes[-1], [-1] * 5 + [None])

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
                                           ("force", [500] * 6),
                                           ("angle", [1000, 1000, 1000, 1000, 1000, 500])])
        node.handle_command("hand.command", json.dumps(command(node)), now=node.clock_now)
        node.supervise(now=node.clock_now + node.command_timeout_us)
        node._safe_pose()  # Exit pose uses the same settings.
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
            self.assertEqual(link.writes, [[1000, 1000, 1000, 1000, 1000, 500]])
            self.assertIn(("force", [500] * 6), link.setting_writes)

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
        self.assertEqual([r["event"] for r in records], ["configuration"])
        self.assertEqual(records[0]["configured_mode"], 0)
        self.assertEqual(records[0]["hold_control"]["current_limit"], [100] * 6)
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


class AdaptiveHoldTests(unittest.TestCase):
    def make_node(self, **overrides):
        policy = dict(enabled=True, current_limit=100, force_limit=500,
                      min_force=20, temperature_limit=60, close_timeout_ms=300,
                      overload_ms=60, settle_ms=60, move_timeout_ms=120)
        policy.update(overrides)
        links = {side: FakeLink() for side in module.SIDES}
        node = module.Rh56FtpNode(links, clock_id="test-clock", closing_hold_ms=60, hold_control=policy)
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
            # Rotation is already at target for the adaptive-hold scenarios.
            msg["hands"][side]["drive_position_normalized"] = target or [.4, 0, 0, 0, 0, 0]
        node.handle_command("hand.command", json.dumps(msg), now=node.clock_now)

    def hold(self, node):
        for _ in range(4):
            self.tick(node)
        self.assertEqual(node.closing_holds["left"][1].phase, "holding")

    def overload(self, node):
        for _ in range(4):
            self.tick(node, forces={"left": [100, 600, 100, 100, 100, 100], "right": [100] * 6})

    def test_stable_contact_required_and_closing_has_a_deadline(self):
        node, _ = self.make_node()
        for _ in range(17):
            self.tick(node, forces=[0] * 6)
        self.assertIn("closing timeout", node.hold_control_error)
        self.assertFalse(node.make_state(node.clock_now)["valid"])
        self.assertFalse(any(c.held_raw is not None for c in node.closing_holds["left"]))

    def test_release_uses_actual_position_and_keeps_old_target_latched(self):
        node, links = self.make_node()
        self.hold(node)
        self.overload(node)
        c = node.closing_holds["left"][1]
        self.assertEqual((c.phase, c.release_target, c.steps), ("moving", 405, 1))
        self.assertEqual(links["left"].writes[-1], [None, None, None, None, 405, None])
        self.tick(node, positions=[400, 405, 400, 400, 400, 400])
        self.assertEqual(node.closing_holds["left"][1].phase, "settling")
        self.assertEqual(links["left"].writes[-1], [None, None, None, None, -1, None])
        for _ in range(5):
            self.tick(node, positions=[400, 405, 400, 400, 400, 400])
        self.assertEqual(node.closing_holds["left"][1].phase, "holding")
        self.assertEqual(node.closing_holds["left"][1].steps, 1)
        self.assertEqual(node.hold_control_error, "")
        self.tick(node, target=[1] * 6)
        self.assertEqual(links["left"].writes[-1], [1000] * 6)
        self.assertTrue(all(c.held_raw is None for c in node.closing_holds["left"]))

    def test_only_one_channel_moves_across_both_hands(self):
        node, _ = self.make_node()
        self.hold(node)
        for _ in range(4):
            self.tick(node, forces=[100] + [600] * 5)
        active = [(side, i) for side in module.SIDES for i, c in enumerate(node.closing_holds[side])
                  if c.phase == "moving"]
        self.assertEqual(active, [("left", 1)])

    def test_logged_partial_release_stops_observes_then_increases_target(self):
        node, _ = self.make_node(force_limit=300)
        positions = {"left": [400] * 6, "right": [400, 63, 400, 400, 400, 400]}
        forces = {"left": [100] * 6, "right": [100, 472, 100, 100, 100, 100]}
        for _ in range(7):
            self.tick(node, positions=positions, forces=forces, currents=[0] * 6)
        c = node.closing_holds["right"][1]
        self.assertEqual((c.phase, c.release_target, c.steps), ("moving", 68, 1))
        positions["right"][1] = 64
        forces["right"][1] = 454
        for _ in range(6):
            self.tick(node, positions=positions, forces=forces, currents=[0] * 6)
        c = node.closing_holds["right"][1]
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual((c.phase, c.release_retry_target, c.steps), ("settling", 68, 1))
        self.assertEqual(node._angle_applied["right"][1], -1)
        self.assertTrue(node.make_state(node.clock_now)["valid"])
        # Repeated old samples cannot complete observation or restart unloading.
        self.tick(node, refresh=False)
        self.assertEqual(node.closing_holds["right"][1].phase, "settling")
        for _ in range(5):
            self.tick(node, positions=positions, forces=forces, currents=[0] * 6)
        self.assertEqual(node.closing_holds["right"][1].release_target, 73)
        self.assertEqual(node.closing_holds["right"][1].steps, 2)
        positions["right"][1] = 73
        forces["right"][1] = 200
        for _ in range(8):
            self.tick(node, positions=positions, forces=forces, currents=[0] * 6)
        c = node.closing_holds["right"][1]
        self.assertEqual((c.phase, c.steps, c.release_retry_target), ("holding", 2, None))
        self.assertEqual(node.hold_control_error, "")

    def test_motionless_release_retries_are_bounded_by_steps_and_distance(self):
        for limits in (dict(max_steps=3), dict(max_release_raw=15)):
            with self.subTest(limits=limits):
                node, _ = self.make_node(**limits)
                self.hold(node)
                forces = {"left": [100, 600, 100, 100, 100, 100], "right": [100] * 6}
                for _ in range(50):
                    self.tick(node, forces=forces)
                    if node.hold_control_error:
                        break
                self.assertIn("release step/distance budget exhausted", node.hold_control_error)
                attempted = [c.kwargs["release_target_raw"] for c in node.diag.emit.call_args_list
                             if c.args[0] == "hold_phase_changed" and c.kwargs["phase"] == "moving"]
                self.assertEqual(attempted, [405, 410, 415])
                self.assertEqual(node._angle_applied["left"], [-1] * 6)
                node.supervise(now=node.clock_now + node.command_timeout_us)
                self.assertEqual(node._angle_applied["left"], [500, 1000, 1000, 1000, 1000, 1000])

    def test_incomplete_release_recovered_load_does_not_retry(self):
        node, _ = self.make_node()
        self.hold(node)
        self.overload(node)
        for _ in range(15):
            self.tick(node)  # Position remains short, but the load has recovered.
        c = node.closing_holds["left"][1]
        self.assertEqual((c.phase, c.steps, c.release_retry_target), ("holding", 1, None))
        self.assertEqual(node.hold_control_error, "")

    def test_release_unexpected_direction_or_overshoot_still_fails(self):
        for actual in (397, 408):
            with self.subTest(actual=actual):
                node, _ = self.make_node()
                self.hold(node)
                self.overload(node)
                for _ in range(6):
                    self.tick(node, positions=[400, actual, 400, 400, 400, 400])
                self.assertIn("release motion outside expected range", node.hold_control_error)

    def test_hysteresis_and_duplicate_samples(self):
        node, _ = self.make_node()
        self.hold(node)
        self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        for _ in range(8):
            self.tick(node, refresh=False)
        self.assertEqual(node.closing_holds["left"][1].steps, 0)
        # Recovery clears the high-load episode even though ample wall time passed.
        self.tick(node)
        self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        for _ in range(3):
            self.tick(node, forces=[100, 450, 100, 100, 100, 100])
        self.assertEqual(node.closing_holds["left"][1].steps, 1)

    def test_adjustment_timeout_and_fault_latch(self):
        node, links = self.make_node(adjust_timeout_ms=120)
        self.hold(node)
        self.overload(node)
        for _ in range(6):
            self.tick(node)
        self.assertIn("adjustment time budget exhausted", node.hold_control_error)
        state = node.make_state(node.clock_now)
        self.assertFalse(state["valid"])
        self.assertEqual(state["hold_control_error"], node.hold_control_error)
        self.tick(node, target=[1] * 6)
        self.assertTrue(node.hold_control_error)
        self.assertEqual(node._angle_applied["left"], [-1] * 6)
        # Protective opening does not clear the fault or resume normal grasping.
        self.tick(node, valid=False)
        self.assertEqual(node._angle_applied["left"], [500, 1000, 1000, 1000, 1000, 1000])
        self.assertTrue(node.hold_control_error)
        node.connect()
        self.assertEqual(node.hold_control_error, "")

    def test_low_force_temperature_device_error_and_thumb_load(self):
        cases = [(dict(forces=[100, 1, 100, 100, 100, 100]), "insufficient holding force", 4),
                 (dict(temps=[60, 35, 35, 35, 35, 35]), "temperature limit", 1),
                 (dict(errors=[0, 0, 2, 0, 0, 0]), "device error 2", 1),
                 (dict(currents=[150, 10, 10, 10, 10, 10]), "outside adaptive hold", 4)]
        for kwargs, error, count in cases:
            with self.subTest(error=error):
                node, _ = self.make_node()
                self.hold(node)
                for _ in range(count):
                    self.tick(node, **kwargs)
                self.assertIn(error, node.hold_control_error)

    def test_current_alone_can_trigger_release(self):
        node, _ = self.make_node()
        self.hold(node)
        for _ in range(4):
            self.tick(node, currents=[10, 150, 10, 10, 10, 10])
        self.assertEqual(node.closing_holds["left"][1].release_target, 405)

    def test_hold_drift_precedes_low_force_fault_without_software_open_command(self):
        node, links = self.make_node(min_force=50, overload_ms=1000)
        positions = {"left": [400] * 6, "right": [400, 400, 218, 400, 400, 400]}
        for _ in range(4):
            self.tick(node, positions=positions)
        before = len(links["right"].writes)
        positions["right"][2] = 800
        # Drift can occur while the load is still above the low-force threshold.
        self.tick(node, positions=positions)
        events = [c.kwargs for c in node.diag.emit.call_args_list if c.args[0] == "holding_position_drift"]
        self.assertEqual((events[0]["held_raw"], events[0]["actual_raw"], events[0]["hold_drift_raw"]), (218, 800, 582))
        self.assertEqual(events[0]["applied_raw"], -1)
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual(len(links["right"].writes), before)
        forces = {"left": [100] * 6, "right": [100, 100, 27, 100, 100, 100]}
        for _ in range(50):
            self.tick(node, positions=positions, forces=forces)
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual(node.closing_holds["right"][2].steps, 0)
        self.tick(node, positions=positions, forces=forces)
        self.assertIn("insufficient holding force", node.hold_control_error)
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertEqual(node._angle_applied["right"], [500, 1000, 1000, 1000, 1000, 1000])

    def test_grasp_motion_skips_hold_limits_even_when_other_fingers_are_held(self):
        node, _ = self.make_node(close_timeout_ms=10000)
        for step in range(15):
            self.tick(node, positions=[400, 800 - step * 15, 400, 400, 400, 400],
                      currents=[150] * 6, forces=[600] * 6, temps=[65] * 6)
        self.assertEqual(node.closing_holds["left"][1].phase, "tracking")
        self.assertEqual(node.closing_holds["left"][2].phase, "holding")
        self.assertEqual(node.hold_control_error, "")
        self.assertTrue(node.make_state(node.clock_now)["valid"])
        for side in module.SIDES:
            self.assertTrue(all(c.steps == 0 and c.overload_since_us == 0 for c in node.closing_holds[side]))

    def test_rotation_motion_skips_its_own_limits_and_opening_disables_hold(self):
        node, _ = self.make_node(close_timeout_ms=10000)
        for step in range(15):
            self.tick(node, positions=[100 + step * 10, 400, 400, 400, 400, 400],
                      currents=[150, 10, 10, 10, 10, 10], forces=[600, 100, 100, 100, 100, 100],
                      temps=[65, 35, 35, 35, 35, 35])
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual(node.closing_holds["left"][0].overload_since_us, 0)
        # Held bending fingers remain monitored independently of rotation.
        self.tick(node, currents=[10, 150, 10, 10, 10, 10])
        self.assertEqual(node.closing_holds["left"][1].steps, 0)
        for _ in range(3):
            self.tick(node, currents=[10, 150, 10, 10, 10, 10])
        self.assertEqual(node.closing_holds["left"][1].steps, 1)
        for _ in range(5):
            self.tick(node, target=[1] * 6, currents=[150] * 6, forces=[600] * 6, temps=[65] * 6)
        self.assertEqual(node.hold_control_error, "")
        self.assertEqual(node._angle_applied["left"], [1000] * 6)

    def test_logged_rotation_offsets_do_not_block_bending_unload(self):
        node, _ = self.make_node()
        positions = {"left": [484, 296, 96, 91, 124, 192], "right": [456, 334, 80, 37, 30, 175]}
        target = [.5, 0, 0, 0, 0, 0]
        for _ in range(4):
            self.tick(node, positions=positions, target=target)
        self.assertTrue(all(c.held_raw is not None for c in node.closing_holds["left"][1:]))
        for _ in range(4):
            self.tick(node, positions=positions, target=target,
                      currents={"left": [0, 0, 400, 0, 0, 0], "right": [0] * 6},
                      forces={"left": [100, 100, 2325, 100, 100, 100], "right": [100] * 6})
        self.assertEqual(node.hold_control_error, "")
        channel = node.closing_holds["left"][2]
        self.assertEqual((channel.phase, channel.release_target, channel.steps), ("moving", 101, 1))
        self.assertEqual(node._angle_applied["left"][0], 500)
        self.assertTrue(any(c.args[0] == "hold_monitor_started" for c in node.diag.emit.call_args_list))

    def test_fault_opens_both_hands_on_timeout_invalid_command_and_exit(self):
        for path in ("watchdog", "invalid_command", "shutdown"):
            with self.subTest(path=path):
                node, links = self.make_node()
                self.hold(node)
                self.tick(node, errors=[0, 0, 32, 0, 0, 0])
                if path == "watchdog":
                    node.supervise(now=node.clock_now + node.command_timeout_us)
                elif path == "invalid_command":
                    self.tick(node, valid=False)
                else:
                    node._safe_pose()  # run_node's shutdown path uses the same method.
                for side, link in links.items():
                    self.assertEqual(node._angle_applied[side], [500, 1000, 1000, 1000, 1000, 1000])
                    self.assertEqual(link.writes[-1], [1000, 1000, 1000, 1000, 1000, 500])
                    self.assertTrue(all(c.held_raw is None for c in node.closing_holds[side]))
                self.assertFalse(node.command_valid)
                self.assertIn("device error 32", node.hold_control_error)
                self.assertFalse(node.make_state(node.clock_now)["valid"])

    def test_fault_open_write_failure_is_retried(self):
        node, links = self.make_node()
        self.hold(node)
        node.hold_control_error = "left[2]: device error 32"
        links["left"].fail_writes = True
        node.supervise(now=node.clock_now + node.command_timeout_us)
        self.assertFalse(node._safe_applied)
        links["left"].fail_writes = False
        node.supervise(now=node.clock_now + node.command_timeout_us + 1)
        self.assertTrue(node._safe_applied)
        self.assertEqual(node._angle_applied, {side: [500, 1000, 1000, 1000, 1000, 1000] for side in module.SIDES})

    def test_device_fault_is_still_reported_during_grasp_motion(self):
        node, _ = self.make_node()
        self.tick(node, errors=[0, 2, 0, 0, 0, 0])
        self.assertIn("device error 2", node.hold_control_error)

    def test_logged_startup_current_words_are_signed_and_transient(self):
        node, _ = self.make_node(overload_ms=1000)
        opened = [0, 1, 1, 1, 1, 1]
        for raw, signed in ((65408, -128), (65367, -169), (65334, -202), (235, 235), (0, 0)):
            self.tick(node, target=opened, currents=[raw, 0, 0, 0, 0, 0], temps=[40] * 6)
            self.assertEqual(node.hold_control_error, "")
            state = node.make_state(node.clock_now)
            self.assertTrue(state["valid"])
            self.assertEqual(state["hands"]["right"]["current"][0], signed)
            self.assertEqual(state["hands"]["right"]["current_register_raw"][0], raw)

    def test_negative_current_magnitude_still_triggers_protection(self):
        for value in (65536 - 150, -150):
            with self.subTest(current=value):
                node, _ = self.make_node()
                self.hold(node)
                for _ in range(4):
                    self.tick(node, currents=[10, value, 10, 10, 10, 10])
                self.assertEqual(node.closing_holds["left"][1].release_target, 405)
                node, _ = self.make_node()
                self.hold(node)
                for _ in range(4):
                    self.tick(node, currents=[value, 0, 0, 0, 0, 0])
                self.assertIn("persistent load outside adaptive hold", node.hold_control_error)
                event = next(c.kwargs for c in node.diag.emit.call_args_list if c.args[0] == "hold_control_failed")
                self.assertEqual((event["current_raw"], event["current_abs_raw"], event["current_register_raw"]),
                                 (-150, 150, 65386))

    def test_signed_current_range_does_not_mask_corrupt_feedback(self):
        for value in (2001, -2001, 65536 - 2001, 32768, 65536, -65536):
            with self.subTest(current=value):
                node, _ = self.make_node()
                self.tick(node, currents=[value, 0, 0, 0, 0, 0])
                self.assertIn("invalid load feedback", node.hold_control_error)

    def test_logged_force_4014_identifies_range_fault_without_twelve_motor_faults(self):
        node, _ = self.make_node()
        node.diag = module.DiagnosticLog()
        output = io.StringIO()
        with patch("sys.stderr", output):
            self.tick(node, forces=[100, 4014, 100, 100, 100, 100],
                      currents=[10, 65473, 10, 10, 10, 10], temps=[32] * 6)
        self.assertIn("force_raw=4014 outside [-4000,4000]", node.hold_control_error)
        self.assertIn("电流=-63", output.getvalue())
        self.assertIn("受力原始值=4014 超出手册范围 [-4000,4000]", output.getvalue())
        self.assertIn("全部已连接通道停止", output.getvalue())
        self.assertEqual(len(output.getvalue().splitlines()), 1)
        self.assertEqual(node._angle_applied, {side: [-1] * 6 for side in module.SIDES})
        self.assertTrue(all(c.phase == "failed" for channels in node.closing_holds.values() for c in channels))

    def test_force_range_and_holding_limit_are_distinct_during_motion(self):
        for value in (4000, -4000, 4014, -4014):
            with self.subTest(force=value):
                node, _ = self.make_node(force_limit=300)
                self.tick(node, forces=[100, value, 100, 100, 100, 100])
                if abs(value) <= 4000:
                    self.assertEqual(node.hold_control_error, "")
                else:
                    self.assertIn(f"force_raw={value}", node.hold_control_error)
                self.assertEqual(node.make_state(node.clock_now)["hands"]["left"]["force"][1], value)

    def test_stale_feedback_cancels_release_and_requires_new_evidence(self):
        node, _ = self.make_node(move_timeout_ms=1000)
        self.hold(node)
        self.overload(node)
        for _ in range(26):
            self.tick(node, refresh=False)
        c = node.closing_holds["left"][1]
        self.assertEqual((c.phase, c.release_target), ("holding", None))
        self.assertEqual(node._angle_applied["left"], [-1] * 6)
        self.assertFalse(node.make_state(node.clock_now)["valid"])
        self.tick(node)
        self.assertEqual(node.closing_holds["left"][1].steps, 1)
        self.assertEqual(node.hold_control_error, "")

    def test_failed_write_does_not_commit_release_or_ack(self):
        node, links = self.make_node()
        self.hold(node)
        for _ in range(3):
            self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        sequence = node.guard.last_sequence
        links["left"].fail_writes = True
        with self.assertRaises(RuntimeError):
            self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        self.assertEqual(node.guard.last_sequence, sequence)
        self.assertEqual(node.closing_holds["left"][1].steps, 0)
        self.assertIn("write failed", node.hold_control_error)
        links["left"].fail_writes = False
        self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        self.assertEqual(node.closing_holds["left"][1].steps, 0)
        self.assertEqual(node._angle_applied["left"], [-1] * 6)

    def test_distance_step_and_time_budgets(self):
        for changes, message in [(dict(max_release_raw=5), "budget exhausted"),
                                 (dict(max_steps=1), "budget exhausted"),
                                 (dict(adjust_timeout_ms=40), "time budget")]:
            with self.subTest(changes=changes):
                node, _ = self.make_node(**changes)
                self.hold(node)
                self.overload(node)
                # Unload to the actual target, but the same finger remains overloaded.
                for _ in range(12):
                    self.tick(node, positions=[400, 405, 400, 400, 400, 400],
                              forces={"left": [100, 600, 100, 100, 100, 100], "right": [100] * 6})
                self.assertIn(message, node.hold_control_error)

    def test_configuration_validation_and_startup_loading(self):
        invalid = [dict(current_limit=None), dict(enabled="true"), dict(unknown=1),
                   dict(current_limit=True), dict(force_limit=[100] * 5),
                   dict(temperature_limit=float("nan")), dict(recovery_ratio=1),
                   dict(release_tolerance_raw=5), dict(min_force=450), dict(max_steps=False)]
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ValueError):
                self.make_node(**config)
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "hand.yaml"
            config.write_text("hold_control:\n  enabled: true\n  current_limit: [100, 110, 120, 130, 140, 150]\n"
                              "  force_limit: 500\n  min_force: 20\n  temperature_limit: 60\n")
            with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                    patch.object(module, "run_node", return_value=0) as run:
                module.main(["--config", str(config)])
                self.assertEqual(run.call_args.args[0].hold_control.current_limit, (100, 110, 120, 130, 140, 150))
            config.write_text("hold_control:\n  enabled: true\n  temperature_limit: null\n")
            with patch.object(module, "load_handlink") as load, patch("sys.stderr"), self.assertRaises(SystemExit):
                module.main(["--config", str(config)])
            load.assert_not_called()

    def test_two_steps_then_recovery_stops_unloading(self):
        node, _ = self.make_node()
        self.hold(node)
        self.overload(node)
        force = {"left": [100, 600, 100, 100, 100, 100], "right": [100] * 6}
        for _ in range(7):
            self.tick(node, positions=[400, 405, 400, 400, 400, 400], forces=force)
        c = node.closing_holds["left"][1]
        self.assertEqual((c.steps, c.release_target), (2, 410))
        for _ in range(15):
            self.tick(node, positions=[400, 410, 400, 400, 400, 400])
        c = node.closing_holds["left"][1]
        self.assertEqual((c.phase, c.steps, c.held_raw), ("holding", 2, 410))
        self.assertEqual(node.hold_control_error, "")

    def test_opening_during_release_preempts_adaptation(self):
        node, links = self.make_node()
        self.hold(node)
        self.overload(node)
        self.tick(node, target=[1] * 6)
        self.assertEqual(links["left"].writes[-1], [1000] * 6)
        self.assertTrue(all(c.phase == "tracking" for c in node.closing_holds["left"]))

    def test_read_failure_stops_release_even_with_recent_cached_sample(self):
        node, _ = self.make_node()
        self.hold(node)
        self.overload(node)
        node.snapshots["right"].error = "read failed"
        self.tick(node, refresh=False)
        self.assertEqual(node._angle_applied["left"], [-1] * 6)
        self.assertFalse(node.make_state(node.clock_now)["valid"])

    def test_partial_dual_hand_write_cannot_retry_an_unaccounted_step(self):
        node, links = self.make_node()
        self.hold(node)
        for _ in range(3):
            self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        # Force a write to the other hand after the left release succeeds.
        node._angle_applied.pop("right")
        links["right"].fail_writes = True
        with self.assertRaises(RuntimeError):
            self.tick(node, forces=[100, 600, 100, 100, 100, 100])
        self.assertEqual(node._angle_applied["left"][1], 405)
        self.assertEqual(node.closing_holds["left"][1].steps, 0)
        links["right"].fail_writes = False
        self.tick(node, positions=[400, 405, 400, 400, 400, 400])
        self.assertEqual(node._angle_applied["left"], [-1] * 6)
        self.assertIn("write failed", node.hold_control_error)

    def test_repository_configuration_loads_configured_temperature(self):
        import yaml
        config = SCRIPT.parents[2] / "config/rh56ftp_hand.yaml"
        temperature = yaml.safe_load(config.read_text())["hold_control"]["temperature_limit"]
        limits = tuple(temperature) if isinstance(temperature, list) else (temperature,) * 6
        with patch.object(module, "load_handlink", return_value=("fake", 6000, FakeLink)), \
                patch.object(module, "run_node", return_value=0) as run:
            module.main(["--config", str(config)])
        node = run.call_args.args[0]
        self.assertTrue(node.hold_control.enabled)
        self.assertEqual(node.hold_control.temperature_limit, limits)
        node.clock_now = 1000000
        node.diag.emit = Mock()
        node.closing_hold_us = 60000
        self.hold(node)
        below = [limit - 1 for limit in limits]
        self.tick(node, temps=below)
        self.assertEqual(node.hold_control_error, "")
        reached = below.copy()
        reached[1] = limits[1]
        self.tick(node, temps=reached)
        self.assertIn("temperature limit reached", node.hold_control_error)
        self.assertFalse(node.make_state(node.clock_now)["valid"])

    def test_raw_defaults_integer_bounds_and_no_unit_conversion(self):
        policy = module.HoldControl.parse({})
        self.assertEqual((policy.current_limit, policy.force_limit, policy.min_force, policy.temperature_limit),
                         ((100,) * 6, (300,) * 6, (50,) * 6, (50,) * 6))
        for name, maximum in (("current_limit", 2000), ("force_limit", 3000),
                              ("min_force", 3000), ("temperature_limit", 100)):
            for value in (-1, maximum + 1, 1.5, True, None):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    module.HoldControl.parse({name: value})
        bounds = module.HoldControl.parse(dict(current_limit=2000, force_limit=3000, min_force=0))
        self.assertEqual(bounds.force_limit, (3000,) * 6)
        self.assertEqual(module.HoldControl.parse(dict(current_limit=0)).current_limit, (0,) * 6)
        with self.assertRaisesRegex(ValueError, "current_limit_ma"):
            module.HoldControl.parse({"current_limit_ma": 100})
        with self.assertRaisesRegex(ValueError, "temperature_limit_c"):
            module.HoldControl.parse({"temperature_limit_c": 50})
        limits = [45, 46, 47, 48, 49, 50]
        self.assertEqual(module.HoldControl.parse({"temperature_limit": limits}).temperature_limit, tuple(limits))
        node, _ = self.make_node()
        self.hold(node)
        # A signed force sample is compared by magnitude, directly in register counts.
        for _ in range(4):
            self.tick(node, forces={"left": [100, -600, 100, 100, 100, 100], "right": [100] * 6})
        self.assertEqual(node.closing_holds["left"][1].release_target, 405)

    def test_failure_log_captures_pre_fault_state_and_limits(self):
        node, _ = self.make_node(adjust_timeout_ms=120)
        self.hold(node)
        self.overload(node)
        for _ in range(6):
            self.tick(node)
        records = [c.kwargs for c in node.diag.emit.call_args_list if c.args[0] == "hold_control_failed"]
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertEqual((record["side"], record["channel"], record["channel_name"]), ("left", 1, "thumb_bend"))
        self.assertEqual(record["phase"], "moving")
        self.assertEqual((record["actual_raw"], record["release_target_raw"]), (400, 405))
        self.assertEqual(record["phase_age_ms"], 120)
        self.assertEqual(record["limits"]["move_timeout_ms"], 120)
        self.assertEqual(record["limits"]["current_limit"], 100)
        self.assertEqual(record["steps"], 1)
        self.assertIn("accepted_sequence", record["command"])
        json.dumps(record, allow_nan=False)

    def test_device_error_is_decoded_and_throttled_even_when_disabled(self):
        node, _ = self.make_node(enabled=False)
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


if __name__ == "__main__":
    unittest.main()
