"""Controller/API regression tests with in-memory devices; no TCP or SDK access."""
import copy
from email.message import Message
import io
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

import ui
import ui_backend as app
from calibration import CalibrationError, matrix_pose, pose_matrix


class FakeNodes:
    def __init__(self, root, log_dir, log):
        self.started = []
        self.stopped = 0

    def start(self, *args, **kwargs):
        self.started.append((args, kwargs))
        raise AssertionError("dry-run must not launch hardware nodes")

    def snapshot(self):
        return []

    def stop_all(self):
        self.stopped += 1


class FakeNative:
    def __init__(self, grasp, events):
        self.grasp, self.events = grasp, events
        self.arms = {s: dict(connected=False, dragging=False, drag_owned=False,
                            operation_state="disconnected") for s in ("left", "right")}
        self.angle, self.displacement = 0.0, -.085
        self.sample_count = 0
        self.closed = False

    def request(self, op, **fields):
        self.events.append((op, dict(fields)))
        if op == "connect":
            for arm in self.arms.values():
                arm.update(connected=True, operation_state="idle")
        elif op in ("drag_start", "drag_stop"):
            selected = self.arms if fields["side"] == "both" else (fields["side"],)
            for side in selected:
                self.arms[side].update(dragging=op == "drag_start", drag_owned=op == "drag_start")
        elif op == "disconnect":
            for arm in self.arms.values():
                arm.update(connected=False, dragging=False, drag_owned=False)
        elif op == "sample":
            self.sample_count += 1
            now = app.mono_us()
            state = dict(source="offline", frame="aircraft", sample_start_mono_us=now,
                         sample_mono_us=now, sample_end_mono_us=now)
            wheel = pose_matrix(dict(position=[0, 0, self.displacement],
                                     quaternion=[math.cos(self.angle / 2), 0, 0, math.sin(self.angle / 2)]))
            for side in ("left", "right"):
                target = pose_matrix(self.grasp["wheel_origin"]) @ wheel @ pose_matrix(self.grasp[side])
                flange = target @ np.linalg.inv(pose_matrix(self.grasp["tool"][side]))
                state[side] = dict(joint_position=[0, .2, 0, .3, 0, .4, 0], flange=matrix_pose(flange))
            return state
        elif op != "status":
            raise AssertionError(f"unexpected operation: {op}")
        return copy.deepcopy(self.arms)

    def close(self):
        self.events.append(("close", {}))
        for arm in self.arms.values():
            arm.update(connected=False, dragging=False, drag_owned=False)
        self.closed = True

    def interrupt(self):
        self.events.append(("interrupt", {}))


class OffsetDemoLink(app.DemoLink):
    def snapshot(self):
        result = super().snapshot()
        result["camera"]["raw"]["steering_wheel"].update(theta_rad=.2, translation_along_axis_m=.015)
        return result


class CalibrationControllerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.grasp_path = self.directory / "grasp.json"
        self.grasp_path.write_bytes((app.ROOT / "config/grasp.json").read_bytes())
        self.grasp = json.loads(self.grasp_path.read_text())
        robot = yaml.safe_load((app.ROOT / "config/robot.yaml").read_bytes())
        robot.update(grasp=str(self.grasp_path), urdf=str(app.ROOT / "models/urdf/aviator.urdf"),
                     posture=str(app.ROOT / "config/posture.json"))
        robot_path = self.directory / "robot.yaml"
        robot_path.write_text(yaml.safe_dump(robot))
        for name in ("system.yaml", "camera.yaml", "rh56ftp_hand.yaml"):
            (self.directory / name).write_bytes((app.ROOT / "config" / name).read_bytes())
        helper = self.directory / "fake-native"
        helper.write_text("#! unused test fixture\n")
        helper.chmod(0o700)
        self.options = SimpleNamespace(dry_run=True, robot_config=robot_path,
                                       system_config=self.directory / "system.yaml",
                                       camera_config=self.directory / "camera.yaml",
                                       hand_config=self.directory / "rh56ftp_hand.yaml",
                                       session_helper=helper, log_dir=self.directory / "logs",
                                       camera_id="cockpit", lease_timeout=10)
        self.events = []
        self.instances = []

        def factory(executable, robot_config, dry_run, log_dir, log):
            self.assertTrue(dry_run)
            native = FakeNative(self.grasp, self.events)
            self.instances.append(native)
            return native

        self.controller = app.CalibrationController(self.options, native_factory=factory,
                                                    nodes_factory=FakeNodes,
                                                    conflict_check=lambda: [])
        self.controller.args.samples = 3
        self.controller.args.interval_ms = 10
        self.controller.args.timeout_s = 2

    def tearDown(self):
        self.controller.close()
        self.temporary.cleanup()

    def wait_for(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            status = self.controller.snapshot()
            if predicate(status):
                return status
            time.sleep(.005)
        self.fail("Controller did not reach expected state: " + str(self.controller.snapshot()))

    def action(self, action, **fields):
        self.controller.submit(dict(action=action, **fields))
        return self.wait_for(lambda s: not s["busy"])

    def prepare(self, source="manual"):
        status = self.action("prepare", wheel_source=source)
        self.assertTrue(status["prepared"], status["error"])
        self.assertEqual(status["phase"], "ready")
        return self.instances[-1]

    def test_construction_and_page_poll_do_not_activate_devices(self):
        self.assertEqual(self.instances, [])
        self.controller.heartbeat()
        status = self.controller.snapshot()
        self.assertFalse(status["prepared"] or status["can_sample"])
        self.assertEqual(self.controller.nodes.started, [])
        with self.assertRaisesRegex(CalibrationError, "准备"):
            self.controller.submit(dict(action="hand", side="both", pose="close"))
        self.assertEqual(self.instances, [])
        self.prepare()
        self.assertEqual(len(self.instances), 1)
        self.assertEqual(self.events[0][0], "connect")
        self.assertEqual(self.controller.nodes.started, [])

    def test_left_right_hand_targets_preserve_other_side_and_thumb(self):
        self.prepare()
        status = self.action("hand", side="left", pose="close")
        self.assertEqual(status["hand"]["targets"]["left"], [.5, 0, 0, 0, 0, 0])
        self.assertEqual(status["hand"]["targets"]["right"], [.5, 1, 1, 1, 1, 1])
        status = self.action("hand", side="right", pose="close")
        self.assertEqual(status["hand"]["targets"]["right"], [.5, 0, 0, 0, 0, 0])
        status = self.action("hand", side="left", pose="open")
        self.assertEqual(status["hand"]["targets"]["left"], [.5, 1, 1, 1, 1, 1])
        self.assertEqual(status["hand"]["targets"]["right"], [.5, 0, 0, 0, 0, 0])

    def test_drag_side_and_sampling_rejects_any_dragging_arm(self):
        native = self.prepare()
        status = self.action("drag_start", side="left")
        self.assertTrue(status["arms"]["left"]["dragging"])
        self.assertFalse(status["arms"]["right"]["dragging"] or status["can_sample"])
        status = self.action("sample", angle_deg=0, displacement_m=-.085)
        self.assertIn("结束双臂拖动", status["error"])
        self.assertEqual(native.sample_count, 0)
        self.action("drag_stop", side="left")
        status = self.action("sample", angle_deg=0, displacement_m=-.085)
        self.assertEqual(status["phase"], "preview", status["error"])

    def test_manual_sampling_keeps_core_coordinates_and_recovers_tool(self):
        native = self.prepare()
        native.angle, native.displacement = math.radians(10), -.12
        before = self.grasp_path.read_bytes()
        status = self.action("sample", angle_deg=10, displacement_m=-.12)
        self.assertEqual(status["phase"], "preview", status["error"])
        report = status["result"]
        self.assertEqual(report["sample_count"], 3)
        self.assertAlmostEqual(report["wheel"]["mean_angle_rad"], math.radians(10))
        self.assertAlmostEqual(report["wheel"]["mean_displacement_m"], -.12)
        for side in ("left", "right"):
            np.testing.assert_allclose(pose_matrix(self.controller._updated["tool"][side]),
                                       pose_matrix(self.grasp["tool"][side]), atol=1e-9)
        self.assertEqual(self.grasp_path.read_bytes(), before)
        self.assertFalse(status["can_save"])
        self.assertEqual(len(list(self.options.log_dir.glob("preview-*.json"))), 1)

    def test_camera_sampling_converts_sign_offset_and_uses_independent_frames(self):
        with patch.object(app, "DemoLink", OffsetDemoLink):
            native = self.prepare("camera")
        native.angle, native.displacement = -.2, -.1
        self.controller.args.interval_ms = 50
        displayed = self.controller.snapshot()["camera"]
        self.assertAlmostEqual(displayed["angle_rad"], -.2)
        self.assertAlmostEqual(displayed["displacement_m"], -.1)
        status = self.action("sample")
        self.assertEqual(status["phase"], "preview", status["error"])
        report = status["result"]
        self.assertAlmostEqual(report["wheel"]["mean_angle_rad"], -.2)
        self.assertAlmostEqual(report["wheel"]["mean_displacement_m"], -.1)
        self.assertEqual(len({s["camera"]["sequence"] for s in report["samples"]}), 3)
        self.assertTrue(all(s["sync_gap_ms"] <= self.controller.args.max_sync_ms for s in report["samples"]))

    def test_stop_cancels_queued_hand_action_before_execution(self):
        native = self.prepare()
        entered, release = threading.Event(), threading.Event()
        link = self.controller.link

        def monitor_pause():
            entered.set()
            release.wait(timeout=2)

        with patch.object(self.controller, "_monitor", side_effect=monitor_pause), \
                patch.object(link, "set_hand", wraps=link.set_hand) as set_hand:
            self.assertTrue(entered.wait(timeout=1))
            try:
                self.controller.submit(dict(action="hand", side="both", pose="close"))
                self.controller.submit(dict(action="stop"))
            finally:
                release.set()
            status = self.wait_for(lambda s: not s["prepared"] and not s["busy"])
            self.assertEqual(status["phase"], "idle")
            self.assertFalse(any(call.args == ("both", "close") for call in set_hand.call_args_list))
        self.assertTrue(native.closed)
        self.assertEqual([event[0] for event in self.events][-2:], ["interrupt", "close"])

    def test_dry_run_write_is_rejected_at_submit_internal_and_http_api(self):
        self.prepare()
        status = self.action("sample", angle_deg=0, displacement_m=-.085)
        self.assertEqual(status["phase"], "preview", status["error"])
        before = self.grasp_path.read_bytes()
        with self.assertRaises(CalibrationError):
            self.controller.submit(dict(action="save"))
        with self.assertRaises(CalibrationError):
            self.controller._save()
        handler = object.__new__(ui.Handler)
        handler.server = SimpleNamespace(server_port=8766, token="test-token", controller=self.controller)
        handler.path = "/api/action"
        body = b'{"action":"save"}'
        handler.headers = Message()
        for name, value in {"Host": "127.0.0.1:8766", "Origin": "http://127.0.0.1:8766",
                            "X-Calibration-Token": "test-token", "Content-Length": str(len(body))}.items():
            handler.headers[name] = value
        handler.connection = Mock()
        handler.rfile = io.BytesIO(body)
        handler._reply = Mock()
        handler.do_POST()
        self.assertEqual(handler._reply.call_args.args[0], 409)
        self.assertEqual(self.grasp_path.read_bytes(), before)
        self.assertEqual(list(self.directory.glob("grasp.json.*.bak")), [])

    def test_config_change_prevents_sampling_before_native_reads(self):
        native = self.prepare()
        with self.options.robot_config.open("a") as stream:
            stream.write("\n# changed during session\n")
        status = self.action("sample", angle_deg=0, displacement_m=-.085)
        self.assertIn("配置发生改变", status["error"])
        self.assertIsNone(status["result"])
        self.assertEqual(native.sample_count, 0)

    def test_config_change_prevents_save_before_file_write(self):
        self.prepare()
        self.action("sample", angle_deg=0, displacement_m=-.085)
        before = self.grasp_path.read_bytes()
        with self.options.system_config.open("a") as stream:
            stream.write("\n# changed during session\n")
        # Exercise the real-save guard with already injected offline devices. No
        # prepare() call happens under this override and save_grasp is forbidden.
        with patch.object(self.options, "dry_run", False), patch.object(app, "save_grasp") as save:
            with self.assertRaisesRegex(CalibrationError, "配置发生改变"):
                self.controller._save()
            save.assert_not_called()
        self.assertEqual(self.grasp_path.read_bytes(), before)

    def test_cancel_during_final_arm_status_prevents_save(self):
        native = self.prepare()
        status = self.action("sample", angle_deg=0, displacement_m=-.085)
        self.assertEqual(status["phase"], "preview", status["error"])
        before = self.grasp_path.read_bytes()
        original_request = native.request

        def cancel_after_status(op, **fields):
            result = original_request(op, **fields)
            if op == "status":
                self.controller._cancel.set()
            return result

        # Serialize this direct guard test with the action worker, so its cleanup
        # cannot clear the injected cancellation before _save checks it.
        with self.controller.lock, patch.object(self.options, "dry_run", False), \
                patch.object(native, "request", side_effect=cancel_after_status), \
                patch.object(app, "save_grasp") as save:
            try:
                with self.assertRaisesRegex(CalibrationError, "取消"):
                    self.controller._save()
                save.assert_not_called()
            finally:
                self.controller._cancel.clear()
        self.assertEqual(self.grasp_path.read_bytes(), before)

    def test_new_session_displays_reloaded_grasp_path_and_robot_ips(self):
        self.prepare()
        status = self.action("stop")
        self.assertFalse(status["prepared"])
        new_grasp = self.directory / "replacement-grasp.json"
        new_grasp.write_bytes(self.grasp_path.read_bytes())
        robot = yaml.safe_load(self.options.robot_config.read_bytes())
        robot["grasp"] = new_grasp.name
        robot["rokae"].update(left_ip="192.0.2.11", right_ip="192.0.2.12")
        self.options.robot_config.write_text(yaml.safe_dump(robot))
        self.prepare()
        status = self.controller.snapshot()
        self.assertEqual(status["config"]["grasp"], str(new_grasp))
        self.assertEqual(status["config"]["left_ip"], "192.0.2.11")
        self.assertEqual(status["config"]["right_ip"], "192.0.2.12")
        self.assertEqual(self.controller.grasp_path, new_grasp)
        self.assertEqual(len(self.instances), 2)

    def test_browser_lease_expiry_cleans_drag_before_releasing_hands(self):
        native = self.prepare()
        self.action("drag_start", side="both")
        link = self.controller.link
        released = []
        original_stop = link.stop

        def stop_hand():
            released.append(all(not a["dragging"] for a in native.arms.values()) and native.closed)
            original_stop()

        with patch.object(link, "stop", side_effect=stop_hand):
            with self.controller.lock:
                self.controller._last_client = time.monotonic() - 11
            status = self.wait_for(lambda s: not s["prepared"] and not s["busy"])
        self.assertEqual(released, [True])
        self.assertEqual(link.poses, dict(left="open", right="open"))
        self.assertTrue(native.closed)
        self.assertGreaterEqual(self.controller.nodes.stopped, 1)
        self.assertTrue(any("心跳超时" in entry["text"] for entry in status["logs"]))


if __name__ == "__main__":
    unittest.main()
