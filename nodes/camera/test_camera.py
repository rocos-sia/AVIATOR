"""Hardware-free checks for the camera node's Logger wire contract."""

import json
import io
import re
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import cv2
import yaml

from detection_subscriber import fmt_pose
from detectors import DetectionResult, create_detector, pose_from_pnp
from main import DEFAULT_CONFIG, load_config, make_message, make_record_frame, parse_args
from recording_client import RecordingClient, camera_packet
from visualization import CameraVisualization, pose_lines, render_preview, visualization_options


class SensorFrame:
    def get_frame_number(self):
        return 42

    def get_timestamp(self):
        return 123.5

    def get_frame_timestamp_domain(self):
        return "hardware_clock"


def decode_packet(packet):
    """Decode the two length-delimited fields of CameraPacket v1."""
    def varint(offset):
        value, shift = 0, 0
        while True:
            byte = packet[offset]
            offset += 1
            value |= (byte & 127) << shift
            if byte < 128:
                return value, offset
            shift += 7

    fields = {}
    offset = 0
    while offset < len(packet):
        tag, offset = varint(offset)
        length, offset = varint(offset)
        fields[tag >> 3] = packet[offset:offset + length]
        offset += length
    return json.loads(fields[1]), fields[2]


class CameraNodeTest(unittest.TestCase):
    def test_visualization_defaults_overrides_and_headless(self):
        _, _, _, settings = load_config(DEFAULT_CONFIG)
        options = visualization_options(settings, parse_args([]))
        self.assertFalse(options["show"])
        self.assertTrue(options["print_pose"])
        self.assertEqual(visualization_options(settings, parse_args([
            "--show", "--print-pose", "--pose-print-interval", "0.2"])),
            dict(show=True, print_pose=True, print_interval_s=0.2))
        overrides = visualization_options(dict(show=True, print_pose=True),
                                           parse_args(["--no-show", "--no-print-pose"]))
        self.assertFalse(overrides["show"])
        self.assertFalse(overrides["print_pose"])
        for invalid in (0, -1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                visualization_options(dict(print_interval_s=invalid), parse_args([]))
        with patch.dict("os.environ", {}, clear=True):
            CameraVisualization(print_pose=True).check_available()
            with self.assertRaisesRegex(RuntimeError, "桌面"):
                CameraVisualization(show=True).check_available()

    def test_pose_diagnostics_units_throttle_and_target_loss(self):
        rvec, tvec = np.array([0., 0., np.pi / 2]), np.array([.1, -.2, .3])
        result = DetectionResult(status="TRACKING", pose=pose_from_pnp(rvec, tvec),
                                 rvec=rvec, tvec=tvec)
        text = " | ".join(pose_lines(result))
        self.assertIn("Position (m): X=0.1000 Y=-0.2000 Z=0.3000", text)
        self.assertIn("yaw=90.0", text)
        # Same singular-angle convention as Downloads/detect.py.
        singular = DetectionResult(status="TRACKING", pose=result.pose,
                                   rvec=np.array([0., np.pi / 2, 0.]))
        self.assertIn("pitch=90.0 yaw=0.0", " | ".join(pose_lines(singular)))
        class Detector:
            kind = "apriltag"
        output = io.StringIO()
        visual = CameraVisualization(print_pose=True)
        with redirect_stdout(output), patch("visualization.cv2.imshow") as show:
            visual.update(None, Detector(), result, 1, now=1.0)
            visual.update(None, Detector(), result, 2, now=1.1)
            visual.update(None, Detector(), result, 3, now=1.5)
            visual.update(None, Detector(), DetectionResult(), 4, now=1.6)
            visual.close()
            show.assert_not_called()
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 3)
        self.assertIn("No valid pose", lines[-1])
        self.assertNotIn("X=", lines[-1])
        # Disabled diagnostics must not touch GUI or pose processing.
        with patch("visualization.cv2.destroyWindow") as destroy:
            disabled = CameraVisualization()
            self.assertTrue(disabled.update(None, None, None, 1))
            disabled.close()
            destroy.assert_not_called()

    def test_preview_quit_and_window_close(self):
        class Detector:
            kind = "apriltag"
            def draw(self, image, result):
                pass
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        visual = CameraVisualization(show=True)
        with patch("visualization.cv2.namedWindow"), patch("visualization.cv2.imshow"), \
             patch("visualization.cv2.waitKey", return_value=ord("q")):
            self.assertFalse(visual.update(image, Detector(), DetectionResult(), 1))
        with patch("visualization.cv2.getWindowProperty", return_value=0), \
             patch("visualization.cv2.imshow") as show:
            self.assertFalse(visual.update(image, Detector(), DetectionResult(), 2))
            show.assert_not_called()
        with patch("visualization.cv2.destroyWindow") as destroy:
            visual.close()
            destroy.assert_called_once()

    def test_terminal_pose_panel_refreshes_in_place_and_clears_lost_values(self):
        class Terminal(io.StringIO):
            flushes = 0

            def isatty(self):
                return True

            def flush(self):
                self.flushes += 1

        class Detector:
            kind = "apriltag"

        output = Terminal()
        result = DetectionResult(status="TRACKING", pose=pose_from_pnp(
            np.array([0., 0., .2]), np.array([.1, -.2, .3])), rvec=np.array([0., 0., .2]))
        wheel = {"valid": True, "theta_rad": -.25, "translation_along_axis_m": .04,
                 "translation_vector_m": [.01, -.02, .03]}
        visual = CameraVisualization(print_pose=True)
        with redirect_stdout(output), patch("visualization.shutil.get_terminal_size") as size:
            size.return_value.columns = 120
            visual.update(None, Detector(), result, 1, now=1., steering_wheel=wheel)
            visual.update(None, Detector(), result, 2, now=1.6, steering_wheel=wheel)
            before_loss = output.getvalue()
            self.assertIn("Tag quaternion (xyzw)", before_loss)
            self.assertIn("Wheel rotation (rad): -0.250000", before_loss)
            self.assertIn("Along axis (m): +0.040000", before_loss)
            self.assertIn("Relative translation (m): X=+0.010000 Y=-0.020000 Z=+0.030000", before_loss)
            visual.update(None, Detector(), DetectionResult(), 3, now=1.7,
                          steering_wheel={"valid": False, "reason": "target_not_tracking"})
            visual.close()
        # Interpret the emitted cursor controls: updates must occupy the same
        # six rows, with no stale numeric pose after losing the target.
        screen, row = {}, 0
        for token in re.findall(r"\x1b\[\d+[FK]|[^\x1b]+", output.getvalue()):
            if token.endswith("F") and token.startswith("\x1b["):
                row -= int(token[2:-1])
            elif token == "\x1b[2K":
                screen[row] = ""
            else:
                for text in token.splitlines(keepends=True):
                    screen[row] = screen.get(row, "") + text.rstrip("\n")
                    row += int(text.endswith("\n"))
        self.assertEqual(row, 6)
        self.assertEqual(len(screen), 6)
        visible = "\n".join(screen.values())
        self.assertIn("frame=3", visible)
        self.assertIn("target_not_tracking", visible)
        self.assertNotIn("-0.250000", visible)
        self.assertNotIn("X=", visible)
        self.assertEqual(output.flushes, 3)

    def test_wheel_validity_change_prints_immediately_in_readable_redirected_logs(self):
        class Detector:
            kind = "apriltag"

        result = DetectionResult(status="TRACKING", pose=pose_from_pnp(
            np.zeros(3), np.array([.1, -.2, .3])), rvec=np.zeros(3))
        wheel = {"valid": True, "theta_rad": -.25, "translation_along_axis_m": .04,
                 "translation_vector_m": [.01, -.02, .03]}
        output = io.StringIO()
        visual = CameraVisualization(print_pose=True, print_interval_s=10)
        with redirect_stdout(output):
            visual.update(None, Detector(), result, 1, now=1., steering_wheel=wheel)
            visual.update(None, Detector(), result, 2, now=1.1,
                          steering_wheel={"valid": False, "reason": "calibration_unavailable"})
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertNotIn("\x1b", output.getvalue())
        self.assertIn("calibration_unavailable", lines[-1])
        self.assertNotIn("Wheel rotation (rad): -0.250000", lines[-1])

    def test_record_depth_choice_comes_from_logger_config(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "recording.yaml"
            for enabled in (False, True):
                path.write_text(yaml.safe_dump({
                    "config_version": 1,
                    "camera": {"mode": "raw", "sources": [{
                        "camera_id": "cockpit", "record_depth": enabled}]}}),
                    encoding="utf-8")
                client = RecordingClient(str(path), "cockpit")
                self.assertTrue(client.enabled)
                self.assertIs(client.record_depth, enabled)
                client.close()

    def test_quaternion_at_half_turn(self):
        pose = pose_from_pnp(np.array([np.pi, 0., 0.]),
                             np.array([[0.], [0.], [0.2]]))
        orientation = pose["orientation"]
        self.assertAlmostEqual(abs(orientation["qx"]), 1.0)
        self.assertAlmostEqual(orientation["qw"], 0.0)

    def test_yaml_selects_one_detector(self):
        # Use an explicit fixture; the deployment detector/tag size may be calibrated by the user.
        config = yaml.safe_load(Path(DEFAULT_CONFIG).read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "camera.yaml"
            for selected in ("apriltag", "charuco"):
                config["detector"]["type"] = selected
                path.write_text(yaml.safe_dump(config), encoding="utf-8")
                _, kind, settings, _ = load_config(path)
                self.assertEqual(kind, selected)
                self.assertEqual(settings, config[selected])

    def test_wire_pose_metres_and_xyzw_quaternion(self):
        pose = pose_from_pnp(np.array([0., 0., np.pi / 2]), np.array([[.1], [-.2], [.3]]))
        message = make_message("cockpit", 1, 1, 123456,
                               "11111111-1111-4111-8111-111111111111", "host-boot",
                               640, 480, "TRACKING", 1.0, pose)
        received = json.loads(json.dumps(message))["pose"]
        self.assertEqual(received["position"], {"x": .1, "y": -.2, "z": .3})
        q = received["orientation"]
        self.assertEqual(list(q), ["qx", "qy", "qz", "qw"])
        np.testing.assert_allclose([q[k] for k in ("qx", "qy", "qz", "qw")],
                                   [0, 0, np.sqrt(.5), np.sqrt(.5)], atol=1e-12)
        text = fmt_pose(received)
        self.assertIn("0.1000", text)
        self.assertIn(") m quat(qx,qy,qz,qw)=", text)

    def test_apriltag_and_charuco_on_synthetic_images(self):
        matrix = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
        distortion = np.zeros(5)
        tag = create_detector("apriltag", {
            "family": "tag36h11", "tag_id": 0, "tag_size_m": 0.05,
            "min_decision_margin": 0.0}, matrix, distortion)
        marker = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 220)
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        image[130:350, 210:430] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        result = tag.detect(image)
        self.assertEqual((result.status, result.tag_id), ("TRACKING", 0))
        self.assertAlmostEqual(result.pose["position"]["z"], 0.13636, delta=0.002)

        other_tag = create_detector("apriltag", {
            "family": "tag36h11", "tag_id": 1, "tag_size_m": 0.05}, matrix, distortion)
        self.assertEqual(other_tag.detect(image).status, "SEARCHING")

        board = create_detector("charuco", {
            "board_size": [5, 5], "square_length": 0.016,
            "marker_length": 0.015, "dictionary": "DICT_4X4_50",
            "min_corners": 6}, matrix, distortion)
        generated = board.board.generateImage((600, 600))
        board_image = cv2.cvtColor(generated, cv2.COLOR_GRAY2BGR)
        board_result = board.detect(board_image)
        self.assertEqual(board_result.status, "TRACKING")
        np.testing.assert_allclose(list(board_result.pose["position"].values()), board_result.tvec.flatten())
        self.assertEqual(board.total_corners, 16)
        self.assertEqual(board_result.confidence, 1.0)
        for source, detector, detection in ((image, tag, result), (board_image, board, board_result)):
            original = source.copy()
            preview = render_preview(source, detector, detection, 42, 30., False)
            np.testing.assert_array_equal(source, original, err_msg="preview modified recording image")
            self.assertFalse(np.array_equal(preview, source))

    def test_rgb_depth_and_detection_share_frame_identity(self):
        sensor = SensorFrame()
        session = "11111111-1111-4111-8111-111111111111"
        rgb = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        depth = np.array([[0x1234, 0x5678]], dtype=np.uint16)
        common = ("cockpit", session, "host-boot", "config1", 7, 123456,
                  1700000000000000, 30)
        rgb_meta, rgb_bytes = make_record_frame(
            *common, "rgb", sensor, rgb, {"fx": 1})
        depth_meta, depth_bytes = make_record_frame(
            *common, "depth", sensor, depth, {"fx": 1}, 0.001)
        detection = make_message("cockpit", 7, 1, 123456, session,
                                 "host-boot", 2, 1, "SEARCHING", 0.0, None)
        for metadata in (rgb_meta, depth_meta):
            self.assertEqual(
                (metadata["publisher_id"], metadata["session_id"],
                 metadata["camera_id"], metadata["frame_id"]),
                (detection["publisher_id"], detection["session_id"],
                 detection["camera_id"], detection["frame_id"]))
            self.assertEqual(metadata["sequence"], 42)
            self.assertEqual(metadata["encoding"], "raw")
        self.assertEqual(rgb_bytes, bytes([3, 2, 1, 6, 5, 4]))
        self.assertEqual(depth_bytes, bytes([0x34, 0x12, 0x78, 0x56]))
        self.assertEqual(rgb_meta["stride_bytes"], 6)
        self.assertEqual(depth_meta["stride_bytes"], 4)
        self.assertEqual(depth_meta["depth_scale"], 0.001)

    def test_camera_packet_wire(self):
        metadata, data = make_record_frame(
            "cockpit", "11111111-1111-4111-8111-111111111111",
            "host-boot", "config1", 7, 123456, 1700000000000000,
            30, "depth", SensorFrame(),
            np.array([[0x1234, 0x5678]], dtype=np.uint16),
            {"fx": 1}, 0.001)
        received_metadata, received_data = decode_packet(camera_packet(metadata, data))
        self.assertEqual(received_metadata, metadata)
        self.assertEqual(received_data, data)


if __name__ == "__main__":
    unittest.main()
