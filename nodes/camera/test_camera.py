"""Hardware-free checks for the camera node's Logger wire contract."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import cv2
import yaml

from detectors import create_detector, pose_from_pnp
from main import DEFAULT_CONFIG, load_config, make_message, make_record_frame
from recording_client import RecordingClient, camera_packet


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
        _, kind, settings = load_config(DEFAULT_CONFIG)
        self.assertEqual(kind, "apriltag")
        self.assertEqual((settings["tag_id"], settings["tag_size_m"]), (0, 0.05))
        config = yaml.safe_load(Path(DEFAULT_CONFIG).read_text(encoding="utf-8"))
        config["detector"]["type"] = "charuco"
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "camera.yaml"
            path.write_text(yaml.safe_dump(config), encoding="utf-8")
            _, kind, settings = load_config(path)
        self.assertEqual(kind, "charuco")
        self.assertEqual(settings["dictionary"], "DICT_4X4_50")

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
        self.assertAlmostEqual(result.pose["position"]["z"], 136.36, delta=2)

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
        self.assertEqual(board.total_corners, 16)
        self.assertEqual(board_result.confidence, 1.0)

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
