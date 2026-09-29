"""Synthetic direct-to-MCAP image test; run in the camera Python environment."""

import os
import tempfile

import cv2
import numpy as np
from mcap.reader import make_reader

from camera_publisher import ImageRecorder, make_message
from record_image_pb2 import ImageFrame


def main():
    with tempfile.TemporaryDirectory(prefix="aviator-camera-record-") as root:
        path = os.path.join(root, "camera.mcap")
        source = np.arange(60 * 80 * 3, dtype=np.uint8).reshape(60, 80, 3)
        recorder = ImageRecorder(path, "shared-session", "cockpit", 6)
        recorder.submit(source, 42, 123456, "boot")
        recorder.close()
        assert recorder.messages == 1 and recorder.dropped == 0
        assert os.path.exists(path) and not os.path.exists(path + ".partial")

        with open(path, "rb") as stream:
            reader = make_reader(stream)
            records = list(reader.iter_messages())
            assert len(records) == 1
            schema, channel, message = records[0]
            assert schema.name == "aviator.record.ImageFrame"
            assert schema.encoding == channel.message_encoding == "protobuf"
            assert channel.topic == "record.camera.cockpit.image"
            assert channel.metadata["session_id"] == "shared-session"
            decoded = ImageFrame.FromString(message.data)
            assert (decoded.session_id, decoded.camera_id, decoded.frame_id) == \
                   ("shared-session", "cockpit", 42)
            assert decoded.sample_mono_us == 123456
            pixels = cv2.imdecode(np.frombuffer(decoded.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            assert np.array_equal(pixels, source)

        detection = make_message("cockpit", 42, 1, 123456, "shared-session", "boot",
                                 80, 60, "SEARCHING", 0.0, None)
        assert (detection["session_id"], detection["camera_id"], detection["frame_id"]) == \
               (decoded.session_id, decoded.camera_id, decoded.frame_id)

        try:
            ImageRecorder(path, "shared-session", "cockpit", 6)
        except FileExistsError:
            pass
        else:
            raise AssertionError("existing MCAP was overwritten")

        print("direct image MCAP roundtrip passed")


if __name__ == "__main__":
    main()
