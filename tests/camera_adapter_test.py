"""Python producer wire format -> C++ Protobuf parser, without a camera."""
import importlib.util
import pathlib
import subprocess
import sys
import tempfile

spec = importlib.util.spec_from_file_location("recording_client", sys.argv[2])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
metadata = {
    "version": 1, "camera_id": "cockpit", "stream": "depth", "publisher_id": "python-camera",
    "session_id": "11111111-1111-4111-8111-111111111111", "clock_id": "python-boot",
    "config_id": "calibration-v1", "pixel_format": "Z16", "byte_order": "little", "encoding": "raw",
    "width": 2, "height": 2, "stride_bytes": 4, "fps": 30, "sequence": 4294967297,
    "timestamp_us": 1700000000000000, "sample_mono_us": 12345,
    "depth_scale": 0.001, "calibration": {"test": True},
}
with tempfile.TemporaryDirectory(prefix="aviator-camera-adapter-") as directory:
    file = pathlib.Path(directory) / "packet.bin"
    file.write_bytes(module.camera_packet(metadata, b"\x00\xff\x01\x80\x02\x00\xff\x7f"))
    subprocess.run([sys.argv[1], "--python-packet", str(file)], check=True, timeout=5)
print("Python CameraPacket interoperability passed")
