"""Actual EGL RGB -> detection/JPEG/Logger, private endpoints and no hardware.

Requires pyzmq, protobuf, NumPy and OpenCV aruco. --gui also exercises GLFW
context switching and closes only the floating camera window using X11.
"""
import argparse
import ctypes
import json
import math
from pathlib import Path
import socket
import subprocess
import tempfile
import time

import cv2
import numpy as np
import yaml
import zmq
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory


def endpoint(sock):
    port = sock.bind_to_random_port("tcp://127.0.0.1")
    return f"tcp://127.0.0.1:{port}"


def free_endpoint():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return f"tcp://127.0.0.1:{s.getsockname()[1]}"


def packet_type():
    file = descriptor_pb2.FileDescriptorProto(name="camera_packet.proto", package="aviator.record.v1", syntax="proto3")
    message = file.message_type.add(name="CameraPacket")
    message.field.add(name="metadata_json", number=1, type=9, label=1)
    message.field.add(name="data", number=2, type=12, label=1)
    pool = descriptor_pool.DescriptorPool()
    pool.Add(file)
    return message_factory.GetMessageClass(pool.FindMessageTypeByName("aviator.record.v1.CameraPacket"))


def close_preview():
    # Close only the test window (WM_DELETE_WINDOW), never another app/window.
    windows = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    matching = [line for line in windows.splitlines() if '"D436 RGB - Simulation"' in line]
    assert len(matching) == 1, matching
    window = int(matching[0].split()[0], 16)
    x = ctypes.CDLL("libX11.so.6")
    x.XOpenDisplay.restype = ctypes.c_void_p
    x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x.XInternAtom.restype = ctypes.c_ulong
    x.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.c_void_p]
    x.XFlush.argtypes = [ctypes.c_void_p]
    x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    class Data(ctypes.Union):
        _fields_ = [("b", ctypes.c_char*20), ("s", ctypes.c_short*10), ("l", ctypes.c_long*5)]
    class Client(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                    ("display", ctypes.c_void_p), ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                    ("format", ctypes.c_int), ("data", Data)]
    class Event(ctypes.Union):
        _fields_ = [("client", Client), ("padding", ctypes.c_long*24)]
    display = x.XOpenDisplay(None)
    assert display
    event = Event()
    event.client.type, event.client.display, event.client.window = 33, display, window
    event.client.message_type = x.XInternAtom(display,b"WM_PROTOCOLS",0)
    event.client.format = 32
    event.client.data.l[0] = x.XInternAtom(display,b"WM_DELETE_WINDOW",0)
    assert x.XSendEvent(display,window,False,0,ctypes.byref(event))
    x.XFlush(display)
    x.XCloseDisplay(display)
    return window


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulation", type=Path)
    parser.add_argument("root", type=Path)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    root, executable = args.root.resolve(), args.simulation.resolve()
    packet_class = packet_type()
    context = zmq.Context()
    business = context.socket(zmq.SUB); business.setsockopt(zmq.SUBSCRIBE,b"camera.detection")
    commands = context.socket(zmq.PUB)
    recording = context.socket(zmq.PULL)
    preview = context.socket(zmq.SUB); preview.setsockopt(zmq.SUBSCRIBE,b"camera.rgb.cockpit")
    sockets = [business,commands,recording,preview]
    for s in sockets: s.setsockopt(zmq.LINGER,0)
    business_endpoint, command_endpoint, record_endpoint = map(endpoint,sockets[:3])
    preview_endpoint = free_endpoint(); preview.connect(preview_endpoint)
    poll = zmq.Poller()
    for s in (business,recording,preview): poll.register(s,zmq.POLLIN)
    try:
        with tempfile.TemporaryDirectory(prefix="aviator-camera-stream-") as directory:
            directory = Path(directory)
            system = yaml.safe_load((root/"config/system.yaml").read_text())
            system.update(robot=str(root/"config/robot.yaml"), manipulator_service=free_endpoint())
            system["bus"] = dict(publish=business_endpoint, subscribe=command_endpoint)
            (directory/"system.yaml").write_text(yaml.safe_dump(system))
            record = yaml.safe_load((root/"config/recording.yaml").read_text())
            record["camera"].update(mode="raw", record_endpoint=record_endpoint,
                                    receive_hwm=2, queue_bytes=16777216)
            (directory/"recording.yaml").write_text(yaml.safe_dump(record))
            cmd = [str(executable), "--config",str(directory/"system.yaml"),
                   "--camera-config",str(root/"config/camera.yaml"),
                   "--recording-config",str(directory/"recording.yaml"),
                   "--preview-endpoint",preview_endpoint,"--duration","25"]
            if not args.gui: cmd += ["--headless"]
            with (directory/"process.log").open("w+") as log:
                process = subprocess.Popen(cmd,cwd=directory,stdout=log,stderr=subprocess.STDOUT)
                try:
                    detections, images, previews = {}, {}, {}
                    deadline = time.monotonic()+20
                    matched = None
                    while time.monotonic() < deadline:
                        assert process.poll() is None, "simulation exited before streaming"
                        for s in dict(poll.poll(100)):
                            if s is business:
                                topic, data = s.recv_multipart(); assert topic == b"camera.detection"
                                detection = json.loads(data)
                                if detection["frame_id"] is not None:
                                    detections[detection["frame_id"]] = detection
                            elif s is recording:
                                packet = packet_class.FromString(s.recv())
                                meta = json.loads(packet.metadata_json)
                                images[meta["frame_id"]] = (meta,packet.data)
                            else:
                                parts = s.recv_multipart(); assert len(parts) == 3
                                assert parts[0] == b"camera.rgb.cockpit"
                                meta = json.loads(parts[1]); previews[meta["frame_id"]] = (meta,parts[2])
                        common = detections.keys() & images.keys() & previews.keys()
                        if common:
                            matched = max(common); break
                    assert matched is not None, (list(detections),list(images),list(previews))
                    detection, (raw_meta, raw), (jpeg_meta, jpeg) = detections[matched], images[matched], previews[matched]
                    for key in ("publisher_id","session_id","clock_id","camera_id","frame_id","sample_mono_us"):
                        assert detection[key] == raw_meta[key] == jpeg_meta[key], key
                    assert detection["msg_type"] == "CameraDetection" and detection["version"] == "1.0"
                    assert detection["publisher_id"] == "simulation" and detection["camera_id"] == "cockpit"
                    assert detection["valid"] and detection["status"] == "TRACKING" and detection["command_ref"] is None
                    assert detection["detector"] == "mujoco_ground_truth"
                    wheel = detection["steering_wheel"]
                    assert wheel["valid"] and wheel["calibration_id"] == "mujoco_ground_truth"
                    assert math.isfinite(wheel["theta_rad"]) and math.isfinite(wheel["translation_along_axis_m"])
                    assert detection["yoke"]["detected"] and all(-1 <= detection["yoke"][k] <= 1 for k in ("roll","pitch"))
                    assert raw_meta["encoding"] == "raw" and raw_meta["pixel_format"] == "RGB8"
                    assert raw_meta["width"] == detection["image_width"] == 1280
                    assert raw_meta["height"] == detection["image_height"] == 800
                    assert raw_meta["stride_bytes"] == 1280*3 and len(raw) == 1280*800*3
                    rgb = np.frombuffer(raw,np.uint8).reshape(800,1280,3)
                    decoded = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                    assert decoded.shape == (360,576,3), decoded.shape
                    assert (jpeg_meta["width"],jpeg_meta["height"]) == (576,360)
                    assert (jpeg_meta["original_width"],jpeg_meta["original_height"]) == (1280,800)
                    expected = cv2.cvtColor(cv2.resize(rgb,(576,360),interpolation=cv2.INTER_AREA),cv2.COLOR_RGB2BGR)
                    assert np.abs(decoded.astype(float)-expected).mean() < 8, "JPEG color/orientation differs from raw"
                    params = cv2.aruco.DetectorParameters(); params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
                    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params)
                    corners, ids, _ = detector.detectMarkers(rgb)
                    assert ids is not None and 0 in ids, "raw camera frame must contain upright decodable tag"
                    center = corners[list(ids.ravel()).index(0)].reshape(4,2).mean(0)
                    p = detection["pose"]["position"]; k = raw_meta["calibration"]["rgb"]
                    projected = [k["fx"]*p["x"]/p["z"]+k["ppx"], k["fy"]*p["y"]/p["z"]+k["ppy"]]
                    assert np.linalg.norm(center-projected) < 3, (center,projected)
                    q = detection["pose"]["orientation"]
                    assert abs(sum(q[k]**2 for k in ("qx","qy","qz","qw"))-1) < 1e-6
                    if args.output_dir:
                        args.output_dir.mkdir(parents=True,exist_ok=True)
                        cv2.imwrite(str(args.output_dir/"rgb.png"),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
                        (args.output_dir/"preview.jpg").write_bytes(jpeg)
                        (args.output_dir/"detection.json").write_text(json.dumps(detection,indent=2))
                        if args.gui:
                            windows = subprocess.check_output(["xwininfo","-root","-tree"],text=True)
                            camera_window = next(line.split()[0] for line in windows.splitlines() if '"D436 RGB - Simulation"' in line)
                            subprocess.run(["import","-window",camera_window,str(args.output_dir/"floating.png")],check=True,timeout=5)
                    if args.gui:
                        closed = close_preview()
                    # Disconnect Logger. Acquisition/business/JPEG must continue without backpressure.
                    poll.unregister(recording); recording.close()
                    later = matched
                    until = time.monotonic()+3
                    while time.monotonic()<until and later<matched+5:
                        if business.poll(100):
                            d = json.loads(business.recv_multipart()[1])
                            later = d["frame_id"] or later
                    assert later >= matched+5 and process.poll() is None, "image consumer/preview close stopped simulation"
                    if args.gui:
                        state = subprocess.check_output(["xwininfo","-id",hex(closed)],text=True)
                        assert "IsUnMapped" in state, state
                    print("PASS detection/JPEG/raw identity, pixels, tag pose, absent Logger" +
                          (", GLFW preview and independent close" if args.gui else ""))
                except BaseException:
                    log.flush(); log.seek(0); print(log.read()); raise
                finally:
                    process.terminate()
                    process.wait(timeout=10)
                    assert process.returncode == 0, process.returncode
    finally:
        for s in sockets: s.close()
        context.term()


if __name__ == "__main__":
    main()
