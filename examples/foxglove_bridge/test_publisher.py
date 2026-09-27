#!/usr/bin/env python3
"""Synthetic AVIATOR bus publisher — drive foxglove_bridge without hardware.

Publishes the same wire shapes the real nodes produce, so you can exercise the
bridge (and Foxglove Studio) with zero camera / no running AVIATOR stack:

  * tcp://127.0.0.1:5556  (control bus, JSON, `timestamp` in µs)
        flight.command @ 50 Hz, arm.state @ 100 Hz
  * tcp://127.0.0.1:5558  (dedicated image channel, multipart)
        Frame0 = b"camera.image"
        Frame1 = JPEG bytes
        Frame2 = JSON metadata {frame, timestamp µs, width, height, format}

Usage:
    python3 test_publisher.py                 # fake data + fake image (no hardware)
    python3 test_publisher.py --data-only     # fake data only (run next to camera_pub)

Run foxglove_bridge first, then this, then test_client.py (or Foxglove Studio).
`--data-only` skips the camera.image channel (does not bind :5558) so it can run
alongside `camera_pub`, which supplies the real image.
Requires: pyzmq
"""

import base64
import json
import math
import time

import zmq

# A tiny 8x8 red JPEG (633 bytes) so the image path is exercised with a real,
# decodable JPEG and no camera is needed.
_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgFBQQEBQoHBwYIDAoM"
    "DAsKCwsNDhIQDQ4RDgsLEBYQERMUFRUVDA8XGBYUGBIUFRT/2wBDAQMEBAUEBQkFBQkUDQsN"
    "FBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBT/wAAR"
    "CAAIAAgDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRA"
    "AAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2Jyggk"
    "KFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIW"
    "Gh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+T"
    "l5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtRE"
    "AAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkN"
    "OEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhY"
    "aHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5eb"
    "n6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD50ooor8MP9Uz/2Q=="
)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-only", action="store_true",
                    help="publish only the data topics (no camera.image), "
                         "so this can run alongside camera_pub")
    args = ap.parse_args()

    ctx = zmq.Context()
    data = ctx.socket(zmq.PUB)
    data.bind("tcp://127.0.0.1:5556")

    image = None
    if not args.data_only:
        image = ctx.socket(zmq.PUB)
        image.bind("tcp://127.0.0.1:5558")

    # Give the bridge's SUB sockets a moment to connect (PUB/SUB drops frames
    # until the subscription has propagated).
    time.sleep(0.5)

    seq = 0
    frame = 0
    t0 = time.time()
    mode = "data-only" if args.data_only else "data + image"
    print(f"publishing {mode}: flight.command@50Hz, arm.state@100Hz"
          + ("" if args.data_only else ", camera.image@33Hz") + "; Ctrl-C to stop")
    while True:
        now_us = int(time.time() * 1_000_000)
        t = time.time() - t0  # seconds since start (drives the sin curves)

        # arm.state @ 100 Hz (every tick) — 7 joints as phase-shifted sin waves
        seq += 1
        joints = [0.5 * math.sin(2 * math.pi * 0.10 * t + 0.7 * i)
                  for i in range(7)]
        arm = json.dumps({
            "msg_type": "ArmState", "version": "1.0", "sequence": seq,
            "timestamp": now_us, "valid": True,
            "joint_position": joints,
        })
        data.send_multipart([b"arm.state", arm.encode()])

        # flight.command @ 50 Hz (every other tick) — sin roll/pitch/yaw
        if seq % 2 == 0:
            cmd = json.dumps({
                "msg_type": "FlightCommand", "version": "1.0", "sequence": seq,
                "timestamp": now_us, "valid": True, "source": "JOYSTICK",
                "control": {
                    "roll": 0.4 * math.sin(2 * math.pi * 0.20 * t),
                    "pitch": 0.3 * math.sin(2 * math.pi * 0.13 * t + 1.0),
                    "yaw": 0.2 * math.sin(2 * math.pi * 0.07 * t),
                },
            })
            data.send_multipart([b"flight.command", cmd.encode()])

        # camera.image @ ~33 Hz (every third tick), unless --data-only
        if image is not None and seq % 3 == 0:
            frame += 1
            meta = json.dumps({
                "frame": frame, "timestamp": now_us,
                "width": 8, "height": 8, "format": "jpeg",
            })
            image.send_multipart([b"camera.image", _JPEG, meta.encode()])

        time.sleep(0.01)  # 100 Hz base loop


if __name__ == "__main__":
    main()
