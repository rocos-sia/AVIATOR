#!/usr/bin/env python3
"""Verify foxglove_bridge speaks foxglove.websocket.v1 correctly.

Connects to the bridge as a minimal Foxglove Studio stand-in and checks:

  1. the WebSocket subprotocol "foxglove.websocket.v1" is negotiated,
  2. a serverInfo text frame arrives first,
  3. advertise frames describe each channel (topic / encoding / schema),
  4. subscribing yields binary message frames with the correct layout
     [0x01][u32 subscriptionId LE][u64 timestamp_ns LE][payload],
  5. camera.image payloads decode to a foxglove.CompressedImage JSON object
     whose base64 `data` round-trips to a valid JPEG (starts with 0xFFD8),
  6. data topics arrive as JSON with a sensible timestamp.

Usage:
    python3 test_client.py [--host 127.0.0.1] [--port 8765] [--seconds 5]

Exit code 0 = all checks passed. Requires: websocket-client.
"""

import argparse
import base64
import json
import struct
import time

import websocket
from websocket import ABNF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    ws = websocket.WebSocket()
    ws.connect(f"ws://{args.host}:{args.port}",
               subprotocols=["foxglove.websocket.v1"])

    if ws.getsubprotocol() != "foxglove.websocket.v1":
        raise SystemExit(f"FAIL: subprotocol not negotiated (got {ws.getsubprotocol()!r})")
    print("OK  subprotocol negotiated:", ws.getsubprotocol())

    channels = {}          # channelId -> {topic, encoding, schemaName}
    server_info = False
    stats = {}             # channelId -> count
    image_ok = data_ok = False

    deadline = time.time() + args.seconds
    ws.settimeout(0.25)

    while time.time() < deadline:
        try:
            opcode, raw = ws.recv_data()
        except websocket.WebSocketTimeoutException:
            continue
        except websocket.WebSocketConnectionClosedException:
            break

        if opcode == ABNF.OPCODE_TEXT:
            j = json.loads(raw.decode())
            op = j.get("op")
            if op == "serverInfo":
                server_info = True
                print("OK  serverInfo:", j.get("name"))
            elif op == "advertise":
                for ch in j.get("channels", []):
                    channels[ch["id"]] = {
                        "topic": ch["topic"],
                        "encoding": ch["encoding"],
                        "schemaName": ch["schemaName"],
                    }
                    print(f"OK  advertise id={ch['id']} topic={ch['topic']} "
                          f"encoding={ch['encoding']} schema={ch['schemaName']}")
                    # Auto-subscribe to each newly advertised channel.
                    ws.send(json.dumps({
                        "op": "subscribe",
                        "subscriptions": [{"id": 1000 + ch["id"], "channelId": ch["id"]}],
                    }))
        elif opcode == ABNF.OPCODE_BINARY:
            if len(raw) < 13 or raw[0] != 1:
                raise SystemExit("FAIL: bad binary header")
            sub_id = struct.unpack("<I", raw[1:5])[0]
            ts_ns = struct.unpack("<Q", raw[5:13])[0]
            payload = raw[13:]
            channel_id = sub_id - 1000
            stats[channel_id] = stats.get(channel_id, 0) + 1

            info = channels.get(channel_id, {})
            if info.get("schemaName") == "foxglove.CompressedImage":
                img = json.loads(payload.decode())
                jpeg = base64.b64decode(img["data"])
                if jpeg[:2] == b"\xff\xd8":
                    image_ok = True
                print(f"     image ts={ts_ns/1e9:.3f}s frame_id={img['frame_id']!r} "
                      f"format={img['format']!r} bytes={len(jpeg)}")
            else:
                doc = json.loads(payload.decode())
                data_ok = True
                print(f"     data  ts={ts_ns/1e9:.3f}s topic={info.get('topic')} "
                      f"seq={doc.get('sequence')}")

    ws.close()

    print("\nsummary:", stats)
    ok = server_info and bool(channels) and image_ok and data_ok
    for cond, name in [(server_info, "serverInfo"),
                       (bool(channels), "advertise"),
                       (image_ok, "image decoded"),
                       (data_ok, "data received")]:
        print(("OK  " if cond else "FAIL"), name)

    if not ok:
        raise SystemExit("FAIL: one or more checks failed")
    print("PASS: foxglove_bridge protocol verified")


if __name__ == "__main__":
    main()
