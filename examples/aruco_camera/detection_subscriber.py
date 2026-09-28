#!/usr/bin/env python3
"""订阅 camera.detection，解码并打印 ChArUco 板 6DOF 姿态（相机测试接收端）。

默认 SUB connect tcp://127.0.0.1:5556（AVIATOR subscribe_endpoint），过滤
camera.detection。也可用 --bind + --endpoint 直连发布端做独立测试。
"""

import argparse
import json
import sys

import zmq


def parse_args(argv):
    p = argparse.ArgumentParser(description="订阅 camera.detection 并打印姿态")
    p.add_argument("--endpoint", default="tcp://127.0.0.1:5556",
                   help="SUB 连接端点（AVIATOR subscribe_endpoint）")
    p.add_argument("--bind", action="store_true",
                   help="SUB 改为 bind 而非 connect（直连测试）")
    p.add_argument("--topic", default="camera.detection", help="订阅前缀")
    p.add_argument("--max-msgs", type=int, default=0,
                   help="收到 N 条后退出（0 表示无限）")
    p.add_argument("--timeout-ms", type=int, default=1000,
                   help="poll 超时（毫秒）")
    p.add_argument("--raw", action="store_true", help="同时打印原始 JSON")
    return p.parse_args(argv)


def fmt_pose(pose):
    if not pose:
        return "pose=null"
    pos = pose["position"]
    ori = pose["orientation"]
    return (f"pos=({pos['x']:7.2f},{pos['y']:7.2f},{pos['z']:7.2f}) mm "
            f"quat=({ori['qx']:.4f},{ori['qy']:.4f},{ori['qz']:.4f},{ori['qw']:.4f})")


def main(argv):
    args = parse_args(argv)
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 8)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.MAXMSGSIZE, 65536)
    sub.setsockopt(zmq.SUBSCRIBE, args.topic.encode("utf-8"))
    if args.bind:
        sub.bind(args.endpoint)
    else:
        sub.connect(args.endpoint)

    print(f"detection_subscriber: topic={args.topic} endpoint={args.endpoint} bind={args.bind}")

    count = 0
    try:
        while args.max_msgs <= 0 or count < args.max_msgs:
            if sub.poll(args.timeout_ms) == 0:
                continue
            topic, payload = sub.recv_multipart()
            data = json.loads(payload.decode("utf-8"))

            if data.get("msg_type") != "CameraDetection":
                print(f"[skip] unexpected msg_type={data.get('msg_type')!r}", file=sys.stderr)
                continue

            count += 1
            seq = data.get("sequence")
            fid = data.get("frame_id")
            status = data.get("status")
            conf = data.get("confidence")
            valid = data.get("valid")
            sample = data.get("sample_mono_us")
            stamp = data.get("timestamp")
            print(f"[{count}] seq={seq} frame={fid} status={status} conf={conf:.3f} "
                  f"valid={valid} sample_mono_us={sample} timestamp={stamp}")
            print(f"        {fmt_pose(data.get('pose'))}")
            if args.raw:
                print("        " + json.dumps(data, ensure_ascii=False))
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        context.term()
        print("detection_subscriber: stopped")


if __name__ == "__main__":
    main(sys.argv[1:])
