# Foxglove 实时可视化桥 (ZMQ → WebSocket)

一个自包含示例，把 AVIATOR 的消息总线（数据 + 相机图像）实时转发到 **Foxglove Studio**，在浏览器里同时看图像和曲线。供你调整、测试，之后可升级为正式的 `nodes/aviator_telemetry_bridge`。

## 为什么图像走独立通道

控制总线把所有消息限制在 **64 KiB**（`common/protocol.hpp` 的 `max_payload_bytes`，经 ZMQ `maxmsgsize` 生效），而一帧彩色 JPEG 通常是 30–150 KiB，会被总线静默丢弃。所以按 SAD §03 的规则：**相机帧不上控制总线**，走自己的 PUB 通道 `tcp://127.0.0.1:5558`（大 `maxmsgsize`）。

## 组件

| 进程 | 作用 |
|------|------|
| `camera_pub` | OpenCV 抓一帧相机 → JPEG 编码 → PUB 到独立图像通道（多帧消息） |
| `foxglove_bridge` | 纯转发：SUB 控制总线 5556 + 图像通道 5558 → Boost.Beast WebSocket 说 `foxglove.websocket.v1` |
| `test_publisher.py` | 造数（无相机/无总线也可测）：flight.command / arm.state / camera.image，`--data-only` 只发数据 |
| `test_client.py` | 以最小 Foxglove 客户端身份验证协议（serverInfo / advertise / 二进制帧 / protobuf 还原 JPEG） |

## 消息格式定义

### 数据 topic（控制总线 `tcp://127.0.0.1:5556`，ZMQ 两帧 `topic + JSON`）

JSON 里带 `timestamp`（**µs** 纪元时间），桥转发时把 µs 换成 ns 写进 foxglove 二进制帧头。

**`/flight.command` @ 50 Hz**

```json
{
  "msg_type": "FlightCommand",
  "version": "1.0",
  "sequence": 42,
  "timestamp": 1727452800123456,
  "valid": true,
  "source": "JOYSTICK",
  "control": { "roll": 0.382, "pitch": 0.291, "yaw": 0.118 }
}
```

| 字段 | 类型 | 含义 |
|------|------|------|
| `control.roll` / `control.pitch` / `control.yaw` | float | 姿态指令（弧度） |
| `source` | string | 指令来源（JOYSTICK / …） |
| `timestamp` | int | µs 纪元时间 |

**`/arm.state` @ 100 Hz**

```json
{
  "msg_type": "ArmState",
  "version": "1.0",
  "sequence": 43,
  "timestamp": 1727452800123466,
  "valid": true,
  "joint_position": [0.0, 0.322, 0.5, 0.322, 0.0, -0.322, -0.5]
}
```

| 字段 | 类型 | 含义 |
|------|------|------|
| `joint_position` | float[7] | 7 个关节位置（弧度） |

### `camera.image`（图像通道 `tcp://127.0.0.1:5558`，ZMQ 三帧 multipart）

ZMQ 消息本身无类型，用 multipart 让消费端还原帧号与采集时间：

- **Frame0** = topic（`camera.image`）
- **Frame1** = JPEG 字节（二进制）
- **Frame2** = JSON 元数据：

```json
{ "frame": 100, "timestamp": 1727452800123456, "width": 1280, "height": 720, "format": "jpeg" }
```

### `foxglove.CompressedImage`（桥对外的 protobuf 编码）

桥把 `camera.image` 转成 protobuf 的 `foxglove.CompressedImage`，`data` 字段直接放原始 JPEG 字节（**无 base64、无文本序列化**），比 JSON+base64 省掉 33% 体积膨胀和 base64 编解码的 CPU/内存拷贝。

```proto
syntax = "proto3";
package foxglove;

message Timestamp { int64 sec = 1; uint32 nsec = 2; }

message CompressedImage {
  Timestamp timestamp = 1;  // 采集时间（秒 + 纳秒）
  bytes    data       = 2;  // 原始 JPEG 字节
  string   format     = 3;  // "jpeg"
  string   frame_id   = 4;  // 相机标识
}
```

## 构建

```bash
cd examples/foxglove_bridge
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

依赖：OpenCV（仅 `camera_pub`）、Boost 1.74（Beast/Asio，仅 `foxglove_bridge`）、libzmq + 仓库内 cppzmq、nlohmann/json（头文件，CMake 配置或裸 include 均可）、protobuf 3.12（运行时，仅 `foxglove_bridge`）。

protobuf 的 `.proto` 源在 `proto/foxglove/CompressedImage.proto`；生成的 `foxglove/CompressedImage.pb.{h,cc}` 与 `foxglove_schema.hpp`（FileDescriptorSet 的 base64）已 vendor 进 `src/generated/`，构建时只需 protobuf 运行时、无需 protoc。改 `.proto` 后跑 `scripts/regenerate_proto.sh` 重新生成。

## 运行

> **环境注意**：`test_publisher.py` 和下面的 ZMQ 订阅脚本需要 **pyzmq**，系统 `python3` 没有，必须用
> `/home/rocos/miniconda3/envs/serl_clean/bin/python`。
> `test_client.py` 只用 `websocket-client`（不 import zmq），系统 `python3` 也能跑。
> 起新进程前先清残留，避免端口占用：

```bash
pkill -9 -f foxglove_bridge; pkill -9 -f camera_pub; pkill -9 -f test_publisher.py; sleep 1
```

### 方式 1：无相机快速体验（推荐先跑）

```bash
./build/foxglove_bridge &                          # 起桥，监听 ws://localhost:8765
/home/rocos/miniconda3/envs/serl_clean/bin/python test_publisher.py &   # 假数据 + 假图像
/home/rocos/miniconda3/envs/serl_clean/bin/python test_client.py --seconds 5   # 验证，期望 PASS
```

### 方式 2：真实相机 + 假总线数据（常用调试组合）

真实相机供给 `/camera.image`，sin 造数供给 `/flight.command` + `/arm.state`。`--data-only` 让 `test_publisher.py` **不绑 5558**，从而与 `camera_pub` 并行无冲突：

```bash
./build/foxglove_bridge &
./build/camera_pub --camera 4 --fps 30 &           # 真实相机 → :5558
/home/rocos/miniconda3/envs/serl_clean/bin/python test_publisher.py --data-only &   # 假数据 → :5556
```

相机节点：本机三个 RealSense 彩色节点 `/dev/video4`（1280×720 YUYV）、`/dev/video10`、`/dev/video16`，对应 `--camera 4/10/16`。`/dev/video4` 打不开就换 `--camera 10`。

### 方式 3：真实相机 + 真实总线

```bash
./build/foxglove_bridge &
./build/camera_pub --camera 4 --fps 30 &
# 数据侧跑真实的 AVIATOR 节点（绑定 tcp://127.0.0.1:5556 发布 flight.command / arm.state）
```

### 方式 4：接 Foxglove Studio（见下）

## 命令行查看数据（不开 Studio）

### 看数据曲线（直接订阅 5556）

```bash
/home/rocos/miniconda3/envs/serl_clean/bin/python - <<'EOF'
import zmq, json
s = zmq.Context().socket(zmq.SUB)
s.connect("tcp://127.0.0.1:5556")
s.setsockopt(zmq.SUBSCRIBE, b"")            # 订阅所有 topic
print("listening 5556 ... Ctrl-C 停止")
while True:
    topic, payload = s.recv_multipart()
    d = json.loads(payload)
    if topic == b"flight.command":
        c = d["control"]
        print(f"CMD  roll={c['roll']:+.3f}  pitch={c['pitch']:+.3f}  yaw={c['yaw']:+.3f}")
    elif topic == b"arm.state":
        print("ARM " + " ".join(f"{x:+.3f}" for x in d["joint_position"]))
EOF
```

### 看相机图像（订阅 5558 存一帧 JPEG）

```bash
/home/rocos/miniconda3/envs/serl_clean/bin/python - <<'EOF'
import zmq, json
s = zmq.Context().socket(zmq.SUB)
s.connect("tcp://127.0.0.1:5558")
s.setsockopt(zmq.SUBSCRIBE, b"camera.image")
topic, jpeg, meta = s.recv_multipart()
m = json.loads(meta)
out = f"/tmp/frame_{m['frame']}.jpg"
open(out, "wb").write(jpeg)
print(f"saved {out}: {m['width']}x{m['height']} {len(jpeg)} bytes")
EOF
```

### 走整条链路（WS 桥）

```bash
/home/rocos/miniconda3/envs/serl_clean/bin/python test_client.py --seconds 5
```

打印每个 channel 的 advertise、图像帧（ts / frame_id / bytes）和数据消息（topic / seq），最后 `PASS`。

## Foxglove Studio 使用

### 打开方式

- **桌面版（推荐）**：https://foxglove.dev/download 下载 Linux `.AppImage` / `.deb`。自带最新 Chromium 内核，不受系统浏览器版本影响。
  ```bash
  chmod +x FoxgloveStudio-*.AppImage && ./FoxgloveStudio-*.AppImage
  ```
- **网页版**：浏览器打开 https://studio.foxglove.dev/ 。需要较新的浏览器（Chrome/Edge ≥ 117、Firefox ≥ 128、Safari ≥ 17.2）。旧浏览器会报 `Intl.DurationFormat is not a constructor` —— 升级浏览器或改用桌面版。

### 连接

1. 先把桥（和发布端）跑起来。
2. Studio 里点 **Open connection** → **Foxglove WebSocket**（或 Custom → WebSocket）。
3. URL 填 `ws://localhost:8765`，点 **Open**。
4. 左侧 topic 列表出现 `/arm.state`、`/flight.command`、`/camera.image`。

### 看图像

点 `/camera.image` → **Add panel → Image**（或把 topic 拖进画布）。Studio 识别 `foxglove.CompressedImage`，自动解码 protobuf 里的 JPEG。

### 看曲线（关键：设置 message path）

Plot 面板**不会自动出曲线**，必须为每个 series 指定 y 值来自哪个字段。空面板会提示 `messages path containing y values for the series`。

两种设置方式：

1. **拖拽**：左侧展开 `/flight.command` → `control`，把 `roll` 拖进 Plot 面板；再拖 `pitch`、`yaw`。
2. **手动填**：选中 Plot 面板 → 左侧 settings 的 Series → `+` 加 series → 在 **Message path** 填：

```
/flight.command.control.roll
/flight.command.control.pitch
/flight.command.control.yaw
/arm.state.joint_position
```

- 一个 series = 一条曲线；`/arm.state.joint_position` 是数组，Foxglove 自动画出 7 条线。
- message path = `topic + 字段路径`，用 `.` 连接（注意是 `control.roll` 不是 `roll`）。

### 同屏显示图像 + 曲线

一个 **panel 只能做一种事**（Image 显示图像、Plot 画曲线），所以"同屏"= 一个 **Layout 里放两个 panel 并排**，共享时间轴、时间对齐：

1. New Layout。
2. 拖 `/camera.image` 进画布 → 自动生成 Image 面板。
3. 拖 `/flight.command` 进画布空白处 → 自动生成 Plot 面板，再按上面设置 message path。
4. 拖动两个面板调整大小：左图右曲线。

## 常见问题（FAQ）

| 现象 | 原因 / 解决 |
|------|-------------|
| 网页版报 `Intl.DurationFormat is not a constructor` | 浏览器太旧。升级浏览器或改用桌面版 Foxglove Studio |
| `ModuleNotFoundError: No module named 'zmq'` | 用了系统 `python3`。改用 `/home/rocos/miniconda3/envs/serl_clean/bin/python` |
| `bind: Address already in use` | 端口冲突（5556/5558/8765）。先 `pkill -9` + `sleep 1` 清残留；注意 `camera_pub` 和 `test_publisher.py` 都绑 5558，数据侧必须用 `--data-only` |
| Plot 空白 + `messages path … for the series` | 没设 message path。按上面"看曲线"设置 |
| 图像黑屏 | `camera_pub` 没跑 / 相机节点不对。看它的日志，换 `--camera 10`；确认 `/camera.image` 在 topic 列表里 |
| 曲线空白（已设 path） | 确认发布端在跑、topic 在列表里、字段名拼写与消息 JSON 一致 |

## 选项

```bash
camera_pub     [--camera N] [--endpoint URL] [--fps N] [--width N] [--height N] [--quality N]
foxglove_bridge [--data-endpoint URL] [--image-endpoint URL] [--listen ADDR] [--port N]
test_publisher.py [--data-only]          # --data-only 只发数据、不绑 5558，与 camera_pub 并行
```

## 协议要点（foxglove.websocket.v1）

- WebSocket 子协议协商 `foxglove.websocket.v1`。
- 服务端 → 客户端：
  - 文本 `{"op":"serverInfo",...}`
  - 文本 `{"op":"advertise","channels":[{id,topic,encoding,schemaName,schema}]}`
  - 二进制 `[0x01][u32 subscriptionId LE][u64 timestampNs LE][payload]`
- 客户端 → 服务端：文本 `{"op":"subscribe"|"unsubscribe",...}`（`clientPublish`/`parameters` 忽略）。
- 通道映射：总线 topic → JSON 通道（payload 原样透传）；`camera.image` → `foxglove.CompressedImage`（protobuf 编码，`data` 为原始 JPEG 字节，无 base64）。通道**惰性注册**：首个消息到达时才 `advertise` 给所有客户端。
- 通道的 `schema` 字段：JSON 通道是 JSON Schema 文本；protobuf 通道是 FileDescriptorSet 的 base64（vendor 在 `foxglove_schema.hpp`）。

## 后续升级路径

- 通道 `advertise` 时机改为启动即注册（用 `schemas/` 里的真实 JSON Schema / 官方 foxglove schemas）。
- 图像多路（2~3 路相机）时复用同一 protobuf 通道，`frame_id` 区分相机，或按相机拆多个 `camera.<id>.image` topic。
- 引入 `common/transport` 的端点常量，避免硬编码端口。
- 加多客户端订阅统计、丢帧计数、ZMQ HWM 调优。
