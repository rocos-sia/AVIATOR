# Foxglove 实时可视化桥 (ZMQ → WebSocket)

一个自包含示例，把 AVIATOR 的消息总线（数据 + 相机图像）实时转发到 **Foxglove Studio**，在浏览器里同时看图像和曲线。供你调整、测试，之后可升级为正式的 `nodes/aviator_telemetry_bridge`。

## 为什么图像走独立通道

控制总线把所有消息限制在 **64 KiB**（`common/protocol.hpp` 的 `max_payload_bytes`，经 ZMQ `maxmsgsize` 生效），而一帧彩色 JPEG 通常是 30–150 KiB，会被总线静默丢弃。所以按 SAD §03 的规则：**相机帧不上控制总线**，走自己的 PUB 通道 `tcp://127.0.0.1:5558`（大 `maxmsgsize`）。

## 组件

| 进程 | 作用 |
|------|------|
| `camera_pub` | OpenCV 抓一帧相机 → JPEG 编码 → PUB 到独立图像通道（多帧消息） |
| `foxglove_bridge` | 纯转发：SUB 控制总线 5556 + 图像通道 5558 → Boost.Beast WebSocket 说 `foxglove.websocket.v1` |
| `test_publisher.py` | 造数（无相机/无总线也可测）：flight.command / arm.state / camera.image |
| `test_client.py` | 以最小 Foxglove 客户端身份验证协议（serverInfo / advertise / 二进制帧 / protobuf 还原 JPEG） |

## 消息线格式

`camera.image` 用 ZMQ multipart（这样消费端能还原帧号与采集时间，尽管 ZMQ 消息本身无类型）：

- **Frame0** = topic（`camera.image`）
- **Frame1** = JPEG 字节（二进制）
- **Frame2** = JSON 元数据 `{ "frame": N, "timestamp": <µs epoch>, "width": W, "height": H, "format": "jpeg" }`

数据 topic（`flight.command` / `arm.state` / …）沿用总线约定：两帧 `topic + JSON`，JSON 里带 `timestamp`（µs）。**所有消息都带时间戳**，桥把 µs 换成 ns 写进 foxglove 二进制帧头，使图像与曲线在 Studio 里时间对齐。

桥在转发时把 `camera.image` 编码为 **protobuf 的 `foxglove.CompressedImage`**：`data` 字段直接放原始 JPEG 字节（**无 base64、无文本序列化**），`format="jpeg"`、`timestamp`/`frame_id` 一并带上。相比 JSON+base64，省掉了 33% 体积膨胀和 base64 编解码的 CPU/内存拷贝。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

依赖：OpenCV（仅 `camera_pub`）、Boost 1.74（Beast/Asio，仅 `foxglove_bridge`）、libzmq + 仓库内 cppzmq、nlohmann/json（头文件，CMake 配置或裸 include 均可）、protobuf 3.12（运行时，仅 `foxglove_bridge`）。

protobuf 的 `.proto` 源在 `proto/foxglove/CompressedImage.proto`；生成的 `foxglove/CompressedImage.pb.{h,cc}` 与 `foxglove_schema.hpp`（FileDescriptorSet 的 base64）已 vendor 进 `src/generated/`，构建时只需 protobuf 运行时、无需 protoc。改 `.proto` 后跑 `scripts/regenerate_proto.sh` 重新生成。

## 运行（三种方式）

### 1. 无相机快速体验（推荐先跑）

```bash
./build/foxglove_bridge &                          # 起桥，监听 ws://localhost:8765
python3 test_publisher.py &                        # 造数（需 pyzmq）
python3 test_client.py --seconds 5                 # 验证协议，期望 PASS
```

### 2. 真实相机 + 曲线

```bash
./build/foxglove_bridge &
./build/camera_pub --camera 4 --fps 30 &           # 本机彩色节点 /dev/video4/10/16，对应 --camera 4/10/16
# 数据侧接真实总线或 test_publisher.py
```

### 3. 接 Foxglove Studio

打开 Foxglove Studio → **Open connection → WebSocket → `ws://localhost:8765`**：
- 左侧 topic 列表会出现 `/arm.state`、`/flight.command`、`/camera.image`。
- 图像：在 `/camera.image` 上点 **Add panel → Image**（识别 `foxglove.CompressedImage`，自动解码 protobuf 里的 JPEG）。
- 曲线：在 `/arm.state` / `/flight.command` 上 **Add panel → Plot**，点字段即可画曲线。

## 选项

```bash
camera_pub     [--camera N] [--endpoint URL] [--fps N] [--width N] [--height N] [--quality N]
foxglove_bridge [--data-endpoint URL] [--image-endpoint URL] [--listen ADDR] [--port N]
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
