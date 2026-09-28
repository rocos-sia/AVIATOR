# aruco_camera — 相机 ChArUco 姿态检测测试程序

把 `examples/get_Aruco.py` 的 RealSense + ChArUco 板 6DOF 姿态检测接入 AVIATOR
ZMQ 节点体系，发布到 `camera.detection` topic，并提供一个订阅端做验证。

## 文件

| 文件 | 作用 |
| --- | --- |
| `camera_publisher.py` | RealSense + ChArUco 检测 → PUB `camera.detection`（发布节点） |
| `detection_subscriber.py` | SUB 订阅、解码、打印姿态（测试接收端） |

## 运行环境

```bash
~/miniconda3/envs/apriltag_realsense/bin/python  # 已装 zmq + cv2 + pyrealsense2
```

无需额外 pip 安装。

## 消息格式（`camera.detection` / `CameraDetection`）

复用 [AVIATOR ZMQ 协议 §12](../../docs/AVIATOR_ZMQ协议格式说明.md) 的公共 Header 与
`camera.detection` 已有字段（`camera_id`/`frame_id`/`image_width`/`image_height`/
`status`/`confidence`），并在 body 中**补充一个 `pose` 块**承载 6DOF 姿态（该字段
为新增补充，不改动已冻结的 YOKE 字段；body 为自由 JSON，`protocol.cpp` 仅强校验
`flight.command`/`arm`/`hand`，故纯增量、无破坏）。

```json
{
  "msg_type": "CameraDetection", "version": "1.0", "sequence": 6141,
  "timestamp": 1790121599999000, "sample_mono_us": 12345659000,
  "clock_id": "hostA-boot1", "publisher_id": "camera",
  "session_id": "44444444-4444-4444-8444-444444444444", "valid": true,
  "camera_id": "cockpit_camera", "frame_id": 9001,
  "image_width": 1280, "image_height": 720,
  "status": "TRACKING", "confidence": 0.96,
  "pose": {
    "position":    {"x": 12.3, "y": -4.5, "z": 340.0},
    "orientation": {"qx": 0.01, "qy": 0.02, "qz": 0.03, "qw": 0.999}
  }
}
```

### 字段说明

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `msg_type` | string | 恒为 `CameraDetection` |
| `version` / `sequence` / `clock_id` / `publisher_id` / `session_id` / `valid` | — | 公共 Header，与 `common/protocol.hpp` 一致；`publisher_id` 恒为 `camera` |
| `timestamp` | uint53 | 检测快照生成时刻，UTC 微秒 |
| `sample_mono_us` | uint53 | 图像采集时刻，单调时钟微秒 |
| `camera_id` | string | 相机流标识（`--camera-id`，默认 `cockpit_camera`） |
| `frame_id` | uint53 | 本会话内采集帧号，从 1 递增 |
| `image_width` / `image_height` | uint53 | 实际彩色帧宽高（像素） |
| `status` | enum | `TRACKING`（solvePnP 成功且角点达标）/ `SEARCHING`（有图但未达有效姿态） |
| `confidence` | number∈[0,1] | 检出的 ChArUco 角点比例（`len(charuco_ids)/总角点数`）；无角点为 0 |
| `valid` | bool | `status == "TRACKING"` |
| `pose` | object\|null | 有效时为下述 6DOF；无效时为 `null` |
| `pose.position.{x,y,z}` | number | 板原点在**相机坐标系**下的位置，单位 **mm**（`tvec * 1000`） |
| `pose.orientation.{qx,qy,qz,qw}` | number | 板姿态四元数（w 在后，模长归一），由 `cv2.Rodrigues` 转换 |

> 时间戳语义：`sample_mono_us` 取帧到达时刻（`time.monotonic_ns()`），`timestamp`
> 取快照打包时刻（`time.time_ns()`），与 §12「采集时刻 / 生成时刻」一致。
> 探测失败帧仍会发布（`status=SEARCHING`、`valid=false`、`pose=null`），便于监控端
> 感知丢失状态。

## 运行

### 方式 A：经 AVIATOR 总线（生产路径）

```bash
# 终端 1：启动总线（XSUB:5555 <- publishers，XPUB:5556 -> subscribers）
./build/nodes/aviator_bus/aviator_bus          # 或按实际构建输出路径

# 终端 2：发布
~/miniconda3/envs/apriltag_realsense/bin/python camera_publisher.py

# 终端 3：订阅验证
~/miniconda3/envs/apriltag_realsense/bin/python detection_subscriber.py
```

### 方式 B：直连（不启动总线）

```bash
# 终端 1：订阅端 bind 到发布端将连接的端口
~/miniconda3/envs/apriltag_realsense/bin/python detection_subscriber.py \
    --bind --endpoint tcp://127.0.0.1:5555

# 终端 2：发布端 connect 同一端口（默认端点即为 5555）
~/miniconda3/envs/apriltag_realsense/bin/python camera_publisher.py
```

发布端常用参数：`--show`（开可视化窗口）、`--camera-id`、`--min-corners`、
`--board-size 5x5`、`--square-length` / `--marker-length`、`--endpoint` / `--bind`。
两脚本均 `--help` 查看完整选项。

## 说明 / 限制

- `confidence` 无 ChArUco 原生分数，采用「检出角点占比」启发式（板 5x5 = 25 角点），
  仅作健康指示；真正的接受阈值仍应在消费端（aviator_core）按配置的安全下限判定。
- PUB/SUB 无送达确认，订阅端晚于发布端启动会漏掉最初几条消息；测试请先起订阅端/总线。
- 本目录为 Python 测试程序，独立于 `nodes/`（C++）构建；如需正式并入节点树，再按
  `nodes/camera` 的 C++ 结构迁移并接入 CMake。
