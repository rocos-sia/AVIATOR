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

记录图像时需要 `mcap==1.5.0` 和 `protobuf>=6.31.1,<7`。在相机环境执行：

```bash
python -m pip install 'mcap==1.5.0' 'protobuf>=6.31.1,<7'
```

图像直接由相机进程写入独立 MCAP，Logger 仍只记录总线 JSON。两者传入同一个 `--session`；检测消息和图像还可通过 `camera_id + frame_id` 对齐。相机把采集帧复制到最多两帧的队列，后台线程进行 PNG 编码与 MCAP 写盘。

终端一：

```bash
SESSION_ID=$(cat /proc/sys/kernel/random/uuid)
printf 'session: %s\n' "$SESSION_ID"
./build/communication/bin/aviator_logger --session "$SESSION_ID" \
  --output /mnt/aviator/session/data.mcap
```

终端二：把上一终端打印的 UUID 填入 `SESSION_ID`。

```bash
SESSION_ID="粘贴终端一打印的UUID"
~/miniconda3/envs/apriltag_realsense/bin/python examples/aruco_camera/camera_publisher.py \
  --session "$SESSION_ID" --image-output /mnt/aviator/session/camera.mcap \
  --png-compression 6
```

输出目录须预先存在。相机先写 `camera.mcap.partial`，Ctrl+C 或 SIGTERM 正常退出并核对 MCAP 索引后发布 `camera.mcap`；已存在文件不会被覆盖。写盘失败时保留 `.partial`、报告错误并停用图像记录，检测仍继续。PNG 是无损格式；`--png-compression` 可设 0–9，较高等级通常更省磁盘但编码更慢。默认 6，实际 30 Hz 是否无丢帧需在目标设备上测试。相机退出时打印已保存帧数和队列/编码丢帧数。

## 配置（YAML）

相机型号/分辨率与 ChArUco 尺寸从 YAML 读取，默认 [config/camera.yaml](../../config/camera.yaml)，
`--config` 可指向其他文件；命令行 `--width/--height/--fps/--warmup-s/--square-length/--marker-length/
--board-size/--min-corners` 仅在显式给出时覆盖 YAML。

```yaml
camera:
  model: "D436"          # 型号标签；多设备时按名称子串匹配选择（不区分大小写）
  serial: ""             # 序列号；非空则精确选择该设备（优先于 model）
  width: 1280
  height: 720
  fps: 30
  warmup_s: 5.0          # 启动后跳过发布的热身秒数（0 禁用）

charuco:
  board_size: [5, 5]        # 格子数 (列, 行)
  square_length: 0.016      # 方块边长，米
  marker_length: 0.015      # 标记边长，米
  dictionary: "DICT_4X4_50" # cv2.aruco 预定义字典名
  min_corners: 6
```

设备选择顺序：`serial`（精确）→ `model`（名称子串）→ 第一台。

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

发布端常用参数：`--show`（开可视化窗口）、`--camera-id`、`--warmup-s`（启动后跳过发布的
热身秒数，0 禁用；默认以 YAML 为准）、`--endpoint` / `--bind`。
采集/检测几何参数来自 YAML，`--config` 指定文件；`--width/--height/--fps/--square-length/
--marker-length/--board-size/--min-corners` 显式给出时覆盖 YAML。两脚本均 `--help` 查看完整选项。

热身期内照常采图与检测但不发布（让 auto-exposure 收敛）；`sequence` 仍从 1 起，`frame_id`
按采集帧递增，故首条消息的 `frame_id` 是热身后的帧号。

## 说明 / 限制

- `confidence` 无 ChArUco 原生分数，采用「检出角点占比」启发式（板 5x5 = 25 角点），
  仅作健康指示；真正的接受阈值仍应在消费端（aviator_core）按配置的安全下限判定。
- PUB/SUB 无送达确认，订阅端晚于发布端启动会漏掉最初几条消息；测试请先起订阅端/总线。
- 本目录为 Python 测试程序，独立于 `nodes/`（C++）构建；如需正式并入节点树，再按
  `nodes/camera` 的 C++ 结构迁移并接入 CMake。
