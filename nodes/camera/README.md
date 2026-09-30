# camera

**位姿规范：** ZMQ `camera.detection.pose.position` 的 `x/y/z` 统一使用 **米（m）**；
`pose.orientation` 为无量纲单位四元数，字段/数组约定顺序为 **`(qx, qy, qz, qw)`**（`qw` 为标量，放在最后）。
位姿表示目标坐标系到相机坐标系的变换：`p_camera = R(q) * p_target + position`；相机 +X 向右、+Y 向下、+Z 向前。
预览、终端及订阅示例的位置也使用米，诊断用 RPY 欧拉角使用度（deg）。旧版发布的位置为毫米，
接收端升级后应移除原有的 `/1000` 转换；历史 MCAP 数据不会自动换算。

Python RealSense + ChArUco/AprilTag 节点。`config/camera.yaml` 的 `detector.type` 每次只选择一种检测器。节点采集彩色帧并发布 `camera.detection` 到业务总线；启用图像记录时将原始 RGB8 帧通过独立 ZMQ PUSH 入口交给 `aviator_logger`，仅在 `record_depth: true` 时额外采集并发送 Z16 深度。相机进程不写 MCAP，也不选择图像编码器。

以下命令均在仓库根目录执行。本机 Python 环境为 `~/miniconda3/envs/apriltag_realsense/bin/python`。
完整构建的可执行文件位于 `build/bin`；若使用通信独立构建，将相应路径替换为 `build/communication/bin`。

## 只看识别效果（不录制）

```bash
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --camera-id cockpit --show --print-pose
```

此命令不需要 Logger，也不依赖 bus 才能显示本地画面；若要让其他节点收到 `camera.detection`，
需启动 `./build/bin/aviator_bus`。相机当前配置选择 **ChArUco**，1280×720、30 fps，
启动热身 5 秒后开始发布；预览和打印可在热身期间查看。未显式开启时，预览和打印均关闭。

## 检测与 MCAP 录制

使用已安装 `pyrealsense2`、`opencv-contrib-python`、`numpy`、`pyzmq`、`PyYAML` 的 Python 环境；选择 AprilTag 时还需要 `apriltag` 包。程序直接从仓库运行，不需要 CMake 构建。

先在 `config/recording.yaml` 中将 `camera.mode` 设为 `raw` 或 `compressed`，并生成一个会话 UUID。`aviator_bus` 不需要会话标识；`aviator_logger` 与 camera 节点必须使用**同一个** session，才能把业务消息与图像记录关联到同一次会话。两者若都省略 `--session`，会各自生成不同的随机 UUID（logger 在 `main.cpp`、camera 在 `main.py` 里分别调用 `new_session_id()` / `session_id()`），导致关联不上。建议生成一次、写进文件，两个节点都读它：

```bash
cat /proc/sys/kernel/random/uuid > /tmp/aviator_session.uuid
cat /tmp/aviator_session.uuid           # 记下备用
```

然后在三个终端分别运行（bus 不用 session）：

```bash
# 终端 1
./build/bin/aviator_bus

# 终端 2
record_dir="output/$(cat /tmp/aviator_session.uuid)"
mkdir -p "$record_dir"
./build/bin/aviator_logger --config config/recording.yaml \
  --output "$record_dir/data.mcap" --image-output "$record_dir/images.mcap" \
  --session "$(cat /tmp/aviator_session.uuid)"

# 终端 3
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --recording-config config/recording.yaml \
  --camera-id cockpit --session "$(cat /tmp/aviator_session.uuid)"
```

`--session` 只做标识、不做格式校验，但务必用真随机 UUID：不要照抄固定的示例值，否则多次运行会共享同一 session，记录会混到一起。输出目录必须已存在。只需检测时省略 `--recording-config`；`camera.mode: disabled` 也不创建图像发送器和深度流。`detection_subscriber.py` 可用于订阅测试；`--help` 列出采集和 ChArUco 参数覆盖项。

同一次录制只生成一次 UUID，运行中不要重写会话文件。已有 bus 可直接复用，不要重复启动。
停止时先退出相机，再停止 Logger，最后按需停止 bus；Logger 正常收尾后生成最终 MCAP。

| 文件 | 内容 |
| --- | --- |
| `output/<session>/data.mcap` | `camera.detection` 等业务 JSON；手节点运行时也包含 `hand.state` 和总线上收到的 `hand.command`。 |
| `output/<session>/images.mcap` | 彩色图像，以及按配置启用的深度图像。 |

当前 [recording.yaml](../../config/recording.yaml) 为 `compressed`、`h264`、`software`，
使用 CPU 编码，无需显卡，默认只录彩色。改为 `raw` 可保存原始像素；深度由
`camera.sources[].record_depth` 控制。省略 `--image-output` 时，图像路径由数据路径派生为 `data.images.mcap`。
两个进程应读取同一份仓库配置；`build/bin/config/recording.yaml` 是另一份文件，避免修改一份却运行另一份。

## 实时预览与位姿打印（默认关闭）

在原启动命令末尾添加 `--show --print-pose`，即可同时看到实时识别效果和终端位姿：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --recording-config config/recording.yaml \
  --camera-id cockpit --session "$(cat /tmp/aviator_session.uuid)" \
  --show --print-pose
```

同一设备只运行一个相机采集进程；先停止旧相机节点，再使用新参数启动。
也可在 `config/camera.yaml` 的 `visualization` 段设置 `show: true`、`print_pose: true`。
二者默认均为 `false`，互相独立；命令行优先，`--no-show` / `--no-print-pose` 可覆盖配置关闭。

- 视频显示 AprilTag 边框/ID 或 ChArUco 标记/角点、目标坐标轴、检测状态、置信度、帧号、处理帧率、位置和姿态角。热身期间标记 `WARMUP`，此时尚未发布/记录。
- 位置为目标在相机坐标系中的 X/Y/Z，单位 **m**，与 `camera.detection.pose.position` 一致；相机坐标 +X 向右、+Y 向下、+Z 向前。
- RPY 为目标相对相机的 roll/pitch/yaw，单位 **度**，采用 `R = Rz(yaw) Ry(pitch) Rx(roll)`，与参考 `Downloads/detect.py` 的角度算法一致。欧拉角有 ±180° 跳变及奇异点；ZMQ 消息仍使用四元数。
- 终端默认每 0.5 秒输出一次，`--pose-print-interval 0.2` 或 `visualization.print_interval_s` 可调整；检测状态变化立即输出。丢失目标/PnP 未通过时显示 `SEARCHING` 和 `No valid pose`，不沿用上一次位姿。
- 窗口按 `q`、Esc 或关闭窗口会结束相机节点；Ctrl+C 同样可以退出。

绘制只作用于图像副本，Logger 收到的原始图像不含叠加文字。预览需要桌面会话和带 GUI 的 OpenCV，
无需独立显卡；无桌面时可只加 `--print-pose`。打印和预览与检测共用主循环，开启后会增加处理开销。

## 识别模式

在 [camera.yaml](../../config/camera.yaml) 中将 `detector.type` 设为 `charuco` 或 `apriltag`，修改后重启相机节点。当前配置为 `charuco`，不是同时检测两种码。

| 模式 | 当前对应参数 | 识别要求 |
| --- | --- | --- |
| `charuco` | `board_size: [5, 5]`、`DICT_4X4_50`、方格边长 0.016 m、码边长 0.015 m、`min_corners: 6` | 使用尺寸和字典匹配的 ChArUco 棋盘；5×5 指格子数。单张普通 ArUco 码不能替代整块棋盘的位姿估计。 |
| `apriltag` | `tag36h11`、`tag_id: 0`、`tag_size_m: 0.05` | 只处理目标 ID，边长应与实际打印的码匹配。 |

AprilTag 的 `use_distortion: false` 沿用参考示例的零畸变假设；若彩色图像未经去畸变，应依据标定结果改为 `true`。`charuco.min_corners` 及其命令行覆盖仅在 ChArUco 模式使用。配置不需要的检测器不会初始化。板尺寸填写错误会导致位姿尺度错误，即使画面上能识别到码。

两种模式都发布 `status`、`confidence`、相机坐标系下的 `pose`，位置单位为 m；额外的 `detector` 标明当前模式。AprilTag 在识别到目标码时还附带 `tag_id` 和原始 `decision_margin`。AprilTag 的 `confidence` 是 `decision_margin / confidence_margin` 截断到 `[0,1]` 的启发式值，不是概率；低于 `min_decision_margin` 或 PnP 失败时 `status=SEARCHING`、`valid=false`、`pose=null`。AprilTag 位姿坐标轴沿用所提供示例的四角点顺序。

## 确认总线收到位姿

在 bus 和相机节点运行时，另开终端执行：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/detection_subscriber.py \
  --endpoint tcp://127.0.0.1:5556 --max-msgs 10 --raw
```

收到 10 条消息后退出（消息数不是目标识别成功次数）；没有消息时继续等待，可 Ctrl+C 退出。
`TRACKING`、`valid=true` 和非空 `pose` 表示得到当前位姿；`SEARCHING`、`pose=null` 表示当前无有效位姿。
本地 `--print-pose` 输出不等于订阅端已经收到数据。

| 现象 | 检查项 |
| --- | --- |
| 没有窗口或没有终端位姿 | 分别加 `--show`、`--print-pose`；确认修改的是 `--config` 指向的 YAML。 |
| 无桌面或 OpenCV 不支持 GUI | 使用 `--no-show --print-pose`；视频需在桌面会话中使用带 GUI 的 OpenCV。 |
| 能看到码但一直 SEARCHING | 核对 detector 类型、字典/ID、棋盘规格、角点数和清晰度；ChArUco 画出部分标记不代表已满足位姿求解条件。 |
| 预览正常但订阅端无消息 | 检查 5 秒热身是否结束、bus 是否运行、相机 PUB 是否连接 5555、订阅端是否连接 5556。 |
| 相机设备被占用 | 关闭旧相机节点及其他 RealSense 采集程序，同一设备仅保留一个采集进程。 |
| 只有业务 MCAP，没有图像 | 核对 `--recording-config`、`camera.mode`、匹配的 `--camera-id`、Logger 5557 入口及首帧记录日志。 |

## 数据关联与记录

检测与图像使用相同的 `publisher_id + session_id + camera_id + frame_id`。`frame_id` 是本次相机会话内收到的彩色帧计数；图像元数据的 `sequence` 是各 RealSense 流的硬件帧号。图像在推理前提交给有界发送队列，彩色 BGR8 转为 RGB8，深度保留 Z16 和 `depth_scale`，并保存内参及到彩色流的外参。`sample_mono_us` 表示主机收到 frameset 的单调时间，传感器时间戳另存于元数据。

Logger 根据 [recording.yaml](../../config/recording.yaml) 选择原始保存或 H.264/H.265 彩色有损编码与 Zstd 深度无损编码；详见 [Logger 说明](../aviator_logger/README.md)。`queued-to-zmq` 只是生产者成功提交到 ZMQ，不保证已写盘。生产者、网络和 Logger 队列都可能丢帧，结束后应检查双方计数及 MCAP `camera_recording` 元数据。

采集和检测目前共用主循环，不能保证读取每个 30 Hz 传感器样本；D436 实机吞吐、压缩码率和长时间运行仍需验收。
