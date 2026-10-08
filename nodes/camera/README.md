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

先在 `config/recording.yaml` 中将 `camera.mode` 设为 `raw` 或 `compressed`。Logger 与 camera 无需共享会话 ID：camera 的检测消息和图像带相同的相机启动标记及 frame_id，Logger 将它们记录到同一批次的数据与图像文件。

在三个终端分别运行：

```bash
# 终端 1
./build/bin/aviator_bus

# 终端 2：默认以启动时间命名，同批图像自动使用 .images.mcap 后缀
./build/bin/aviator_logger --config config/recording.yaml

# 终端 3
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --recording-config config/recording.yaml \
  --camera-id cockpit
```

节点不再生成会话 UUID，`session_id` 是启动时间和进程号组成的普通文本，仅用于追踪及序号重置，不参与授权匹配。`--session` 可选，无需手动设置或与 Logger 相同。自定义批次可给 Logger 传 `--output output/batch_001.mcap`，自动配对 `output/batch_001.images.mcap`；父目录须存在。

只需检测时省略 `--recording-config`；`camera.mode: disabled` 也不创建图像发送器和深度流。`detection_subscriber.py` 可用于订阅测试；`--help` 列出采集和 ChArUco 参数覆盖项。已有 bus 可直接复用，不要重复启动。

停止时先退出相机，再停止 Logger，最后按需停止 bus；Logger 正常收尾后生成最终 MCAP。

| 文件 | 内容 |
| --- | --- |
| `aviator_<启动时间>.mcap` | `camera.detection` 等业务 JSON；手节点运行时也包含 `hand.state` 和总线上收到的 `hand.command`。 |
| `aviator_<启动时间>.images.mcap` | 彩色图像，以及按配置启用的深度图像。 |

当前 [recording.yaml](../../config/recording.yaml) 为 `compressed`、`h264`、`software`，
使用 CPU 编码，无需显卡，默认只录彩色。改为 `raw` 可保存原始像素；深度由
`camera.sources[].record_depth` 控制。省略 `--image-output` 时，图像路径由数据路径派生为 `data.images.mcap`。
两个进程应读取同一份仓库配置；`build/bin/config/recording.yaml` 是另一份文件，避免修改一份却运行另一份。

## 实时预览与原地位姿打印

在原启动命令末尾添加 `--show --print-pose`，即可同时看到实时识别效果和终端位姿：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --recording-config config/recording.yaml \
  --camera-id cockpit --session "$(cat /tmp/aviator_session.uuid)" \
  --show --print-pose
```

同一设备只运行一个相机采集进程；先停止旧相机节点，再使用新参数启动。
也可在 `config/camera.yaml` 的 `visualization` 段设置 `show: true`、`print_pose: true`。
当前配置中视频默认关闭，终端位姿打印默认开启，二者互相独立；
命令行优先，`--no-show` / `--no-print-pose` 可覆盖配置关闭。

- 视频显示 AprilTag 边框/ID 或 ChArUco 标记/角点、目标坐标轴、检测状态、置信度、帧号、处理帧率、位置和姿态角。热身期间标记 `WARMUP`，此时尚未发布/记录。
- 位置为目标在相机坐标系中的 X/Y/Z，单位 **m**，与 `camera.detection.pose.position` 一致；相机坐标 +X 向右、+Y 向下、+Z 向前。
- RPY 为目标相对相机的 roll/pitch/yaw，单位 **度**，采用 `R = Rz(yaw) Ry(pitch) Rx(roll)`，与参考 `Downloads/detect.py` 的角度算法一致。欧拉角有 ±180° 跳变及奇异点；ZMQ 消息仍使用四元数。
- 终端固定六行原地刷新：帧号/检测状态、原始码位置（m）、原始四元数（xyzw）、相对零位旋转（rad）、沿标定轴平移（m）、相对变换的三维平移向量（m）。方向盘数值直接使用本帧 `camera.detection.steering_wheel`，打印与发布一致。
- 默认每 0.5 秒刷新一次，`--pose-print-interval 0.2` 或 `visualization.print_interval_s` 可调整；检测或标定有效性变化立即刷新。用光标上移、清行和 `flush()` 替换同一块内容，行宽按终端宽度裁剪以防换行滚屏。丢码或标定失效时清除旧数值并显示 `unavailable` 及原因。
- 输出重定向到文件或管道时使用限频的普通文本记录，不写终端光标控制字符；视频预览中的 RPY 仍为度。
- 窗口按 `q`、Esc 或关闭窗口会结束相机节点；Ctrl+C 同样可以退出。

绘制只作用于图像副本，Logger 收到的原始图像不含叠加文字。预览需要桌面会话和带 GUI 的 OpenCV，
无需独立显卡；无桌面时可只加 `--print-pose`。打印和预览与检测共用主循环，开启后会增加处理开销。

## 识别模式

在 [camera.yaml](../../config/camera.yaml) 中将 `detector.type` 设为 `charuco` 或 `apriltag`，修改后重启相机节点。当前配置为 `apriltag`，不是同时检测两种码。

| 模式 | 当前对应参数 | 识别要求 |
| --- | --- | --- |
| `charuco` | `board_size: [5, 5]`、`DICT_4X4_50`、方格边长 0.016 m、码边长 0.015 m、`min_corners: 6` | 使用尺寸和字典匹配的 ChArUco 棋盘；5×5 指格子数。单张普通 ArUco 码不能替代整块棋盘的位姿估计。 |
| `apriltag` | `tag36h11`、`tag_id: 0`、`tag_size_m: 0.12` | 对应 [Apriltag120id0.pdf](../../docs/Apriltag120id0.pdf)，120 mm 为黑边外尺寸，不含白边；含白边纸面为 150 mm。 |

AprilTag 的 `use_distortion: false` 沿用参考示例的零畸变假设；若彩色图像未经去畸变，应依据标定结果改为 `true`。`charuco.min_corners` 及其命令行覆盖仅在 ChArUco 模式使用。配置不需要的检测器不会初始化。板尺寸填写错误会导致位姿尺度错误，即使画面上能识别到码。

打印时使用实际尺寸（100%），不要缩放到适合页面；实测黑边外尺寸应为 120×120 mm。
更换纸张后重启相机节点。已有方向盘标定文件不自动缩放；如果标签中心、安装姿态改变，
或原标定使用了错误的码尺寸，应重新标定零位和运动轴。

两种模式都发布 `status`、`confidence`、相机坐标系下的 `pose`，位置单位为 m；额外的 `detector` 标明当前模式。AprilTag 在识别到目标码时还附带 `tag_id` 和原始 `decision_margin`。AprilTag 的 `confidence` 是 `decision_margin / confidence_margin` 截断到 `[0,1]` 的启发式值，不是概率；低于 `min_decision_margin` 或 PnP 失败时 `status=SEARCHING`、`valid=false`、`pose=null`。AprilTag 位姿坐标轴沿用所提供示例的四角点顺序。

## 确认总线收到位姿

相机还会从 `config/steering_wheel_calibration.yaml` 读取零位矩阵和运动轴，
在同一条 `camera.detection` 中同时发布原始 `pose` 和派生的 `steering_wheel`：

```json
{
  "steering_wheel": {
    "valid": true,
    "reason": "",
    "theta_rad": -0.15,
    "translation_along_axis_m": 0.04,
    "translation_vector_m": [0.01, -0.02, 0.03],
    "axis_direction": [-0.0158, -0.6139, 0.7893],
    "axis_frame": "camera_color_optical_frame",
    "axis_error_rad": 0.01,
    "axis_match": true,
    "calibration_id": "标定内容摘要"
  }
}
```

上例仅展示字段结构。`theta_rad` 为相对标定零位、绕标定轴的带符号旋转，单位 **rad**；
`translation_along_axis_m` 为沿轴的带符号平移，单位 **m**。两者与标定工具
`validate.py` 的计算一致：`T_relative = T_current @ inverse(T_zero)`。
`translation_vector_m` 是相对变换的三维平移项，不是标签中心的位置差。
方向的正负遵循 `axis_direction`；旋转范围为 `[-pi, pi]`，没有做多圈展开。
零位或纯平移附近，旋转不足 3° 时 `axis_match=null`；旋转足够时按 5° 轴夹角输出
匹配结果。`valid` 表示已加载标定且当前目标可计算；`axis_match=false` 时不能把
依赖标定轴计算的角度当作真实方向盘转角。

`config/camera.yaml` 的 `steering_wheel.calibration_file` 相对该 camera YAML 所在目录解析，
默认是同目录的 `steering_wheel_calibration.yaml`。
`--steering-wheel-calibration /path/to/calibration.yaml` 可覆盖路径；
`steering_wheel.enabled: false` 可禁用派生计算。
运行期间约每秒检查一次标定文件，文件更新后自动重载，`calibration_id` 随使用的标定内容变化。

标定未完成、文件缺失或损坏时，相机仍发布原始 `pose`，
`steering_wheel.valid=false`、`reason=calibration_unavailable`、运动数值为 `null`。
检测丢失时为 `target_not_tracking`；相机或 AprilTag ID 与标定来源不匹配时为
`calibration_source_mismatch`。不会沿用上一帧的方向盘数值。
顶层 `valid` 仍表示码检测是否有效，使用方向盘运动的订阅端还需检查 `steering_wheel.valid`。
`calibrate.py` 和 `validate.py` 继续使用原始 `pose`，因此首次标定和重新标定仍可照常进行。

在 bus 和相机节点运行时，另开终端执行：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/detection_subscriber.py \
  --endpoint tcp://127.0.0.1:5556 --max-msgs 10 --raw
```

收到 10 条消息后退出（消息数不是目标识别成功次数）；没有消息时继续等待，可 Ctrl+C 退出。
`TRACKING`、`valid=true` 和非空 `pose` 表示得到当前位姿；`SEARCHING`、`pose=null` 表示当前无有效位姿。
订阅工具同时打印方向盘的旋转（rad）和沿轴平移（m）。
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

## Monitor 独立 RGB 预览

`config/camera.yaml` 的 `preview` 默认启用本机 `tcp://127.0.0.1:5561` JPEG PUB。预览缩放上限 640×360、15 fps、质量 80，工作线程只保留最新待编码帧。启动现有相机节点后，双 Tab Monitor 可直接订阅该通道；不需要开启 Logger 录制。

`--preview-endpoint tcp://127.0.0.1:6561` 可覆盖地址，`--preview-endpoint off` 禁用。Monitor 对应使用相同 `--preview` 地址。ZMQ 消息为 topic `camera.rgb.<camera_id>`、元数据 JSON、JPEG 三帧，frame_id/session/采样时间与检测共享身份。预览与 Logger 5557 PUSH/PULL 完全独立，不分流录制数据，不占用第二个相机进程。

无硬件验证：在相机 Python 环境运行 `python nodes/camera/test_preview.py`，检查 JPEG 解码、帧身份、最新帧覆盖及线程退出。详细接口和标定配置见 [Monitor README](../aviator_monitor/README.md)。
