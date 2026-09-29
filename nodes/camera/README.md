# camera

Python RealSense + ChArUco/AprilTag 节点。`config/camera.yaml` 的 `detector.type` 每次只选择一种检测器。节点采集彩色帧并发布 `camera.detection` 到业务总线；启用图像记录时同时采集 Z16 深度，将原始 RGB8/Z16 帧通过独立 ZMQ PUSH 入口交给 `aviator_logger`。相机进程不写 MCAP，也不选择图像编码器。

## 运行

使用已安装 `pyrealsense2`、`opencv-contrib-python`、`numpy`、`pyzmq`、`PyYAML` 的 Python 环境；选择 AprilTag 时还需要 `apriltag` 包。程序直接从仓库运行，不需要 CMake 构建。

先在 `config/recording.yaml` 中将 `camera.mode` 设为 `raw` 或 `compressed`，并生成一个会话 UUID。`aviator_bus` 不需要会话标识；`aviator_logger` 与 camera 节点必须使用**同一个** session，才能把业务消息与图像记录关联到同一次会话。两者若都省略 `--session`，会各自生成不同的随机 UUID（logger 在 `main.cpp`、camera 在 `main.py` 里分别调用 `new_session_id()` / `session_id()`），导致关联不上。建议生成一次、写进文件，两个节点都读它：

```bash
uuidgen > /tmp/aviator_session.uuid     # 也可用 cat /proc/sys/kernel/random/uuid
cat /tmp/aviator_session.uuid           # 记下备用
```

然后在三个终端分别运行（bus 不用 session）：

```bash
# 终端 1
./build/communication/bin/aviator_bus

# 终端 2
./build/communication/bin/aviator_logger --config config/recording.yaml \
  --output /mnt/aviator/session/data.mcap \
  --session "$(cat /tmp/aviator_session.uuid)"

# 终端 3
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --recording-config config/recording.yaml \
  --camera-id cockpit --session "$(cat /tmp/aviator_session.uuid)"
```

`--session` 只做标识、不做格式校验，但务必用真随机 UUID：不要照抄固定的示例值，否则多次运行会共享同一 session，记录会混到一起。输出目录必须已存在。只需检测时省略 `--recording-config`；`camera.mode: disabled` 也不创建图像发送器和深度流。`--show` 显示预览。`detection_subscriber.py` 可用于订阅测试；`--help` 列出采集和 ChArUco 参数覆盖项。

## 识别模式

在 [camera.yaml](../../config/camera.yaml) 中将 `detector.type` 设为 `apriltag` 或 `charuco`；默认 AprilTag。AprilTag 配置采用用户提供示例中的 `tag36h11`、ID 0、50 mm 标签，只处理目标 ID。`tag_size_m` 必须与实际打印的码尺寸一致。`apriltag.use_distortion: false` 沿用示例的零畸变假设；若彩色图像未经去畸变，应依据标定结果改为 `true`。ChArUco 保留原有板配置；`charuco.min_corners` 及其命令行覆盖仅在 ChArUco 模式使用。配置不需要的检测器不会初始化。

两种模式都发布 `status`、`confidence`、相机坐标系下的 `pose`，位置单位为 mm；额外的 `detector` 标明当前模式。AprilTag 在识别到目标码时还附带 `tag_id` 和原始 `decision_margin`。AprilTag 的 `confidence` 是 `decision_margin / confidence_margin` 截断到 `[0,1]` 的启发式值，不是概率；低于 `min_decision_margin` 或 PnP 失败时 `status=SEARCHING`、`valid=false`、`pose=null`。AprilTag 位姿坐标轴沿用所提供示例的四角点顺序。

## 数据关联与记录

检测与图像使用相同的 `publisher_id + session_id + camera_id + frame_id`。`frame_id` 是本次相机会话内收到的彩色帧计数；图像元数据的 `sequence` 是各 RealSense 流的硬件帧号。图像在推理前提交给有界发送队列，彩色 BGR8 转为 RGB8，深度保留 Z16 和 `depth_scale`，并保存内参及到彩色流的外参。`sample_mono_us` 表示主机收到 frameset 的单调时间，传感器时间戳另存于元数据。

Logger 根据 [recording.yaml](../../config/recording.yaml) 选择原始保存或 H.264/H.265 彩色有损编码与 Zstd 深度无损编码；详见 [Logger 说明](../aviator_logger/README.md)。`queued-to-zmq` 只是生产者成功提交到 ZMQ，不保证已写盘。生产者、网络和 Logger 队列都可能丢帧，结束后应检查双方计数及 MCAP `camera_recording` 元数据。

采集和检测目前共用主循环，不能保证读取每个 30 Hz 传感器样本；D436 实机吞吐、压缩码率和长时间运行仍需验收。
