# 配置

- `recording.yaml`：Logger 配置 v1。通过 `aviator_logger --config config/recording.yaml` 加载；`camera.mode` 选择 `disabled`、`raw` 或 `compressed`。字段、默认值、校验及 CLI 覆盖规则见 [Logger 说明](../nodes/aviator_logger/README.md)。
- `camera.yaml`：相机节点的设备选择、彩色采集、深度尺寸与 ChArUco/AprilTag 参数。`detector.type` 每次只选择一种识别模式，当前选择 `charuco`。`visualization.show`、`visualization.print_pose` 默认关闭；可用 `--show --print-pose` 临时开启视频和位姿输出，`print_interval_s` 默认 0.5 秒。传入启用图像的 `--recording-config` 后发送 RGB8；对应 `camera.sources[].record_depth: true` 时才额外采集和记录 Z16。`--camera-id` 必须匹配录制配置。完整命令见 [相机说明](../nodes/camera/README.md)。
- 手节点配置位于 [`inspire_hand.yaml`](inspire_hand.yaml)：包含左右 CAN 接口/ID、速度、力、安全姿态、指令超时与实际位置读取参数；控制及只读检查见 [手节点说明](../nodes/aviator_hand/README.md)。
- `robot.yaml`：机械臂后端、模型与规划参数。`system.yaml`：总线、设备服务与 Core 配置；`core_hand` 控制真实手接入、双手归一化开合目标与反馈时限，当前张开全 1、闭合全 0。MuJoCo 后端不发布真实手命令；见 [Core 使用说明](../nodes/aviator_core/README.md#core-控制真实机械手)。

参数修改后重启会话，不支持热加载。帧率由相机采集端决定；Logger 不隐式降采样。队列大小与压缩参数为试验初值，需实机验证。完整接入过程见 [实现说明](../docs/Logger配置与图像记录实现说明.md)。
