# 配置

`grasp.json` 的工具变换分别放在 `tool.left` 和 `tool.right` 下，各含 `position` 和
`quaternion`。`position` 单位为米，表示在对应法兰坐标系中的工具中心位置；四元数顺序为
`[w, x, y, z]`，表示工具坐标系相对法兰的朝向。修改 `tool.left.position[1]` 只调整左侧
工具的 Y 偏移，右侧同理。抓握、Servo、Rokae TCP、MuJoCo 工具及规划碰撞圆柱都使用各自变换。
`tool.radius/length/mass/mount_radius` 继续共用；安装杆几何仍由模型文件定义。
旧格式 `tool.position/quaternion` 仍可作为两侧共用配置，但不能与左右配置混用；新格式必须同时提供两侧。

抓握时从 home 用一条 Ruckig 双臂同步关节轨迹到达最终抓握目标，不经过预接近停靠点。
`grasp.json` 的 `hand_closing_distance` 默认 `0.07`（米，即 70 mm）：各侧按规划关节轨迹的
FK tool 中心到对应抓握目标的距离分别触发闭合，以五次平滑曲线从 `core_hand.open` 插值到
`core_hand.close`，与机械臂轨迹同时到达终点。执行时跟随机械臂反馈中的轨迹游标，手命令以
50 Hz 发布，终点立即发布；不按规划开始后的墙钟时间提前推进。若轨迹进入范围后又离开，
规划会报错，避免在范围外继续闭合。`approach_distance`（当前 0.06 m）只用于松开后的撤离。
实际角度同步误差需在设备上验证；终点确认仍使用 CAN/设备写入 ACK，不代表已抓稳。

- `flight.yaml`：Flight Gateway 的 `joystick`（SDL2 摇杆和键盘同时可用）/ `rs422` 输入选择、SDL 摇杆索引或自动选择、键盘方向键速度/限位及状态按钮映射，详见 [Gateway 说明](../nodes/flight_gateway/README.md#键盘操作)。
- `monitor.yaml`：Monitor 的消息来源、超时、RGB 预览及显示标定。可在网页“配置”页保存并即时应用，详见 [Monitor 说明](../nodes/aviator_monitor/README.md)。
- `recording.yaml`：Logger 配置 v1。通过 `aviator_logger --config config/recording.yaml` 加载；`camera.mode` 选择 `disabled`、`raw` 或 `compressed`。字段、默认值、校验及 CLI 覆盖规则见 [Logger 说明](../nodes/aviator_logger/README.md)。
- `camera.yaml`：相机节点的设备选择、彩色采集、深度尺寸与 ChArUco/AprilTag 参数。`detector.type` 每次只选择一种识别模式，当前选择 `charuco`。`visualization.show`、`visualization.print_pose` 默认关闭；可用 `--show --print-pose` 临时开启视频和位姿输出，`print_interval_s` 默认 0.5 秒。传入启用图像的 `--recording-config` 后发送 RGB8；对应 `camera.sources[].record_depth: true` 时才额外采集和记录 Z16。`--camera-id` 必须匹配录制配置。完整命令见 [相机说明](../nodes/camera/README.md)。
- SocketCAN 手节点配置位于 [`inspire_hand.yaml`](inspire_hand.yaml)；RH56FTP Modbus TCP 节点通过命令行指定 IP/端口，说明见 [`rh56ftp_hand`](../nodes/rh56ftp_hand/README.md)。
- 方向盘 AprilTag 标定结果写入 [`steering_wheel_calibration.yaml`](steering_wheel_calibration.yaml)，标定和验证程序见 [`tools/steering_wheel_calibration`](../tools/steering_wheel_calibration/README.md)。
- `robot.yaml`：机械臂后端、模型与规划参数。`system.yaml`：总线、设备服务与 Core 配置；`core_hand` 控制真实手接入、双手归一化开合目标与反馈时限，当前张开为 `[0,1,1,1,1,1]`、闭合为 `[0.3,0,0,0,0,0]`。MuJoCo 后端不发布真实手命令；见 [Core 使用说明](../nodes/aviator_core/README.md#core-控制真实机械手)。

除 Monitor 网页配置外，参数修改后重启会话，不支持热加载。帧率由相机采集端决定；Logger 不隐式降采样。队列大小与压缩参数为试验初值，需实机验证。完整接入过程见 [实现说明](../docs/Logger配置与图像记录实现说明.md)。
