# simulation — MuJoCo ZMQ 仿真节点

加载 `models/mjcf/aviator.xml` 和 `aviator_home` keyframe，在一个进程中替代机械臂、手和相机设备节点。复用 `common/` 的两帧 JSON 协议、输入授权和 watchdog；位置力矩控制与 GLFW 窗口参考 `examples/AviatorRobot_simple`，不依赖其 IK、机器人 SDK 或业务控制器。

## 构建

完整工程在 `AVIATOR_BUILD_MUJOCO=ON` 时默认构建 `simulation`。也可以仅构建通信节点、仿真节点及仓库内的 `third_party/mujoco-3.4.0`，避免构建机器人算法依赖。两种方式均从仓库源码构建 MuJoCo，不查找 `/opt/mujoco` 或系统安装的 MuJoCo：

```bash
# Ubuntu 开发依赖：nlohmann-json3-dev libzmq3-dev libglfw3-dev libegl1-mesa-dev
# MuJoCo 源码构建还需要：libqhull-dev libccd-dev libtinyobjloader-dev libtinyxml2-dev
cmake -S . -B build/simulation-node \
  -DAVIATOR_COMMUNICATION_ONLY=ON \
  -DAVIATOR_BUILD_SIMULATION_NODE=ON \
  -DBUILD_TESTING=ON
cmake --build build/simulation-node --parallel
ctest --test-dir build/simulation-node --output-on-failure
```

MuJoCo 编译产物位于构建目录的 `third_party/install`，安装仿真节点时会一并安装 `libmujoco.so*`。lodepng、marchingcubecpp、trianglemeshdistance 同样使用仓库中的源码，不联网下载。

MuJoCo 3.4.0 已验证。相机使用 EGL OpenGL 离屏渲染，需要可用的 EGL 驱动（Mesa 软件渲染亦可），不需要 X server。GUI 使用 GLFW。`AVIATOR_BUILD_SIMULATION_NODE=OFF` 可禁用本节点，不改变原有 communication-only 的默认依赖。

## 启动

默认连接与 `aviator_bus`、`aviator_monitor` 一致的总线 `5555/5556`，`publisher_id=simulation`。上层消费者按生产者配置来源；启动标记使用普通文本，不用于授权匹配。

```bash
# 终端 1：总线
build/simulation-node/bin/aviator_bus

# 终端 2：窗口 + 相机，无命令授权时保持初始姿态
build/simulation-node/bin/simulation

# 或无头运行，仍渲染相机
build/simulation-node/bin/simulation --headless

# 无 EGL/GPU 时，仅物理仿真；camera.detection 持续报告 OFFLINE
build/simulation-node/bin/simulation --headless --no-camera

# 可选观测工具
build/simulation-node/bin/aviator_monitor
```

完整工程构建时，将上述 `build/simulation-node/bin/` 换为 `build/bin/`。监控页面为 http://127.0.0.1:8081/，话题列表可查看 `arm.state`、`hand.state` 和 `camera.detection`。

窗口复用示例中的鼠标旋转、平移、缩放、R 复位视角与 Esc 退出。窗口相机与固定的仿真传感器相机独立。SIGINT/SIGTERM 正常退出；`--duration 10` 可用于有限时间运行。

默认模型路径支持从任意工作目录启动。安装时模型和网格复制到 `share/aviator/models`；自定义安装布局可用 `--model /absolute/path/aviator.xml` 指定。`--pub-endpoint` 和 `--sub-endpoint` 可覆盖默认端点；连接 `5555/5556` 前应确保对应真实设备节点未同时发布同类反馈。

## 命令授权

启动时显式安装本次测试的 control_epoch；不根据最先到达的消息自动授权。会话 UUID 与会话匹配已取消，旧 --core-session / --origin-session 参数兼容读取但不作为授权条件。以下 epoch 仅供示意，实际由控制方授权后提供：

```bash
build/simulation-node/bin/simulation --headless \
  --control-epoch aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa
```

默认命令生产者为 `aviator_core`，上游为 `flight_gateway`，可分别用 `--core-publisher` / `--origin-publisher` 设置。输入必须使用本机 `clock_id`（hostname + `-` + `/proc/sys/kernel/random/boot_id`）与 `CLOCK_MONOTONIC` 微秒。跨主机时钟映射、可靠使能服务和在线切换授权尚未实现；更换授权需重启。

- 臂、手命令有效期 50 ms，origin 有效期 100 ms；执行前再次检查。
- 相机命令有效期默认 200 ms，可用 `--camera-timeout-ms` 指定。
- 未授权、重复/乱序、过期、未来时间、跨时钟域、越界、缺侧或数组长度错误均拒绝。非法消息不更新目标和有效期限；合法 `valid=false` 立即撤销可执行性。
- 失效后臂、手保持当时的关节位置，仍启用仿真保持伺服，状态为 `SAFE`。未接纳过命令时为 `READY`。重新收到当前授权下的新鲜命令可恢复 `ACTIVE`。
- PUB/SUB 是持续目标流，发送方应周期发送新鲜输入，不提供执行确认；`accepted_command` 仅表示最近接纳目标。

## 话题与模型映射

| 方向 | Topic | 内容 |
| --- | --- | --- |
| 发布 | `arm.state` | 目标 100 Hz；双臂 7 轴位置/速度、TCP 世界位姿、设备状态和命令引用 |
| 发布 | `hand.state` | 目标 100 Hz；双手各 6 通道实际驱动位置（raw/normalized）、设备状态和命令引用 |
| 发布 | `camera.detection` | 目标 30 Hz；640×480 渲染帧对应的方向盘真值检测或失效报告 |
| 订阅 | `arm.command` | `JOINT_POSITION`，双侧各 7 个 rad 目标 |
| 订阅 | `hand.command` | `JOINT_POSITION` 或 `NORMALIZED_POSITION`，双侧各 6 个目标 |
| 订阅 | `camera.command` | `cockpit_camera` / `YOKE`，tracking_enabled、ROI、min_confidence |

所有消息为 `[Topic, JSON]` 两帧，公共字段位于根对象。`config_id=aviator-mjcf-v1`；命令若携带此字段必须匹配。

臂关节顺序：左 `AR5-5_07L-W4C4A2_joint_1..7`，右 `AR5-5_07R-W4C4A2_joint_1..7`。手关节顺序：`{left,right}_thumb_1_joint`、`thumb_2_joint`、`index_1_joint`、`middle_1_joint`、`ring_1_joint`、`little_1_joint`。其他手指关节由模型的 mimic 等式驱动。归一化 1 表示张开（MJCF 下限），0 表示闭合（MJCF 上限）。手部反馈从实际积分关节位置反算驱动位置，限幅后量化为 `drive_position_raw[6]`（整数 0～1000），`drive_position_normalized[6]` 严格为 raw/1000；`feedback_available=true`，`joint_position` 和 `joint_velocity` 为 null。`commanded_drive_position_normalized` 单独回显目标，不作为实际反馈。臂反馈仍为 rad 和 rad/s。

监控仿真时，将 Monitor 配置的 `sources["arm.state"]` 和 `sources["hand.state"]` 设为 `simulation`，并避免真实设备同时发布同类状态。手部默认 URDF 行程映射可直接显示仿真反馈；硬件专用标定曲线应按仿真模型重新配置。

TCP 使用 `left_tcp/right_tcp` site，`frame_id=mujoco_world`，姿态为 `qx,qy,qz,qw`。它不是未经变换的 `robot_base` 坐标。关节反馈是真实积分结果，不直接把目标当反馈。

## 相机语义与范围

每次相机采样都实际渲染并读取 RGB 缓冲，但检测使用 MuJoCo 方向盘关节真值，不执行图像识别，也不判断遮挡。固定相机初始朝向双把手中点；把手中点在视锥和 ROI 中且跟踪命令有效时报告 `TRACKING`，置信度为 1。ROI 按原始图像左上角像素坐标检查，范围不能超出 640×480；置信度阈值须在 `[0,1]`，真值检测分数 1 满足此范围内的阈值。

roll/pitch 分别将 `roll_input_joint` / `pitch_input_joint` 的 MJCF 下限到上限线性映射为 `[-1,1]`。因此当前 home 中 pitch=0 m 对应归一化 +1；这只是版本化的仿真标定，不表示真实飞机标定。未跟踪、ROI 排除或超时为 `SEARCHING/valid=false`，角度值为 null。`--no-camera` 为 `OFFLINE`、frame_id=null。帧号与消息序号独立增长。

不向控制总线添加图像帧；当前未实现 `record.camera.*` 原始图像记录通道。未实现 `JOINT_TRAJECTORY`、`GRASP_SETPOINT`、自动抓握/焊接锁定与视觉算法，相关命令明确拒绝；`grasp_verified=false`，不会把手指闭合伪装为抓握完成。IK 与飞控到关节目标转换由上游控制器承担。

节点使用单线程调度物理、通信和渲染，模型步长为 1 ms，臂伺服参考示例的 Kp=1000、附加阻尼 80；另有力矩限幅。发布频率为尽力而为，不是硬实时保证。渲染过慢会降低频率和仿真速度；超过 100 ms 的墙钟积压会丢弃。协议时间始终是实际单调时钟，不使用 `mjData.time` 冒充采样时钟。

## 验证

`simulation_model` 检查动力学响应、关节映射、双侧完整性、限位、epoch/origin/时钟校验及文本启动标记、序号、invalid、超时保持和相机 ROI。`simulation_bus` 启动真实子进程及临时 TCP 总线，验证状态发布、命令订阅和退出。可启用 EGL 集成测试：

```bash
cmake -S . -B build/simulation-node -DAVIATOR_SIMULATION_TEST_EGL=ON
cmake --build build/simulation-node --parallel
ctest --test-dir build/simulation-node -R simulation --output-on-failure
```

测试端点使用随机空闲端口，不占用生产总线。
