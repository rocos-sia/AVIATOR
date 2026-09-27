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

默认连接隔离测试总线 `6555/6556`，`publisher_id=simulation`。上层消费者应将此生产者和启动时打印的会话登记到测试授权配置。

```bash
# 终端 1：独立测试总线
build/simulation-node/bin/aviator_bus \
  --input tcp://127.0.0.1:6555 --output tcp://127.0.0.1:6556 \
  --lock-file /tmp/aviator-simulation-bus.lock

# 终端 2：窗口 + 相机，无命令授权时保持初始姿态
build/simulation-node/bin/simulation

# 或无头运行，仍渲染相机
build/simulation-node/bin/simulation --headless

# 无 EGL/GPU 时，仅物理仿真；camera.detection 持续报告 OFFLINE
build/simulation-node/bin/simulation --headless --no-camera

# 可选观测工具
build/simulation-node/bin/aviator_monitor --subscribe tcp://127.0.0.1:6556
```

窗口复用示例中的鼠标旋转、平移、缩放、R 复位视角与 Esc 退出。窗口相机与固定的仿真传感器相机独立。SIGINT/SIGTERM 正常退出；`--duration 10` 可用于有限时间运行。

默认模型路径支持从任意工作目录启动。安装时模型和网格复制到 `share/aviator/models`；自定义安装布局可用 `--model /absolute/path/aviator.xml` 指定。`--pub-endpoint` 和 `--sub-endpoint` 可覆盖默认端点；连接 `5555/5556` 前应确保对应真实设备节点未同时发布同类反馈。

## 命令授权

启动时显式安装本次测试 Core 的 session、control_epoch 和上游 session；不根据最先到达的消息自动授权。以下 UUID 仅供本次联调示意，实际运行时由发送方生成新会话并传入：

```bash
build/simulation-node/bin/simulation --headless \
  --core-session 22222222-2222-4222-8222-222222222222 \
  --control-epoch aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa \
  --origin-session 11111111-1111-4111-8111-111111111111
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
| 发布 | `hand.state` | 目标 100 Hz；双手各 6 个独立关节位置/速度、设备状态和命令引用 |
| 发布 | `camera.detection` | 目标 30 Hz；640×480 渲染帧对应的方向盘真值检测或失效报告 |
| 订阅 | `arm.command` | `JOINT_POSITION`，双侧各 7 个 rad 目标 |
| 订阅 | `hand.command` | `JOINT_POSITION` 或 `NORMALIZED_POSITION`，双侧各 6 个目标 |
| 订阅 | `camera.command` | `cockpit_camera` / `YOKE`，tracking_enabled、ROI、min_confidence |

所有消息为 `[Topic, JSON]` 两帧，公共字段位于根对象。`config_id=aviator-mjcf-v1`；命令若携带此字段必须匹配。

臂关节顺序：左 `AR5-5_07L-W4C4A2_joint_1..7`，右 `AR5-5_07R-W4C4A2_joint_1..7`。手关节顺序：`{left,right}_thumb_1_joint`、`thumb_2_joint`、`index_1_joint`、`middle_1_joint`、`ring_1_joint`、`little_1_joint`。其他手指关节由模型的 mimic 等式驱动。归一化 0/1 分别映射到各驱动关节 MJCF 下限/上限；反馈始终为 rad 和 rad/s。

TCP 使用 `left_tcp/right_tcp` site，`frame_id=mujoco_world`，姿态为 `qx,qy,qz,qw`。它不是未经变换的 `robot_base` 坐标。关节反馈是真实积分结果，不直接把目标当反馈。

## 相机语义与范围

每次相机采样都实际渲染并读取 RGB 缓冲，但检测使用 MuJoCo 方向盘关节真值，不执行图像识别，也不判断遮挡。固定相机初始朝向双把手中点；把手中点在视锥和 ROI 中且跟踪命令有效时报告 `TRACKING`，置信度为 1。ROI 按原始图像左上角像素坐标检查，范围不能超出 640×480；置信度阈值须在 `[0,1]`，真值检测分数 1 满足此范围内的阈值。

roll/pitch 分别将 `roll_input_joint` / `pitch_input_joint` 的 MJCF 下限到上限线性映射为 `[-1,1]`。因此当前 home 中 pitch=0 m 对应归一化 +1；这只是版本化的仿真标定，不表示真实飞机标定。未跟踪、ROI 排除或超时为 `SEARCHING/valid=false`，角度值为 null。`--no-camera` 为 `OFFLINE`、frame_id=null。帧号与消息序号独立增长。

不向控制总线添加图像帧；当前未实现 `record.camera.*` 原始图像记录通道。未实现 `JOINT_TRAJECTORY`、`GRASP_SETPOINT`、自动抓握/焊接锁定与视觉算法，相关命令明确拒绝；`grasp_verified=false`，不会把手指闭合伪装为抓握完成。IK 与飞控到关节目标转换由上游控制器承担。

节点使用单线程调度物理、通信和渲染，模型步长为 1 ms，臂伺服参考示例的 Kp=1000、附加阻尼 80；另有力矩限幅。发布频率为尽力而为，不是硬实时保证。渲染过慢会降低频率和仿真速度；超过 100 ms 的墙钟积压会丢弃。协议时间始终是实际单调时钟，不使用 `mjData.time` 冒充采样时钟。

## 验证

`simulation_model` 检查动力学响应、关节映射、双侧完整性、限位、会话/epoch/origin/时钟校验、序号、invalid、超时保持和相机 ROI。`simulation_bus` 启动真实子进程及临时 TCP 总线，验证状态发布、命令订阅和退出。可启用 EGL 集成测试：

```bash
cmake -S . -B build/simulation-node -DAVIATOR_SIMULATION_TEST_EGL=ON
cmake --build build/simulation-node --parallel
ctest --test-dir build/simulation-node -R simulation --output-on-failure
```

测试端点使用随机空闲端口，不占用生产总线。
