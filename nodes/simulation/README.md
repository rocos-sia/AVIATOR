# simulation — 臂、手一体的 MuJoCo 设备节点

`simulation` 在同一个物理场景中实现双臂、RH56FTP 双手和相机仿真，替代
`manipulator` 与 `rh56ftp_hand`。Core 使用相同配置、可靠服务和消息协议，
不判断硬件类型；`robot.yaml` 不再包含 `backend`。

## 启动

在不同终端执行，等待 simulation 的 `READY simulation` 再启动 Core：

```bash
./build/debug/bin/aviator_bus --config config/system.yaml
./build/debug/bin/simulation --config config/system.yaml
./build/debug/bin/aviator_core_managed --config config/system.yaml
./build/debug/bin/flight_gateway
```

无桌面/GPU时使用 `simulation --config config/system.yaml --headless --no-camera`。
`--headless` 只关闭窗口，`--no-camera` 关闭 EGL 渲染并持续报告相机 OFFLINE。
`--duration 10` 可限制运行时间。完整启动脚本也支持：

```bash
AVIATOR_BIN="$PWD/build/debug/bin" ./scripts/start_aviator.sh --simulation
```

同一设备服务端口和控制总线上，真机设备组与 simulation 二选一，不能同时运行。
仿真不加载 Rokae SDK，也不访问 Modbus 硬件。窗口、相机渲染、设备通信与
物理执行分离，渲染不会占用设备通信线程。

## 配置与接口

- `--config`：与 Core 使用同一个 `system.yaml`。设备服务默认绑定
  `manipulator_service: tcp://127.0.0.1:5558`，总线默认 5555/5556。
- `robot.yaml` 的 `model` 指向含双臂和完整双手的 `models/mjcf/aviator.xml`；
  `viewer` 控制窗口。`wheel_initial`、URDF 限位、工具变换和制动参数与真机共用。
- `--model` 可覆盖模型路径；`--pub-endpoint` / `--sub-endpoint` 可覆盖总线。
- `arm.state` 的逻辑发布者为 `manipulator`；手部发布者读取
  `system.yaml` 的 `core_hand.publisher_id`（缺省 `rh56ftp_hand`）。
  Monitor 的来源配置对两种设备相同。
- 臂服务包括 `describe`、`authorize`、`enable`、`disable`、`stop`、`lock`、
  `unlock`、`reset_fault`、`get_result`。请求/回复的逻辑 target/server_id
  均为 `manipulator`。共享设备服务实现保留请求去重、10 秒期限、轨迹游标、
  50 ms 指令 watchdog、100 ms origin watchdog和本地制动。
- 臂命令使用 `JOINT_TRAJECTORY` / `SYNCHRONIZED_TICKS`，由 Core 的
  `authorize` 请求安装 control_epoch；不再通过 CLI 给机械臂预设授权。
- 手部接受 `NORMALIZED_POSITION` 与 `GRASP_SETPOINT`。六通道顺序为
  拇指旋转、拇指弯曲、食指、中指、无名指、小指；归一化 1=张开，0=闭合。
  与 RH56FTP 相同，首次合法命令绑定发布者/epoch/origin，反馈携带
  `accepted_command`（包含原始采样时间）、`command_valid`、`feedback_only`
  和各侧实际驱动位置。非法消息不刷新授权或 ACK；`valid=false` 或 100 ms
  超时使双手回安全张开目标。弯曲通道持续闭合且实测位置在五秒内变化不超过
  10 个 raw 单位时冻结目标，拇指旋转除外，并分别回显 requested/commanded/closing_hold。
  手部没有独立 RPC，ACK 通过 `hand.state` 返回。
- 手部位置来自实际积分关节，按 0..1000 量化，不以目标冒充反馈；
  力、电流、温度等硬件寄存器不可用时返回空数组，`grasp_verified=false`。
- `lock` / `unlock` 同时切换抓取 weld；仿真测得轮盘位形放在
  `wheel_measurement`，与 `wheel_reference` 分开。

## 相机

启动后自动采集并发布 `publisher_id=simulation`、`camera_id=cockpit` 的
`camera.detection`，无需先发送 camera.command。业务总线仍是 topic + JSON 两帧，
公共头、状态、置信度、图像尺寸、`frame_id`、归一化 `yoke` 均沿用 ZMQ 协议。
`pose` 补充标签到相机光学坐标系的变换：位置单位 m，四元数字段
`qx/qy/qz/qw`，相机 +X 向右、+Y 向下、+Z 向前。目标轴采用 MJCF 的
`yoke_apriltag_center` site，`target_frame` 显式标明此坐标系。
**检测仍为仿真真值**（`detector=mujoco_ground_truth`），不执行图像识别或遮挡判断；
有效时 `confidence=1`，失效时 `pose`、`yoke.roll/pitch` 清空。
同时发布 Monitor 使用的 `steering_wheel`：`theta_rad=-roll_input_joint`，
`translation_along_axis_m=-pitch_input_joint-0.085`（rad / m），来自同帧关节真值；
`calibration_id=mujoco_ground_truth`，目标失效时 `valid=false` 并清空物理量。

默认使用 MJCF 的固定 `cockpit_apriltag` 相机，图像尺寸读取其 `resolution`
（当前 1280×800）；未声明尺寸的其他模型回退到 640×480。ROI 和 detection 图像
尺寸同步采用该分辨率。检测位姿、轮盘值、RGB 都来自同一次物理快照，
`sample_mono_us` 在持有物理锁复制快照时记录，避免渲染期间物理推进导致错帧。
该帧采用的命令、command_ref 和超时状态也在采集时固定，渲染期间的新命令作用于后续帧。
需要控制 ROI/跟踪开关时，仍通过显式 `--control-epoch`、`--core-publisher`
等选项授权；命令的 `camera_id` 应为 `cockpit` 或 `--camera-id` 指定的名称。
首次接受命令后按命令控制检测，超时（`--camera-timeout-ms` 默认 200）转为
SEARCHING；图像采集和图像通道继续工作。这些选项不影响臂、手授权。

有桌面且启用主仿真窗口时，默认同时显示置顶、可拖动缩放的
`D436 RGB - Simulation` 浮动窗口。画面保持原始宽高比；关闭窗口或在其中按
Q/Esc 只关闭预览，不停止仿真和图像发送。主窗口与相机视角独立。
`--no-camera-window` 只禁用浮动窗口；`--headless` 禁用两个窗口但仍采集和传图；
`--no-camera` 停止所有相机图像并报告 OFFLINE。

图像接口复用 `nodes/camera` 的格式，不将大图像塞进业务总线：

| 通道 | 接口及默认行为 |
| --- | --- |
| 实时预览 | PUB bind `tcp://127.0.0.1:5561`；三帧 `camera.rgb.cockpit`、元数据 JSON、JPEG。默认启用，640×360 上限、15 fps 上限、质量 80；1280×800 等比例输出 576×360。 |
| Logger 录制 | 指定 `--recording-config config/recording.yaml` 且 mode 为 raw/compressed 时，PUSH connect 5557；发送单帧 Protobuf `aviator.record.v1.CameraPacket`，内含全分辨率 RGB8 与内参。压缩/MCAP 由 Logger 完成。 |

默认读取 system.yaml 同目录 camera.yaml 的 `preview` 段；缺少该文件时使用
上述默认值。`--camera-config` 可指定路径，仿真只读取其 preview 段，采集内参和
分辨率始终来自 MJCF。`--preview-endpoint <地址>` 覆盖绑定地址，`off` 关闭该通道。
预览线程只保留最新待编码帧；录制队列受 recording.yaml 的字节预算约束，
溢出/接收端缺席时计数丢帧，非阻塞发送。退出日志的 `queued-to-zmq` 不是落盘保证。
当前只录 RGB；启用录制时要求匹配源 `record_depth: false`。

检测、预览和录制共享 `publisher_id/session_id/camera_id/frame_id/sample_mono_us`，
所有 RGB 均为左上角原点，窗口画面不叠加到传输图像中。
Monitor 需同时设置 `sources["camera.detection"]` 和 `preview.publisher_id` 为
`simulation`，`preview.camera_id` 为 `cockpit`。一键启动默认不开启 Logger；添加
`--logger` 才启用记录及图像录制通道。脚本已自动生成这些 Monitor 覆盖，
并按参数传入相机/录制配置：

```bash
AVIATOR_BIN="$PWD/build/debug/bin" ./scripts/start_aviator.sh --simulation
# 单独运行，启用两条图像通道：
./build/debug/bin/simulation --config config/system.yaml \
  --camera-config config/camera.yaml --recording-config config/recording.yaml
```

D436 相机固定在双臂底座安装件上方，朝向驾驶盘中位的 AprilTag。RGB 视场
为 90°×65°，另提供 1280×720、87°×58° 的 `realsense_d436_depth`
理想深度视角；当前节点只采集 RGB，采集循环为 30 Hz。相机外壳使用 geom
group 1，在主窗口可见，自身 RGB 渲染中排除，避免不透明 CAD 镜片遮挡光心。
相机内参是官方标称视场推导值，不是实机标定值；未模拟畸变、主动双目噪声
或深度流发布。模型来源、安装坐标和参数见
[RealSense_D436/README.txt](../../models/meshes/RealSense_D436/README.txt)。

## 构建和测试

完整工程默认构建。也可不构建 Pinocchio/IK/Rokae SDK：

```bash
cmake -S . -B build/simulation-node -DAVIATOR_COMMUNICATION_ONLY=ON \
  -DAVIATOR_BUILD_SIMULATION_NODE=ON -DBUILD_TESTING=ON
cmake --build build/simulation-node --parallel 2
ctest --test-dir build/simulation-node --output-on-failure -R simulation
```

需要 libzmq、yaml-cpp、urdfdom、GLFW、EGL、OpenGL 和工程已有 FFmpeg/Protobuf 依赖；MuJoCo 从仓库源码构建。
完整工程的 `control_nodes_*`、`robot_state_machine_process` 和
`managed_gateway_process` 测试均启动 simulation，不启动真实 manipulator。

`AVIATOR_SIMULATION_TEST_EGL=ON` 可启用 EGL 集成测试。若 Python 环境安装了
MuJoCo、NumPy、OpenCV aruco、pyzmq、PyYAML 和 protobuf，还会注册图像流测试。
该测试使用隔离端口校验业务/预览/录制帧身份、JPEG 解码、RGB 方向、AprilTag
投影和 Logger 断开后的持续采集。桌面浮动窗口可另行验证：

```bash
python3 tests/simulation_camera_stream_test.py build/debug/bin/simulation . --gui
```

`--gui` 测试需要 X11、xwininfo；传 `--output-dir /tmp/camera-test` 时保存接收图像
和检测消息，桌面测试还用 ImageMagick 的 import 保存窗口截图。
