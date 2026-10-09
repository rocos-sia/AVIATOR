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
  `unlock`、`reset_fault`、`set_impedance_profile`、`get_result`。请求/回复的逻辑 target/server_id
  均为 `manipulator`。共享设备服务实现保留请求去重、10 秒期限、轨迹游标、
  50 ms 指令 watchdog、100 ms origin watchdog和本地制动。
- 臂命令使用 `JOINT_TRAJECTORY` / `SYNCHRONIZED_TICKS`，由 Core 的
  `authorize` 请求安装 control_epoch；不再通过 CLI 给机械臂预设授权。
- 手部接受 `NORMALIZED_POSITION` 与 `GRASP_SETPOINT`。六通道顺序为
  拇指旋转、拇指弯曲、食指、中指、无名指、小指；归一化 1=张开，0=闭合。
  与 RH56FTP 相同，首次合法命令绑定发布者/epoch/origin，反馈携带
  `accepted_command`（包含原始采样时间）、`command_valid`、`feedback_only`
  和各侧实际驱动位置。非法消息不刷新授权或 ACK；`valid=false` 或 100 ms
  超时使双手回安全张开目标。六个通道使用与 RH56FTP 默认参数一致的保持判据：
  实际位置与目标的绝对偏差连续十秒大于 10 个 raw 单位，或完整十秒窗口内
  位置最大值减最小值小于 10 个 raw 单位，满足任一条件即冻结目标。
  到位保持后目标变化恢复运动；受阻保持后仅目标越过停止位置反向离开时恢复运动。
  分别回显 requested/commanded/closing_hold。
  手部没有独立 RPC，ACK 通过 `hand.state` 返回。
- 手部位置来自实际积分关节，按 0..1000 量化，不以目标冒充反馈；
  力、电流、温度等硬件寄存器不可用时返回空数组，`grasp_verified=false`。
- `lock` / `unlock` 同时切换抓取 weld；仿真测得轮盘位形放在
  `wheel_measurement`，与 `wheel_reference` 分开。

## 关节柔顺性

双臂共用 `robot.yaml` 中 `rokae.joint_stiffness` 的七轴刚度（Nm/rad），
使用动力学偏置补偿加 `K(q_target-q)` 力矩，允许接触外力造成关节偏移。
MuJoCo 隐式积分的附加阻尼为 `80*sqrt(K/1000)` Nm·s/rad，并保留 MJCF 原有被动阻尼。
默认前四轴为 500、末三轴为 50。Managed FOLLOWING 使用 `rokae.following_joint_stiffness`，进入 CONTROL 或释放前恢复默认刚度；切换同时更新附加阻尼，保持目标不变。配置修改后重启仿真生效。
该控制律用于近似关节阻抗，不是 Rokae 内部控制器的精确复现。
手柄已有局部软接触参数；`lock` 的 weld 行为仍按原流程执行。
`tests/mjcf_grasp_contact_test.py --grip-inset 0.0025` 可在不修改标定目标的情况下，
验证实际手柄间距缩小 5 mm 时的无 weld 接触抓握。
此测试不代表任意间距误差均可适应；缩小 10 mm 的额外检查仍出现右手中指过深穿入。

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

D436 虚拟相机固定在双臂底座安装件上方，朝向驾驶盘中位的 AprilTag。RGB 视场
为 90°×65°，另提供 1280×720、87°×58° 的 `realsense_d436_depth`
理想深度视角；当前节点只采集 RGB，采集循环为 30 Hz。新版 URDF 网格已经包含
相机外观，MJCF 不再重复添加 D436 外壳和支架，只保留光学坐标系和两台虚拟相机。
为避开新版座舱横杆，虚拟机位比旧机位提高 100 mm，并重新瞄准新驾驶盘标签板；
它不代表网格中实体相机的实机外参。33 个 roll/pitch 组合均验证标签可解码、
120 mm 黑边尺寸正确、整块标签无遮挡。兼容旧模型的 group 1 排除逻辑仍保留。
相机内参是官方标称视场推导值，不是实机标定值；未模拟畸变、主动双目噪声
或深度流发布。模型来源、安装坐标和参数见
[MJCF](../../models/mjcf/aviator.xml)；旧外壳资源及来源说明保留在
[RealSense_D436/README.txt](../../models/meshes/RealSense_D436/README.txt)，不再由此场景加载。

## URDF 与 MJCF 同步

[aviator.xml](../../models/mjcf/aviator.xml) 使用当前
[aviator.urdf](../../models/urdf/aviator.urdf) 的机身、外壳、带标签板的主驾驶盘及
第二驾驶盘显示网格、材质和关节范围。第二驾驶盘位于 Y=0.47 m，通过两个 joint equality
跟随主驾驶盘；两个 pitch 范围均为 -0.170～0 m，home 为 -0.085 m。
无质量的 dummy_link 与对应驾驶盘 body 合并，保留同轴 roll/slide 的运动学关系。
自定义初始驾驶盘位置同时写入第二驾驶盘，避免初始 equality 残差。

机身、外壳和第二驾驶盘按 URDF 不参与碰撞。主驾驶盘仍保留仿真专用的凸分解和
软手柄接触代理、抓握 site/weld，不将整件凹网格替换成一个凸包；这些接触代理及
[grasp.json](../../config/grasp.json) 的抓握标定不是由新显示网格自动重算的。
地面移至 Z=-0.73 m，低于新座舱底部，避免截穿新模型。
标签纹理来自 [Apriltag120id0.pdf](../../docs/Apriltag120id0.pdf)，有效黑边为
120×120 mm，含四周各 15 mm 白边的整体尺寸为 150×150 mm。
纹理覆盖在新网格的标签板表面，纸面超出原有约 120 mm 的 CAD 安装板；
标签沿局部 +Z 外移 1 mm，避免放大的白边与周围网格重叠，虚拟相机同步微调瞄准。
不缩放驾驶盘、支架或 URDF 网格。STL 本身不携带纹理。

MuJoCo 单个二进制 STL 限制 200,000 面，而轻量机身有 244,284 面。
[Cessna/mjcf](../../models/meshes/Cessna/mjcf) 内的两个网格由
[split_stl.py](../../scripts/split_stl.py) 无损拆分；拼接面记录与源 STL 逐字节一致，
不再减面，也不覆盖 URDF 使用的模型。更换源机身后须重新生成这两个文件。
[mjcf_urdf_test.py](../../tests/mjcf_urdf_test.py) 检查源网格、拆分完整性、关节范围、
全部 mimic 关系、关键帧、URDF/MJCF 前向运动学和短时仿真稳定性；
具备 Python MuJoCo/NumPy/PyYAML 时注册为 CTest `simulation_urdf_sync`。

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

完整构建的连续 CONTROL 与 Manipulator 共用 `WHEEL_SERVO/LATEST_TARGET` 执行器：只保留最新轮盘目标，按 1 ms 步长运行 Ruckig + 双臂 IK，不预填充轨迹。有限的回零/抓取/释放仍走原轨迹协议。速度、加速度、jerk 和可选碰撞检查从同一 robot.yaml 加载，`servo_prefill_ms` / `servo_lookahead_ms` 不参与新通道。

`AVIATOR_COMMUNICATION_ONLY=ON` 的轻量仿真构建不加载机器人规划库，因此不声明 latest_servo 能力，仅保留原窗口接口。验证最新目标链路应使用完整构建的 simulation。
