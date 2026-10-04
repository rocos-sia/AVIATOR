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

仍发布 `publisher_id=simulation` 的 `camera.detection`。监控仿真相机时将
`sources["camera.detection"]` 设为 `simulation`。默认使用 MJCF 的固定
`cockpit_apriltag` 相机，图像尺寸读取其 `resolution`（当前 1280×800）；
未声明尺寸的其他模型回退到 640×480。ROI 和 detection 图像尺寸同步采用该分辨率。
这是 EGL 图像对应的轮盘真值检测，不做图像识别或遮挡判断。相机命令沿用显式 `--control-epoch`
与 `--core-publisher` / `--origin-publisher` 授权；这些选项不影响臂、手授权。
`--camera-timeout-ms` 缺省为 200；窗口视角与传感器相机独立。

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

需要 libzmq、yaml-cpp、urdfdom、GLFW 和 EGL；MuJoCo 从仓库源码构建。
完整工程的 `control_nodes_*`、`robot_state_machine_process` 和
`managed_gateway_process` 测试均启动 simulation，不启动真实 manipulator。
