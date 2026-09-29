# aviator_core

从 `examples/AviatorRobot_simple` 迁移的双臂业务控制节点。保留 Home、接近、软件锁定、MoveWheel、ServoWheel、停止、复位和交互操作。PIN-IK、Pinocchio 碰撞检查和 Ruckig 规划都在本进程；不连接设备、不加载 xCore SDK，也不创建 MuJoCo 窗口。

从 AVIATOR 根目录构建：

```bash
cmake -S . -B build
cmake --build build --target aviator_bus aviator_core aviator_core_servo manipulator -j3
```

三个终端分别启动：

```bash
./build/bin/aviator_bus --config config/system.yaml
./build/bin/manipulator --config config/system.yaml
./build/bin/aviator_core --config config/system.yaml --demo
```

`manipulator --headless` 关闭仿真窗口。Core 的 `--servo-demo` 演示每 20 ms 更新目标；不加 Demo 选项进入交互模式：

```text
enable
approach
lock
wheel 0.1 -0.01 0.5
servo 0.02 -0.002 0.5
status
stop
unlock
disable
reset
quit
```

阻塞动作在工作线程执行，过程中可输入 status 或 stop；其他动作需等待 `[ok]`。Servo 超过 robot.yaml 的 servo_timeout 没有新目标则停止并保持。v 是速度倍率 `(0,1]`，不是时间。主线程保持任务监督心跳，通信线程独占 PUB/SUB，规划线程只操作强类型接口。

`main.cpp` 保留易读的顺序 Demo 和目标数组。`Aviator.cpp` 实现业务与规划；`RemoteLink` 将已检查的轨迹分成有界窗口发布，等待执行进度，不以实际关节/TCP 到位误差判断完成。初次使能从设备返回的实际保持目标开始，后续从上一条已下发目标衔接。仿真与真机共用上述代码。

`ServoPlanner` 用持久的二维 Ruckig 状态在线规划轮盘角度/推拉位移。新目标继承上一段末端的位置、速度、加速度；不是每 20 ms 从静止重新规划。双臂在同一轮盘路径上用 Pinocchio 微分运动学延续接近阶段选定的逆解分支，并抑制冗余 J2 漂移。开始 Servo 时保留上一条关节指令 FK 的微小数值残差（不超过 2 µm / 2 µrad），避免在首个短周期内突然修正残差；不使用实测 TCP 做此处理。关节段采用匹配两端 q/dq/ddq 的五次曲线，逐毫秒检查关节位置限位、有限数值和规划几何。Servo 规划、传输和执行层不再按 joint_speed/joint_acceleration/joint_jerk 拦截。这里的几何检查针对**规划指令**，没有恢复实际 TCP/抓取偏差到位判断。

Servo 预填充至少 80 ms，随后持续追加不可改写的样本，Core 队列上限 250 ms；正常提前量约 80～100 ms（周期大于 20 ms 时可到 130 ms）。ZMQ 每帧最多 51 个原始 1 ms 样本，附带 q/dq/ddq；Manipulator 不再对 Servo 做 2 ms 线性插值，两臂共用执行游标。缓冲耗尽、通信超时、限位或 SDK 故障仍会停止并报错，不能重复旧点伪装成正常执行。RT 回调不做 IK、Ruckig 或 JSON 编解码。

`robot.yaml` 新增 `wheel_angular_acceleration`、`wheel_linear_acceleration`、`wheel_angular_jerk`、`wheel_linear_jerk`。Servo 继续按 `wheel_*_speed` 及这些轮盘加速度/jerk 参数规划；移除关节动态上限检查不代表生成的关节轨迹满足原动态上限。Rokae 后端按 URDF 速度限制的单周期防跳变检查和控制器自身保护仍保留。正常 Stop/输入超时在已提交的短缓冲之后按 Ruckig 速度模式减速至零，再保持最终位置；完成制动需要时间。通信或规划故障走后端独立制动，此时不承诺正常规划的 C2 衔接。C2 指规划曲线，真实机械臂仍受离散采样、RT 调度、阻抗刚度和负载影响。

设备轨迹采用 `local.task`：已显式使能的本地任务，由 Core 主线程持续监督。它与 flight.command 分开授权，不伪造 FLIGHT/JOYSTICK 输入。普通 Demo/交互入口的 flight.state 中 control_source=NONE，task_phase 表达本地任务阶段，缺失手部和视觉反馈报告无效。

## 摇杆实时控制入口

`main_servo.cpp` 生成 `aviator_core_servo`。先 Enable → ApproachHandles → LockHandles，再订阅 `flight.command`，以 `robot.yaml` 的 `servo_period` 更新 ServoWheel。它与原 `aviator_core` 是二选一的控制入口，同一 Manipulator 不要同时启动两个 Core。

先启动 Bus、Manipulator 和 flight_gateway，再直接启动：

```bash
./build/bin/flight_gateway
./build/bin/aviator_core_servo
```

真机的 `manipulator` 需要实时调度权限，可用 `sudo ./build/bin/manipulator --config config/system.yaml` 启动；Core 不需要 sudo。网关自定义总线时，使用 `--publish` / `--subscribe` 与 system.yaml 保持一致。

默认配置路径与原 Core 相同，开发构建读取源码根目录 `config/system.yaml`。省略 `--gateway-session` 时，自动绑定第一条通过来源、时钟、有效性和时效检查的 flight_gateway 消息，日志打印绑定的 session；无效或过期消息不能抢先绑定。也可用 `--config <system.yaml>` 指定配置，或用 `--gateway-session <UUID>` 手动指定会话。自动绑定只进行一次，不因输入过期而解除绑定。

映射直接写在 `main_servo.cpp`：`angle = roll * 0.87266` rad，`displacement = min(pitch, 0) * 0.170` m，`v = 1.0`。负 pitch 拉动，非负 pitch 的目标位移为零；轮盘速度上限由 robot.yaml 的 wheel_*_speed 决定，目标仍经 Servo 规划执行；不再按应用层关节动态上限拦截。

仅接收已绑定会话、同一时钟域、publisher_id=flight_gateway、source=JOYSTICK 的有限且处于 [-1,1] 的数据。沿用 InputGuard 检查序号与 valid。POSITION_HOLD 输入显式按 input_state.checked_mono_us 和消息接收时刻检查时效，同时保留原始轴事件时间并拒绝时间倒退；没有扩展的普通输入仍按原始采样时间检查。输入时效取 system.yaml 的 origin_timeout_ms（默认 100 ms）。失效后不再更新 Servo 目标，现有 servo_timeout（默认 250 ms）触发减速并保持软件锁定，因此停更到触发停止的上限约为两项超时之和，实际停止还需制动时间。同一会话恢复有效数据后可恢复跟随；网关重启后需重启本入口重新绑定，不自动切换来源，手动指定模式则需更新 UUID。Ctrl+C 停止、释放软件锁定并失能。

新入口有效跟随时 flight.state 的 control_source=JOYSTICK，等待或失效时为 NONE。Manipulator 仍验证 Core 的 local.task 心跳；网关输入时效在本入口检查，不宣称已实现端到端 flight.command origin 传播。位置保持输入在设备检查正常时允许静止持杆，详见 [flight_gateway 位置保持与设备检查](../flight_gateway/README.md#位置保持与设备检查)。网关与 Core 需一起更新并重启。

`control_nodes_flight_hold` 验证长时间无新轴事件的阶跃跟随、设备查询过期、网关静默和设备断开；`control_nodes_flight` / `control_nodes_flight_auto` 分别验证普通输入的手动指定与自动绑定会话，通过 ZMQ 注入同格式输入，验证完整接近/锁定流程、正负映射、最大速度参数、无效消息不能抢先绑定、其他会话不能替换当前绑定、旧采样重发不能维持运动、输入恢复和 Ctrl+C 退出；使用无头 MuJoCo，不连接真机或 USB 摇杆。

已废除的 TCP 对齐、grasp.ready、跟踪误差及锁定丢失判据没有迁移。软件锁定只表示操作阶段。限位、IK 失败、碰撞规划、来源/时效、指令连续性、设备故障和取消仍独立生效。Home 完成后保留关节速度停稳检查。MuJoCo 实测轮盘跟踪只作为测试断言，不能反向成为运行准入条件。

协议能力、窗口与时序见 [ZMQ 文档第 19 节](../../docs/AVIATOR_ZMQ协议格式说明.md)。配置入口为根目录 `config/system.yaml`，不是旧示例的 `build/bin/config/aviator.yaml`。安装后默认查找 `share/aviator/config/system.yaml`；开发构建默认使用源码根目录配置，不依赖工作目录。
