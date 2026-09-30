# aviator_core

从 `examples/AviatorRobot_simple` 迁移的双臂业务控制节点。保留 Home、接近、软件锁定、MoveWheel、ServoWheel、停止、复位和交互操作。PIN-IK、Pinocchio 碰撞检查和 Ruckig 规划都在本进程；不连接设备、不加载 xCore SDK，也不创建 MuJoCo 窗口。

## 整机状态机入口

新增 `--state-machine`，按 [整机状态机设计](../../docs/AVIATOR机器人状态机设计.md) 使用本地
`boost::sml` 1.2.0。原有交互、`--demo`、`--servo-demo` 和 `aviator_core_servo` 保留原行为，
与状态机入口二选一；不能把旧入口的执行器阶段当作新状态机的整机状态。

```bash
cmake --build build --target aviator_core --parallel 2
./build/bin/aviator_core --config config/system.yaml --state-machine \
  --safety-file /run/aviator/safety.json
```

先启动同配置的 bus 和 manipulator。状态机入口在主线程处理所有事件，阻塞的初始化、抓握、
撤离只占用一个异步任务槽；每个任务有 generation 和截止时间，取消后的回调不能恢复旧状态。
初始化预算 30 秒、抓握 180 秒、撤离 60 秒；不是串口 100 ms 应答时间。初始化仅加载规划配置，
不自动使能、回零或释放；没有已脱离方向盘的证据时不会报告 STANDBY。

终端支持六个操作名称：

```text
ENTER_STANDBY
GRASP_WHEEL
START_CONTROL
EXIT_CONTROL
LEAVE_WHEEL
RESET_ERROR
```

正常流程为 `INIT → INITIALIZING → STANDBY → GRASPING → FOLLOWING → CONTROL → FOLLOWING → RELEASING → STANDBY`。
`GRASP_WHEEL` 复用 Enable/ApproachHandles/LockHandles；`ReleaseHandles` 从当前参考解除软件锁定后，
沿既有接近偏移退至预接近位置，复用 IK、轨迹验证和执行，不回 home。该释放程序须单独授权；
启用 `core_hand` 时，LockHandles 等待真实手节点接收闭合指令；ReleaseHandles 等待实际驱动位置达到张开目标后再撤离。闭合应答不代表物理抓握已验证。

`FOLLOWING` 当前执行适配为保持现有参考；安全监督器只能为这个已批准策略设置
`following_authorized=true`，不能把它当作已实现被动柔顺跟随。退出 CONTROL 立即关闭目标接纳，
调用非阻塞 Stop 让 Servo 从当前参考减速；未稳定期间拒绝重新操控或释放。
保护取消会撤销 RemoteLink 普通轨迹发布，迟到工作线程也不能重新发布；需工作线程退出及设备
停止游标确认后才视作稳定。底层仍独立检查指令 watchdog。

状态机返回 `ACCEPTED`、`COMPLETED`、`BUSY`、`INVALID_STATE` 或 `CAPABILITY_UNAVAILABLE`。
`status` 查看整机和执行器状态，`emergency` 触发本地急停，`quit` 退出。
本地调试目标 `servo <angle_rad> <displacement_m> [v]` 仅在 CONTROL 接受，需在 100 ms 内持续更新；
开始操控后 100 ms 没有首条目标或输入中断会进入 SAFE，不缓存 FOLLOWING 期间的目标。
普通终端手工输入不适合连续 Servo 调试，应由测试程序按周期提供。

### 安全证据与能力边界

`--safety-file` 是独立同机监督器**原子替换、持续更新**的 JSON 快照。字段如下；下面的时间戳和
clock 仅示意，不能直接用静态文件作为运行证据：

```json
{
  "clock_id": "<本机 hostname-boot_id>",
  "sample_mono_us": 123456789,
  "emergency_latched": false,
  "clear_of_wheel": false,
  "following_authorized": false,
  "release_authorized": false,
  "fault_cleared": false,
  "source_authorized": false,
  "input_ready": false
}
```

证据需要与当前启动周期同一时钟，且年龄小于 100 ms。缺失、非法、过期或急停置位都直接进入
EMERGENCY_STOP。`ready` 和 `settled` 由设备反馈/执行进度派生，不能通过此文件强行置真。
故障复位还要求实际设备不再故障、工作线程退出和执行器稳定；本入口不自动替代设备维护复位。

急停时额外持久化 `<safety-file>.emergency` 和 `<safety-file>.brake-request`；后者是供独立安全执行器
读取的幂等请求，不经过普通轨迹队列。重启前先检查锁存，锁存或证据未知时不申请普通运动授权。
普通命令不能删除文件或解除急停。此软件证据不能替代独立硬件锁存、急停电路和手刹供电保障。
**本仓库尚无独立安全监督器及手刹执行硬件适配器**，因此导出 `brake_confirmed=false`，不会把发请求
当作手刹到位。部署前需实现监督器、独立执行/反馈和维护解除规程。

`--fsm-simulation` 仅允许 MuJoCo 后端，提供显式的仿真测试守卫，不读取/持久化硬件安全证据；
不能用于真机。当前部署的 `config/robot.yaml` 为 Rokae，需另用仿真配置副本，勿直接在真机配置上测试。

本轮最小接入的普通事件来源为本地终端，`flight.state.system` 与 `system.state.system` 导出真实 SML
状态、状态码、错误历史、任务代号和手刹请求。Gateway 已接纳 INITIALIZING/RELEASING 字符串。
没有新增 RS422 REQUEST/REPLY、Core ZMQ 服务端或飞控来源适配；`aviator_core_servo` 仍是原独立入口，
不能视作已受此状态机门控。RS422 的 Word/线上状态码迁移留待串口适配时确认。

### 无硬件验证

启用 `BUILD_TESTING` 后，`robot_state_machine` 覆盖转换、守卫、迟到回调、任务超时、故障恢复、
全部非急停状态进入急停及急停无出口。带 pyzmq/PyYAML 的 Python 可执行进程级测试：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python tests/robot_state_machine_process_test.py \
  "$PWD/build/bin/aviator_bus" "$PWD/build/bin/manipulator" \
  "$PWD/build/bin/aviator_core" "$PWD"
```

测试只使用临时 MuJoCo 配置、随机 loopback 端口和模拟安全监督器，不连接真机；日志目录打印为
`/tmp/aviator-fsm-test-*`。覆盖完整抓握/撤离流程、非法状态拒绝、急停请求和 Core 重启锁存。

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

阻塞动作在工作线程执行，过程中可输入 status 或 stop；其他动作需等待 `[ok]`。Servo 超过 robot.yaml 的 servo_timeout 没有新目标则停止并保持。v 是速度倍率 `(0,1]`，不是时间。主线程保持任务监督心跳，机械臂与手部各自的通信线程独占自己的 PUB/SUB，规划线程只操作强类型接口。

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

仅接收已绑定会话、同一时钟域、publisher_id=flight_gateway、source=JOYSTICK 的有限且处于 [-1,1] 的数据。沿用 InputGuard 检查序号与 valid。POSITION_HOLD 输入显式按 input_state.checked_mono_us 和消息接收时刻检查时效，同时保留原始轴事件时间并拒绝时间倒退；没有扩展的普通输入仍按原始采样时间检查。输入时效取 system.yaml 的 origin_timeout_ms（默认 100 ms）。失效后不再更新 Servo 目标，现有 servo_timeout（默认 250 ms）触发减速并保持软件锁定，因此停更到触发停止的上限约为两项超时之和，实际停止还需制动时间。同一会话恢复有效数据后可恢复跟随；网关重启后需重启本入口重新绑定，不自动切换来源，手动指定模式则需更新 UUID。Ctrl+C 停止、调用 UnlockHandles（启用真实手时等待张开）、释放软件锁定并失能。

新入口有效跟随时 flight.state 的 control_source=JOYSTICK，等待或失效时为 NONE。Manipulator 仍验证 Core 的 local.task 心跳；网关输入时效在本入口检查，不宣称已实现端到端 flight.command origin 传播。位置保持输入在设备检查正常时允许静止持杆，详见 [flight_gateway 位置保持与设备检查](../flight_gateway/README.md#位置保持与设备检查)。网关与 Core 需一起更新并重启。

`control_nodes_flight_hold` 验证长时间无新轴事件的阶跃跟随、设备查询过期、网关静默和设备断开；`control_nodes_flight` / `control_nodes_flight_auto` 分别验证普通输入的手动指定与自动绑定会话，通过 ZMQ 注入同格式输入，验证完整接近/锁定流程、正负映射、最大速度参数、无效消息不能抢先绑定、其他会话不能替换当前绑定、旧采样重发不能维持运动、输入恢复和 Ctrl+C 退出；使用无头 MuJoCo，不连接真机或 USB 摇杆。

已废除的 TCP 对齐、grasp.ready、跟踪误差及锁定丢失判据没有迁移。软件锁定只表示操作阶段。限位、IK 失败、碰撞规划、来源/时效、指令连续性、设备故障和取消仍独立生效。Home 完成后保留关节速度停稳检查。MuJoCo 实测轮盘跟踪只作为测试断言，不能反向成为运行准入条件。

协议能力、窗口与时序见 [ZMQ 文档第 19 节](../../docs/AVIATOR_ZMQ协议格式说明.md)。配置入口为根目录 `config/system.yaml`，不是旧示例的 `build/bin/config/aviator.yaml`。安装后默认查找 `share/aviator/config/system.yaml`；开发构建默认使用源码根目录配置，不依赖工作目录。

## Core 控制真实机械手

`aviator_core`（交互、Demo、状态机）与 `aviator_core_servo` 共用 `RemoteLink` 的手部控制。
配置位于 [`config/system.yaml`](../../config/system.yaml) 的 `core_hand`，手节点仍读取
[`config/inspire_hand.yaml`](../../config/inspire_hand.yaml)。两者需连接同一个 Bus，运行在同一主机。
当前源码配置启用真实手控制，双手张开为 `[1,1,1,1,1,1]`，闭合为用户指定的 `[0,0,0,0,0,0]`。
六路顺序为拇指旋转、拇指弯曲、食指、中指、无名指、小指。配置使用 0～1，手节点转换为 CAN 0～1000。

```yaml
core_hand:
  enabled: true
  publisher_id: inspire_hand
  completion_timeout_ms: 5000
  feedback_timeout_ms: 500
  open_tolerance: 0.03
  open:
    left: [1, 1, 1, 1, 1, 1]
    right: [1, 1, 1, 1, 1, 1]
  close:
    left: [0, 0, 0, 0, 0, 0]
    right: [0, 0, 0, 0, 0, 0]
```

`close` 缺失时拒绝 lock，超范围/非六元素目标拒绝加载。未配置 `core_hand` 或 `enabled: false`
保留旧的软件锁定行为。MuJoCo 后端强制关闭真实手发布，保持仿真原有行为。

从仓库根目录，在独立终端依次启动（CAN 接口应已配置并启用）：

```bash
# 终端 1，已有 Bus 可复用
./build/bin/aviator_bus --config config/system.yaml
# 终端 2，不加 --feedback-only
./build/bin/aviator_hand --config config/inspire_hand.yaml
# 终端 3，真机机械臂使用实时调度权限
sudo ./build/bin/manipulator --config config/system.yaml
# 终端 4，分步控制
./build/bin/aviator_core --config config/system.yaml
```

Core 终端逐条输入，等待每条阻塞动作打印 `[ok]`：

```text
enable
approach
lock
status
```

- `enable`：先发送张开目标并等待双手实际位置到位，再使能机械臂，确保接近时手已张开。
- `lock`：发送闭合目标，收到本 Core 会话、当前目标对应的 `accepted_command` 且实际反馈有效后，执行机械臂软件 lock。不会等待闭合位置全为 0，也不宣称已接触或抓稳方向盘。
- `unlock`：持续发送张开目标，等待两手各六路实际位置距目标不超过 `open_tolerance`，再执行软件 unlock；超时抛错，不继续状态机撤离。
- `stop`：只停止机械臂运动，保持当前手目标；等运动停止后可输入 `unlock`、`disable`、`quit`。普通 `unlock` 不带机械臂撤离。
- `disable`：若软件仍锁定，先解锁张开，再失能。正常退出也保持监督心跳直到清理动作结束。

使用摇杆时，前三个节点保持运行，改为启动：

```bash
./build/bin/flight_gateway
./build/bin/aviator_core_servo --config config/system.yaml
```

Servo 入口会自动张开、使能、接近、闭合，然后接收摇杆输入。两个 Core 入口只能运行一个。
状态机模式中 `GRASP_WHEEL`/`LEAVE_WHEEL` 自动复用上述开合；它仍要求本说明前述外部安全监督器，
本次没有新增其摇杆/RS422 适配。

Core 的独立手部通信线程 `HandLink` 以 50 Hz 发布 `hand.command`，并独立接收 `hand.state`。
它拥有单独的 PUB/SUB socket、目标/反馈锁和条件变量，不使用机械臂 IO 线程的锁。`lock/unlock`
只提交完整的双手目标快照；目标版本防止旧等待被新目标的应答完成，需要确认的操作在释放机械臂锁后等待。
`flight.state.hand_control.target_version` 可用于诊断当前目标版本。目标长期不变仍持续发送，
包括规划、等待及普通 Stop 阶段；这是普通 Linux 周期线程，不承诺硬实时调度。

origin 保留主线程的真实监督心跳。发送线程在取得手部锁后先读取心跳快照，再读取当前单调时间，
避免“先读取旧 now、再读取更新后的 heartbeat”造成误判。真正超时会打印 `age_us` 和 `limit_us`，
时钟顺序异常单独报告 `ahead_us`，发送线程不会自行刷新主线程心跳。
主线程心跳超过 100 ms、状态机撤销授权或 Core 退出后，不再续发旧目标，手节点按自身
`timeout_ms`（默认 100 ms）回到 `safe_pose`（默认全张开）。因此 SAFE/ERROR/急停不会保证继续抓握；
软件锁定状态也不表示手仍闭合。这里沿用手节点既有超时策略，没有实现物理制动。

实际手反馈超时、只读模式、被其他会话占用或节点重启会终止相关操作并阻断普通机械臂轨迹发布。
通信故障会锁存手部错误，排障后重启 Core 和手节点；节点重启后不自动切换反馈会话。
不要同时运行 `hand_command.py` 或旧 `inspire_hand_node`；手节点绑定首个有效发布会话，重启 Core 后也需
重启手节点重新绑定。`flight.state.hands` 现在来自真实手节点，`freshness.hand` 报告实际反馈时效，
`hand_control.error` 报告控制错误；`grasp_verified` 仍由手节点保持 false。

离线验证（模拟设备和 SocketCAN 写入，不驱动硬件）：

```bash
cmake --build build --target aviator_core aviator_core_servo aviator_core_hand_test aviator_core_hand_link_driver aviator_hand_test -j2
ctest --test-dir build -R '^(core_hand|aviator_hand_controller|robot_state_machine|motion_protocol)$' --output-on-failure
# Python 需安装 pyzmq；亦可使用已有 apriltag_realsense 环境
python tests/core_hand_process_test.py "$PWD/build/tests/aviator_core_hand_link_driver" "$PWD/build/bin/aviator_bus"
```

手部线程回归还覆盖：人为占用机械臂 IO 锁 400 ms 时，手部发送和反馈保持连续；停止 Core 心跳后仍停止
续发旧目标并报告实际超时时间；目标替换、撤销授权和线程退出能结束相应的等待；机械臂 IO 异常会停止
手目标续发，构造失败能清理已启动的线程。测试使用随机本机端口
和模拟设备，不打开 CAN，也不连接实际机械臂。
