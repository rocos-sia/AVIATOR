# manipulator

真机后端使用 `third_party/xCoreSDK-0.7.1` 中 AR 专用的 `rokae::ArRobot`（七轴），头文件和静态库取自同一 SDK 目录。

连续 CONTROL 已改为最新目标通道；阻抗切换仍按已验证流程停止和恢复状态接收，复用现有 RT 控制器。实际刚度以 `config/robot.yaml` 为准。

Rokae 双臂硬件执行节点。仿真由独立 `simulation` 节点提供，二者共享设备服务与轨迹执行协议。后台执行线程管理设备生命周期；有限动作消费轨迹窗口，连续 Servo 只保留最新轮盘目标并执行 1 ms Ruckig + 双臂 IK；ZMQ 和 JSON 位于非实时域。Rokae SDK 自己的两条周期回调只读写数值状态与指令，保持 SCHED_FIFO 80 和关节阻抗模式，保留实际调度及回调耗时诊断。

```bash
./build/bin/manipulator --config config/system.yaml
```

`config/robot.yaml` 的 `rokae` 段配置 left_ip、right_ip、left_local_ip、right_local_ip、joint_stiffness、following_joint_stiffness；没有 backend 选择，也没有窗口或 --headless 参数。机械臂连接真实设备，需要设备和实时调度权限；手由独立 xCoreSDK_hand 进程提供。

启动保持未使能。Core 先通过可靠服务读取本次 server_session 和关节保持目标，再显式授权/使能。启动标记使用普通文本，不校验会话匹配；重启后仍需重新 authorize，重连不自动使能。初次使能后的当前位置保持允许最多 1 s 的命令接入窗口，进入目标流后使用配置的 50 ms watchdog。没有指令时不会自动向 Home 运动。

显式 `stop` 从上一条关节指令本地减速并保持使能，制动期间持续更新反馈；完成后清除首次命令等待计时，不因静止保位超过 1 s 再次报错。下一段已授权轨迹会重新启用 50 ms watchdog；首次 `enable` 的 1 s 接入期限仍保留。

`emergency_disable` 是独立的高优先级服务操作，参数仅为 `config_id`。
接收后立即锁存、清除待执行运动和待处理普通服务；执行线程在上使能、阻抗切换、
Servo 及普通减速循环的检查点中断，直接请求双臂下使能，不执行普通减速轨迹。
该操作绕过普通请求的 pending 槽和去重缓存容量，重复请求只查询同一个设备级锁存结果。
接收返回 ACCEPTED，双臂下使能确认后返回 COMPLETED，结果为
`{"latched":true,"disabled_confirmed":true,"error":""}`。
下使能失败会尝试另一侧并每约 100 ms 重试；请求期限只限制接收，已锁存动作不因客户端超时取消。
锁存后拒绝 authorize、enable、reset_fault 及运动指令，需重启设备进程才能重新授权。
`describe.capabilities.emergency_disable=true` 表明支持此操作；新版 Core 启用光电联锁时拒绝连接不支持它的设备端。
SDK 运动停止后先请求下电并读回电源状态，再由后续正常清理处理接收线程和轨迹日志。

设备指令采用双臂整体快照。回零/抓取/释放的有限轨迹仍按原协议执行；CONTROL 不发送轨迹缓冲，只产生下一条 Ruckig 规划指令；两个 SDK 回调均取走上一条指令后才推进下一点，延迟不跳点追赶。执行进度反馈用于更新窗口，不能解释为两台控制器硬件同步，也不表示实际位置到位。

SDK 回调有独立命令截止时间，通信或执行线程停顿不能无限维持授权。执行线程在正常取消时从最后下发指令减速；命令失效、反馈失效或执行异常锁存错误并停止双臂。停止路径检查关节限位；设备故障不能完成减速时转为停止 SDK 控制。现场仍需验证实际时序和物理停止效果。

订阅 `arm.command`，发布 `arm.state`（100 Hz）。RH56FTP 手控制由独立 xCoreSDK_hand 进程提供。ArmState 包含每侧原始采样时间、实际关节角/速度、TCP、已接纳命令和执行游标。TCP 使用各自 left_base/right_base；四元数均为 qx/qy/qz/qw。SDK 适配分别设置工具的 trans/rpy 与实时齐次矩阵。双臂在 RT 初始化时、上电与启动运动前调用 `setLoad`，统一设置质量 `0.71 kg`、质心 `(0.02134, -0.02368, 0.11188) m`（工具设置截图中的 `(21.34, -23.68, 111.88) mm`），转动惯量按确认值设为 `(0, 0, 0) kg·m²`。参数定义在 `rokae_sdk.cpp` 的 `RokaeArm` 构造函数中；设置失败中止初始化，成功后输出每臂 `RT load` 日志。一键启动的 `--fake-hand` 仍使用真实双臂，因此同样设置负载。SDK 头文件未明确质心坐标系，本次仅换算单位；真机质心坐标系、TCP/安装标定一致性需要现场验证。

lock/unlock 是软件阶段，不做 TCP 对齐判断。wheel_reference 为指令参考，不能冒充真机传感器。

可靠服务包括 describe、authorize、enable、disable、stop、lock、unlock、reset_fault、set_impedance_profile、get_result。单帧 REQ/REP，长操作先返回 ACCEPTED，客户端使用同一请求身份查询已有状态。服务缓存有界，不重复执行同 ID 请求；同 ID 改内容被拒绝。server_session 仅兼容旧请求，不再用于匹配。请求 UUID 去重、config_id、时钟、期限和显式控制授权仍检查。接口约束见 [协议文档](../../docs/AVIATOR_ZMQ协议格式说明.md)。

`nodes/simulation` 实现同一臂服务以及 RH56FTP 手部消息，启动它即可替代本节点的双臂和双手。同一控制总线只能启动一组设备生产者。Core 算法与手部控制不区分真机、仿真。

## 轮盘初始参考

`robot.yaml` 的 `wheel_initial: {angle: 0.0, displacement: -0.085}` 指定启动参考（rad/m）。角度范围 ±0.87266 rad，位移范围 [-0.170, 0] m；缺省保持旧版 (0, 0)。Rokae 仅初始化软件参考，不驱动机械轮盘归位；simulation 在启动物理线程前设置 roll_input_joint / pitch_input_joint 实体初值。初始化后的 `arm.state.wheel_reference` 与该配置一致，后续由执行轨迹更新；使能、停止、重新授权不会重复重置初值。启动日志打印有效数值。修改后重启 Manipulator 及相连 Core。

`set_impedance_profile` 参数为 `config_id` 和 `profile: default|following`。仅在双臂健康使能、轨迹结束且指令速度为零时执行。FOLLOWING 请求即使刚度相同也会强制执行全流程；default 请求仅在刚度数值相同且不属于握盘状态下的 profile 切换时跳过循环暂停与恢复。实际切换前保持原目标和原刚度，等待双臂全部关节实测速度连续 300 ms 小于 0.02 rad/s；反馈年龄必须小于 100 ms，时间戳停滞不累计停稳时间，反馈中断或速度超限重新计时。最长等待 5 s，期间继续检查取消、Core 心跳及服务期限。停稳后先采样双臂实测角度作为保持终点，在原刚度和实时循环下按 joint_speed（同时受 URDF 上限约束）、joint_acceleration、joint_jerk 平滑消除旧目标差值。双臂回调均采用该终点后，继续发送固定保持目标至少 `robot.yaml` 中 `impedance_hold_duration` 指定的时长（单位 s，默认 0.5，即 500 ms；必须为有限正数），再按暂停、设置、恢复分阶段执行，等待新反馈后完成。过渡和保持期间持续检查取消、Core 心跳、服务期限、反馈时效和 SDK 状态，不通过重置运行中的步长基准跳过连续性检查；该驻留时间从终点被双臂回调采用后开始计时，不包含过渡时间。该操作只减小位置误差力矩，不保证外部接触力或模型力矩偏差已消失。切换依次 stopLoop、stopMove、停止并 join 独立反馈线程、stopReceiveRobotState、setJointImpedance，再 startReceiveRobotState、setControlLoop（useStateDataInLoop=false）、startMove、启动最新反馈线程、startLoop。复用连接和 RT 控制器，不重新上电或初始化 RT。暂停后通过 jointPos 重新读取实际角度作为恢复保持目标；等待期间不改变目标。任何阶段失败均停止双臂。状态新增 `execution.impedance_switching` 和 `execution.impedance_profile`；暂停时消息 valid=false，保留旧反馈时间戳，`status_mono_us` 继续更新。Core 的有效 origin 心跳和服务期限在切换中仍被检查。`Impedance settle begin/complete/timeout` 日志记录等待耗时、各臂连续停稳时长、反馈年龄、最大速度关节及全部关节速度、相对等待起点的最大位置漂移（漂移仅用于诊断，不作额外判据）。停稳判据不代表外力或模型力矩已满足控制器启动力控条件。Core 阻抗服务总期限为 20 s（涵盖停稳、SDK 暂停/恢复及结果返回），反馈切换宽限为 20.5 s；Core 任务总期限仍按原配置执行。暂停路径分别记录 stopLoop、stopMove、joinStateReceiver、stopReceiveRobotState 的开始与完成耗时，便于定位 SDK 阻塞。SDK 同步调用尚未返回时无法由业务检查即时取消；返回后仍检查撤销、心跳及期限，不自动重试启动。

### 持续跟踪滞后诊断

执行器在有指令推进时每秒输出两条 `Motion timing`（每臂一条），均在非 SDK 回调线程格式化。`command_hz` 为共享轨迹的实际推进速度，连续 Servo 按名义 1 ms 采样，因此应接近 1000；轨迹开始、结束或停顿跨越的统计窗口不能据此判断降速。`callback_hz` 为 SDK 原始回调频率，`feedback_hz` 为成功更新双臂共享状态的频率；二者差异可帮助识别锁竞争。`max_target_error_rad/error_joint` 表示当前指令与实测关节的最大偏差及关节编号，`max_actual_speed_rad_s` 为该臂各关节实测速度绝对值的最大值。`rt_max_gap_ms/rt_max_work_us` 为本次使能以来累计最大间隔/耗时，不是本秒最大值；有意暂停不计入回调间隔，切换完成会重新建立速率统计基准。

最新目标执行器每秒输出 `Servo timing`，包含规划步数、平均/最大 Ruckig+IK 耗时和目标输入年龄；它在非 SDK 回调线程输出。

Core 同时每秒输出 `Control timing`：输入接纳频率、输入年龄、请求轮盘角度/位移与已执行指令参考。reference 是指令参考，不是实测轮盘位置。结合两侧日志区分输入延迟、轨迹推进变慢和机械跟踪误差，日志本身不改变运动参数或控制模式。停止时 SDK 回调计数、调度策略与优先级也会输出。

SDK 状态接收与指令回调解耦：每臂只有一个线程调用 updateRobotState/getStateData，持续消费 SDK 内部到达数据并覆盖一份最新状态。回调只读快照，不阻塞等反馈，不再执行“最多清理 32 帧”。SDK 内部传输缓存没有公开删除接口，因此仍由独立线程消费旧包；应用层不排队回放状态。主机接收时间仅在接收新快照时更新，重复回调不会给旧反馈续期，超过 100 ms 或接收异常仍触发故障。该时间是主机消费到最新反馈的时间，不是控制器硬件采样时间。CSV 诊断队列独立于控制通道，仍保留用于留证。

`command_hz` 与 `callback_hz` 仍用于检查实际周期，`feedback_hz` 现在统计回调看到的新快照数（可能低于接收频率）。SDK 回调不因反馈到达频率增加而增加发送次数。移除应用发送提前量后仍需真机验证 SDK 实际回调节拍、CPU 调度和机械跟踪延迟。

### Following 目标位置留证

每次阻抗服务申请都会在 `manipulator.log` 写入 `Impedance request`，包含 profile、轨迹编号、执行 tick、14 轴目标（左臂 J1–J7 后接右臂 J1–J7，单位 rad）及轮盘指令参考。Core 同时记录 `Core impedance target`。真机后端在执行切换前（包括 FOLLOWING 的相同参数申请），记录每臂 `Impedance target snapshot`：期望保持目标、上一帧交给 SDK 的目标、实测关节位置/速度、命令及消费序号、请求刚度。

`Impedance pre-pause hold begin/accepted/complete` 分别记录保持终点及规划过渡时间、双臂
回调采用终点、配置的驻留时间（默认 500 ms）保持完成；完成日志包含最大位置跟踪误差和实测速度，随后
才应出现 `Impedance stage=pause`。回调采用不等于控制器内部滤波已完成或力矩归零。

恢复阻抗时，SDK 在 `startMove` 前再次读取位置并更新保持目标；恢复首个回调再用新鲜实测
位置同步目标及步长基准。首帧遇到反馈锁竞争时直接输出本帧实测位置，保持同步待确认状态，
不能重发旧目标。恢复回调不再因关节速度达到 0.02 rad/s 而否决保持同步（FOLLOWING/default
均适用）；切换前的连续停稳检查仍保留。恢复时仍检查反馈数值有效性、时效及指令连续性，
同步超时或 SDK 故障仍停止双臂。
两臂均确认同步后，设备才返回最终保持目标和轮盘软件参考，并记录
`Impedance resumed hold confirmed target_rad`。
处于握盘状态时，即使 FOLLOWING/default 刚度数值相同，profile 切换仍执行实测位置保持流程。
真机不从柔性抓握姿态拟合轮盘参考。`Servo joint takeover from held joints` 和
`Servo joint takeover complete` 分别记录从实测保持角度开始接管和到达绝对目标；
接管完成前的 wheel_reference 不是实测轮盘位置。

每次使能的首个后端切换申请触发逐帧采集（包括 FOLLOWING 的相同参数申请）：保存最近 256 帧和申请后最多 1 秒的数据（总量上限 4352 帧/臂）。实时回调只向预分配的单生产者/单消费者队列复制数值，不加锁、分配内存或写文件。执行器消费队列；完整停止时才导出 `/tmp/aviator-target-trace-<pid>-<arm-instance>-<serial>.csv`，`Target trace` 日志记录路径、端点、帧数、丢帧数和摘要。FOLLOWING 相同参数切换也触发后端逐帧采集；default 相同参数的无操作服务仅保留服务层目标快照。

CSV 的 `mono_us` 是回调开始的主机单调时钟，`loop_epoch` 区分循环启动批次，`callback` 为回调计数；`target_rad1..7` 是应用回调返回给 SDK 的目标，`measured_rad1..7`、`velocity_rad_s1..7` 是该回调读到的反馈。它不记录 SDK 内部滤波后的 UDP 指令，也不能将主机回调时刻等同于机器人采样时刻。若要定位 SDK/控制器内部差异，还需要相应的发送报文或控制器侧记录。

摘要的 `max_target_change_rad` 比较触发后的目标与申请时保持目标，`max_target_step_rad` 比较相邻回调目标，`max_tracking_error_rad` 比较目标与反馈。`post_frames=0` 表示没有触发后证据，不能解释为没有跳变。丢帧或容量耗尽时证据不完整；异常后保护制动引起的目标变化也必须结合停机日志判读，不能直接归因为原始故障。之前运行未启用该记录，无法补录。

### 最新目标通道验证（2026-10-09）

完整构建通过；针对本次通道的 9 项回归通过：control_nodes_flight、control_nodes_watchdog、robot_state_machine_process、following_hold_process、servo_planner、servo_buffer、motion_protocol、arm_target_recorder、impedance_lifecycle。覆盖目标覆盖、输入超时、旧 ID 不重启、重入、规划失败直接停止双臂及阻抗生命周期。当前配置的离线双臂 1 ms 规划测试为 2128 步，平均约 16 μs；此测量不包含 SDK/硬件执行，不能作为真机时延保证。

全量 83 项初次运行有 11 项失败：6 项旧 AviatorRobot_simple 示例、gateway_sdl（现场接入实际手柄，测试假设键盘）、2 项 Monitor 资源断言、control_nodes_flight_hold 的物理到位断言，以及并行负载下的一项状态机通信超时。后者单独复测通过。flight_hold 新通道指令参考已到 -0.174532 rad，仿真实际约 -0.121511 rad；用旧配置、旧缓冲通道对照得到 -0.121518 rad，同样失败，故未改动刚度/物理模型或放宽断言。其余失败位于此次未修改的模块，未为本任务扩展修复范围。未执行真机运动验证。

## 独立手节点

手控制已迁至 [xCoreSDK_hand](../xCoreSDK_hand/README.md)，本进程只订阅 arm.command、发布 arm.state。
手节点使用独立 SDK 连接及进程，不与本进程共享 SDK 对象或锁。
`--fake-hand` 由一键启动脚本传给手节点，不是 manipulator 参数。
