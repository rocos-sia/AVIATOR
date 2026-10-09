# manipulator

**当前排查配置（2026-10-09）**：已移除动态阻抗切换禁用开关。FOLLOWING 与默认刚度均为 `[500, 500, 500, 500, 50, 50, 50]`；进入 FOLLOWING 仍执行暂停、设置和恢复全流程，以区分参数变化与切换流程的影响。进入 CONTROL 时若已是默认刚度则不重复暂停。此前 `aviator-start.foxocUjx` 中的左臂异动原因仍待确认，相同参数不代表恢复流程已通过真机验证。

Rokae 双臂硬件执行节点。仿真由独立 `simulation` 节点提供，二者共享设备服务与轨迹执行协议。后台执行线程管理设备生命周期、消费有界轨迹窗口、执行本地插值并采样；ZMQ 和 JSON 位于非实时域。Rokae SDK 自己的两条周期回调只读写数值状态与指令，保持 SCHED_FIFO 80 和关节阻抗模式，保留实际调度及回调耗时诊断。

```bash
./build/bin/manipulator --config config/system.yaml
```

`config/robot.yaml` 的 `rokae` 段配置 left_ip、right_ip、left_local_ip、right_local_ip、joint_stiffness、following_joint_stiffness；没有 backend 选择，也没有窗口或 --headless 参数。本节点只连接真实设备，需要设备和实时调度权限。

启动保持未使能。Core 先通过可靠服务读取本次 server_session 和关节保持目标，再显式授权/使能。启动标记使用普通文本，不校验会话匹配；重启后仍需重新 authorize，重连不自动使能。初次使能后的当前位置保持允许最多 1 s 的命令接入窗口，进入目标流后使用配置的 50 ms watchdog。没有指令时不会自动向 Home 运动。

显式 `stop` 从上一条关节指令本地减速并保持使能，制动期间持续更新反馈；完成后清除首次命令等待计时，不因静止保位超过 1 s 再次报错。下一段已授权轨迹会重新启用 50 ms watchdog；首次 `enable` 的 1 s 接入期限仍保留。

设备指令采用双臂整体快照。2 ms 间隔的轨迹点在本地展开为 1 ms 位置点；两个 SDK 回调均取走上一条指令后才推进下一点，延迟不跳点追赶。执行进度反馈用于更新窗口，不能解释为两台控制器硬件同步，也不表示实际位置到位。

SDK 回调有独立命令截止时间，通信或执行线程停顿不能无限维持授权。执行线程在正常取消时从最后下发指令减速；命令失效、反馈失效或执行异常锁存错误并停止双臂。停止路径检查关节限位；设备故障不能完成减速时转为停止 SDK 控制。现场仍需验证实际时序和物理停止效果。

订阅 `arm.command`，发布 `arm.state`（100 Hz）；不订阅 `hand.command`，不发布 `hand.state`，手部命令与反馈由独立手节点负责。ArmState 包含每侧原始采样时间、实际关节角/速度、TCP、已接纳命令和执行游标。TCP 使用各自 left_base/right_base；四元数均为 qx/qy/qz/qw。SDK 适配分别设置工具的 trans/rpy 与实时齐次矩阵，保留控制器原有标定负载。真机 TCP/安装标定一致性需要现场验证。

lock/unlock 是软件阶段，不做 TCP 对齐判断。wheel_reference 为指令参考，不能冒充真机传感器。

可靠服务包括 describe、authorize、enable、disable、stop、lock、unlock、reset_fault、set_impedance_profile、get_result。单帧 REQ/REP，长操作先返回 ACCEPTED，客户端使用同一请求身份查询已有状态。服务缓存有界，不重复执行同 ID 请求；同 ID 改内容被拒绝。server_session 仅兼容旧请求，不再用于匹配。请求 UUID 去重、config_id、时钟、期限和显式控制授权仍检查。接口约束见 [协议文档](../../docs/AVIATOR_ZMQ协议格式说明.md)。

`nodes/simulation` 实现同一臂服务以及 RH56FTP 手部消息，启动它即可替代本节点与真实手节点。同一控制总线只能启动一组设备生产者。Core 算法与手部控制不区分真机、仿真。

## 轮盘初始参考

`robot.yaml` 的 `wheel_initial: {angle: 0.0, displacement: -0.085}` 指定启动参考（rad/m）。角度范围 ±0.87266 rad，位移范围 [-0.170, 0] m；缺省保持旧版 (0, 0)。Rokae 仅初始化软件参考，不驱动机械轮盘归位；simulation 在启动物理线程前设置 roll_input_joint / pitch_input_joint 实体初值。初始化后的 `arm.state.wheel_reference` 与该配置一致，后续由执行轨迹更新；使能、停止、重新授权不会重复重置初值。启动日志打印有效数值。修改后重启 Manipulator 及相连 Core。

`set_impedance_profile` 参数为 `config_id` 和 `profile: default|following`。仅在双臂健康使能、轨迹结束且指令速度为零时执行。FOLLOWING 请求即使刚度相同也会强制执行全流程；default 请求在数值相同时跳过循环暂停与恢复。实际切换时双臂按暂停、设置、恢复分阶段执行，等待新反馈后完成。切换只停止/恢复运动循环，保留已有状态订阅和回调；清理积压状态后 startMove/startLoop，不重复 startReceiveRobotState/setControlLoop。完整停止/失能仍会关闭状态接收。保持原关节目标，任何阶段失败均停止双臂。状态新增 `execution.impedance_switching` 和 `execution.impedance_profile`；暂停时消息 valid=false，保留旧反馈时间戳，`status_mono_us` 继续更新。Core 的有效 origin 心跳和服务期限在切换中仍被检查。

### 持续跟踪滞后诊断

执行器在有指令推进时每秒输出两条 `Motion timing`（每臂一条），均在非 SDK 回调线程格式化。`command_hz` 为共享轨迹的实际推进速度，连续 Servo 按名义 1 ms 采样，因此应接近 1000；轨迹开始、结束或停顿跨越的统计窗口不能据此判断降速。`callback_hz` 为 SDK 原始回调频率，`feedback_hz` 为成功更新双臂共享状态的频率；二者差异可帮助识别锁竞争。`max_target_error_rad/error_joint` 表示当前指令与实测关节的最大偏差及关节编号，`max_actual_speed_rad_s` 为该臂各关节实测速度绝对值的最大值。`rt_max_gap_ms/rt_max_work_us` 为本次使能以来累计最大间隔/耗时，不是本秒最大值；有意暂停不计入回调间隔，切换完成会重新建立速率统计基准。

Core 同时每秒输出 `Control timing`：输入接纳频率、输入年龄、请求轮盘角度/位移与已执行指令参考。reference 是指令参考，不是实测轮盘位置。结合两侧日志区分输入延迟、轨迹推进变慢和机械跟踪误差，日志本身不改变运动参数或控制模式。停止时 SDK 回调计数、调度策略与优先级也会输出。

2026-10-09 真机日志 `/tmp/aviator-start.zJ5dBNb7` 显示：切换前 callback_hz 约 996，Following 切换后约 1996，恢复默认后约 2996；轨迹 command_hz 仍约 1000，输入年龄约 0–20 ms。该异常发生在反复关闭/开启状态接收的状态驱动循环路径。修正改为保留订阅和回调；真实恢复效果需以新的真机日志验证，预期多次切换后仍约 1000 回调/秒。该记录不证明控制器内部存在特定队列或线程缺陷。

### Following 目标位置留证

每次阻抗服务申请都会在 `manipulator.log` 写入 `Impedance request`，包含 profile、轨迹编号、执行 tick、14 轴目标（左臂 J1–J7 后接右臂 J1–J7，单位 rad）及轮盘指令参考。Core 同时记录 `Core impedance target`。真机后端在执行切换前（包括 FOLLOWING 的相同参数申请），记录每臂 `Impedance target snapshot`：期望保持目标、上一帧交给 SDK 的目标、实测关节位置/速度、命令及消费序号、请求刚度。

每次使能的首个后端切换申请触发逐帧采集（包括 FOLLOWING 的相同参数申请）：保存最近 256 帧和申请后最多 1 秒的数据（总量上限 4352 帧/臂）。实时回调只向预分配的单生产者/单消费者队列复制数值，不加锁、分配内存或写文件。执行器消费队列；完整停止时才导出 `/tmp/aviator-target-trace-<pid>-<arm-instance>-<serial>.csv`，`Target trace` 日志记录路径、端点、帧数、丢帧数和摘要。FOLLOWING 相同参数切换也触发后端逐帧采集；default 相同参数的无操作服务仅保留服务层目标快照。

CSV 的 `mono_us` 是回调开始的主机单调时钟，`loop_epoch` 区分循环启动批次，`callback` 为回调计数；`target_rad1..7` 是应用回调返回给 SDK 的目标，`measured_rad1..7`、`velocity_rad_s1..7` 是该回调读到的反馈。它不记录 SDK 内部滤波后的 UDP 指令，也不能将主机回调时刻等同于机器人采样时刻。若要定位 SDK/控制器内部差异，还需要相应的发送报文或控制器侧记录。

摘要的 `max_target_change_rad` 比较触发后的目标与申请时保持目标，`max_target_step_rad` 比较相邻回调目标，`max_tracking_error_rad` 比较目标与反馈。`post_frames=0` 表示没有触发后证据，不能解释为没有跳变。丢帧或容量耗尽时证据不完整；异常后保护制动引起的目标变化也必须结合停机日志判读，不能直接归因为原始故障。之前运行未启用该记录，无法补录。
