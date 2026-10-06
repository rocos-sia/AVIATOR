# rh56ftp_hand — RH56FTP Modbus TCP ZMQ 节点

这个节点与 `aviator_hand` 并列运行，复用 `third_party/RH56FTP/python/pendant/handlink.py` 的
Modbus 寄存器实现，将 RH56FTP 接入 AVIATOR 的两帧 ZMQ 总线：

- 订阅 `hand.command`，支持 `NORMALIZED_POSITION` 和 `GRASP_SETPOINT`；
- 发布 `hand.state`，反馈角度、力、电流、错误码、状态码和温度；
- 不读取、不发布触觉寄存器；
- 控制命令超时、`valid=false` 或进程退出时写入安全张开姿态（拇指侧摆 `500`，其余五路 `1000`）；
- 支持右手单机，也可用 `--left-host` 接入第二台 RH56FTP。

RH56FTP 的自由度顺序是 `[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指侧摆]`，节点会转换为
AVIATOR 的 `[拇指侧摆, 拇指弯曲, 食指, 中指, 无名指, 小指]`。角度寄存器值按 `0..1000`
归一化到 `drive_position_normalized`。

每侧状态同时保留 `angle`、`force`、`current`、`err`、`status_code`、`temp`（以及
`error`、`status_values` 别名），数组均已转换为 AVIATOR 顺序；触觉字段不出现在消息中。

## 启动

先启动 AVIATOR 总线，再启动节点。运行环境需要 `pyzmq`、`pymodbus` 和读取配置所需的 `PyYAML`：

```bash
python3 nodes/rh56ftp_hand/rh56ftp_node.py \
  --right-host 192.168.11.210

# 两台手：.210 为右手，.220 为左手
python3 nodes/rh56ftp_hand/rh56ftp_node.py \
  --right-host 192.168.11.210 --left-host 192.168.11.220

# 只有左手时可关闭右手默认连接
python3 nodes/rh56ftp_hand/rh56ftp_node.py \
  --right-host "" --left-host 192.168.11.210
```

节点默认从总线输出 `tcp://127.0.0.1:5556` 订阅 `hand.command`（`--endpoint`），
向总线输入 `tcp://127.0.0.1:5555` 发布 `hand.state`（`--state-endpoint`）。
手动发布器的方向相反：向 5555 发布命令，从 5556 订阅状态。正常启动不打印连接信息。
单机模式下 `hands.left.valid=false`，
因此 Core 的双手闭合流程需要同时接入 `--left-host`。

节点为每只手建立独立的控制和状态读取连接；状态轮询不会阻塞 ZMQ 命令看门狗。
`--modbus-timeout` 默认 `0.2` 秒，用于限制单次 Modbus 请求等待时间。

默认状态读取和发布周期为 **60 ms**（`--state-hz` 默认 `1000/60`，约 16.67 Hz）。
读取完成后只等待本轮 60 ms 周期剩余的时间，读取超出周期时直接开始下一轮，不补发积压轮次。
每侧反馈使用该侧 Modbus 读取开始时的单调时间，数据和时间戳一起更新；另一侧读取变慢
不会使刚读到的本侧反馈带上整轮的旧时间戳。单侧读取本身超时仍会使该侧反馈过期。
命令批次处理最多占用约 10 ms（当前一次写入完成后让出），避免积压命令长期阻塞状态发布。
60 ms 是目标更新周期，不是实时调度保证，也不是过期阈值：节点 `--feedback-timeout-ms`
与 Core 的 `core_hand.feedback_timeout_ms` 仍默认 **500 ms**。
启动命令若显式带有 `--state-hz 10`，应去掉该参数以使用新默认值。

控制模式启动时，在写入全张开安全姿态之前，节点先将所有已连接手的六路
模式字节地址 **1625～1630 设置为 0（速度力保护模式）**，再写入速度、力设定。
按设备手册 2.6.20，此模式在到达目标角度或实际受力超过设定值时停止。
`NORMALIZED_POSITION` 和 `GRASP_SETPOINT` 最终都下发角度目标，因此必须使用模式 0；
模式 1 按目标力闭环运动，不适用于本节点的角度控制及全张开安全姿态。
启动时显式写回模式 0，也会恢复旧版节点留下的力控模式。
模式按两个字节一组打包为三个 Modbus 寄存器，从 1625 写入。
一键启动加载 `config/rh56ftp_hand.yaml`，其中 `speed` 为整数 `0..1000`、
`force` 为整数 `0..3000`，默认均为 500，统一应用于左右手六路。
配置在启动时读取，修改后重启生效；显式 `--speed`、`--force` 优先于配置。
不传 `--config` 时沿用内置默认值。

```bash
python3 nodes/rh56ftp_hand/rh56ftp_node.py \
  --right-host 192.168.21.210 --left-host 192.168.11.210 \
  --config config/rh56ftp_hand.yaml
```

这些是设备寄存器设定值，不是实时测得的速度或力。配置成功后，控制、超时回落和
退出沿用这组设定，不随每条角度命令重复写入；写角度失败后的下一次尝试会重新配置
该手。速度、力或模式配置失败时不写入新的角度目标，并在后续控制或安全姿态回落时重试。
节点不调用保存 Flash；重启节点会重新写入参数。

## 握紧停滞后停止下发

五个弯曲通道（拇指弯曲、食指、中指、无名指、小指）独立观察握紧停滞，
拇指侧摆不参与。`NORMALIZED_POSITION` 和 `GRASP_SETPOINT` 转换后的目标都适用。
当一个通道满足以下条件时，底层记录触发时的反馈位置，并向该通道发送一次 `-1` 停止指令：

- 上位机持续发送有效的握紧命令；握紧方向的目标变化不重置观察窗口。
- 目标比当前实际位置低超过 10 个刻度，即仍要求继续握紧而且没有到位。
- 至少覆盖完整 5 秒的有效反馈样本中，位置最大值减最小值不超过 10 个刻度。

例如目标为 `0.0`、实际位置停在 `0.42`，满足条件后记录 `0.42` 并发送一次停止，
随后只查询该通道状态，不再写入当前位置或继续握紧目标。目标 `0.0`、`0.2`、`0.42`
均保持停止，目标大于 `0.42` 时按松开方向恢复执行。记录位置不随反馈抖动变化。
其他仍在运动的通道继续执行；角度写入只覆盖变化的通道，不涉及已停止通道。
相同角度指令去重；所有通道目标均不变时，不再进行角度写入。
运动停滞不代表已验证接触或达到力阈值，本判断不依赖力值。

仅目标大于记录的停止位置时立即解除保持，张开不受停滞逻辑限制。
读取失败、反馈过期或采样间隔达到反馈超时阈值会重新累计观察窗口；重复旧样本不能触发保持。
已经锁定的停止状态在暂时缺少反馈时仍保持。
命令中断达到命令超时、`valid=false`、安全姿态或重新连接会清除保持。
无法写入的新目标不会推进指令 ACK 或提交新的保持状态。

可用 `--closing-hold-ms` 调整观察窗口（默认 `5000`，必须为正数），
`--closing-motion-raw` 调整运动范围及目标残差容差（默认 `10`，范围 `0..999`）。
10 个寄存器刻度对应归一化位置 `0.01`，不是关节角度单位。

`hand.state.hands.{side}` 新增以下字段（六路均使用 AVIATOR 顺序）：

| 字段 | 含义 |
|---|---|
| `requested_drive_position_normalized` | 上位机最近成功接受的原始位置目标。 |
| `commanded_drive_position_normalized` | 运动通道为实际下发目标；停止通道为停止时记录的位置，结合 `closing_hold_active` 判断，不表示重复下发该位置。 |
| `closing_hold_active` | 六路保持标志；拇指侧摆恒为 `false`。 |
| `closing_hold_position_normalized` | 六路固定保持位置；未保持为 `null`。 |

实际位置反馈仍来自设备，不用停止位置替换反馈值。相同目标的有效新命令仍正常更新 ACK，
表示节点已接受并应用停止策略，不代表每次都发生寄存器写入；`ack_write_age_ms` 保留距
最近实际角度写入的时间。Core 仍须持续发送命令，命令看门狗继续有效。
停止下发不是电机断电，也不切换到模式 2；设备实际停止行为和电流需真机验证。

## 可选的握持负载调节

设计和参数说明见 [RH56FTP 握持负载调节设计](../../docs/RH56FTP握持负载调节设计.md)。
`config/rh56ftp_hand.yaml` 的 `hold_control` 在启动时加载，仓库配置已设置 `enabled: true`。
未加载此配置且未显式启用时，程序仍保留原有运动行为。
`current_limit` 默认 100（整数 0..2000），`force_limit` 默认 300、`min_force` 默认 50
（整数 0..3000），`temperature_limit` 默认 50（整数 0..100）。四项均使用原始寄存器刻度，
直接与相应反馈比较（受力取绝对值），
不做物理单位换算，也不归一化到 0..1000；力阈值与 `force` 指令同刻度。
设备没有对应的电流设置指令，`current_limit` 是软件对电流反馈的限制。
电流寄存器先按有符号 16 位解码，再以幅值比较 `current_limit`。
例如 `0xFF80`（无符号 65408）表示 -128，比较幅值为 128；负向电流同样参与超限保护。
`hand.state.current` 和日志 `current_raw` 保留解码后的正负号，`current_register_raw` 保留原始
16 位字；调节诊断额外输出 `current_abs_raw`，避免把补码误判为数万刻度电流。
可填写标量或 AVIATOR 顺序的六项数组，左右手共用；必须满足
`min_force < force_limit * recovery_ratio`。旧的 `_ma`、`_g` 及 `temperature_limit_c` 字段名不再接受。
默认值是初始调试参数；进入保持监测后温度反馈达到 50 即锁存故障，不等待调节观察窗口结束。
温度寄存器每一刻度对应 1 ℃；采用原始整数比较，未增加换算。

启用后，稳定停滞还需满足最低接触力；30 秒闭合期限是失败上限，不是必须等待时间。
保持时电流/受力持续超限，会从实测位置向张开方向小步调整一个弯曲通道，
到位后写 `-1` 并等待反馈稳定，再决定是否继续。单步超过 `move_timeout_ms` 仍未到位时，
只要位移没有越过目标或明显朝闭合方向偏移（允许 `release_tolerance_raw` 容差），
也先写 `-1` 停止，记录 WARNING 并进入观察，不因单步未到位立即锁存故障。
观察和持续超限确认结束后，仍需减载时，目标取“上次未到位目标与新鲜实测位置的较大值 + 步长”，
避免反复发送同一个无效的小目标；负载达到恢复阈值后清除未到位重试目标。
每次尝试都计入原来的 `max_steps`、`max_release_raw` 指令范围和 `adjust_timeout_ms` 总预算，
预算耗尽、反向位移/超程、设备故障、过温等仍触发故障。两手合计最多一个通道处于调整/观察阶段。
每只手独立启用保持监测：至少一个弯曲通道已保持，且五个弯曲通道均已保持或到达请求位置
（沿用 `closing_motion_raw` 到位容差）。侧摆位置偏差不阻塞已保持的弯曲通道减载，
否则侧摆受力未到位时可能导致长期监测一直无法启动。仍有弯曲通道执行抓握/张开时，
该手不应用长期阈值，不累计长期超限或握持不足时间。
进入保持后从新样本重新计时；侧摆自身仍需到位才应用长期阈值，只监测、不自动松开。
监测开始/暂停各打印一次提示，并显示侧摆偏差。
握持识别、闭合超时、反馈有效性、设备自身错误码及模式 0 的设备力保护保持有效。
重新开始张开时取消该手正在进行的自动减载。

超温、设备错误、握持力不足或调节预算耗尽会锁存 `hold_control_error`，
请求停止并发布 `hand.state.valid=false`，供 Core 现有反馈保护处理；重连/重启节点后清除。
启动、命令 watchdog、无效命令和退出路径共用安全张开姿态：所有已连接手的拇指侧摆为 `500`，
其余五个弯曲通道为 `1000`，避免侧摆到 `1000` 时与手柄干涉。
AVIATOR 通道顺序为 `[500,1000,1000,1000,1000,1000]`，写入设备时转换为反序。
握持故障锁存也不阻止保护性张开；写入失败时 watchdog 路径继续重试。
握持故障首次触发仍先请求 `-1`，随后 Core 停止命令，watchdog 执行全张开。
保护性张开清除逐指保持/减载状态，但不清除故障锁存，也不代表已经物理到位。
该策略不能保证 `-1` 后电机断流或实际抓握可靠，
`grasp_verified` 仍为 false。调节状态位于各手的 `hold_control` 数组；
终端默认使用简洁中文日志，正常运动和稳定保持不周期打印。保持、减载及故障阶段切换时输出提示。

日志沿用现有 Logger 的时间戳和 INFO/WARNING/ERROR 级别。默认显示左右手、手指名称、
原因和相关测量值；例如 `左手食指 负载过高，开始减载；当前位置=400；减载目标=405…`。
设备故障码会展开成堵转、过温、过流等中文说明。数值均沿用原寄存器刻度。

需要完整上下文时，在手节点启动参数中加入 `--log-format json`，恢复 JSON 事件格式：

- `configuration`：连接时打印生效的模式配置、速度/力指令、超时、六路阈值和设备地址。
  这是配置值，不代表设备寄存器读回结果。
- `hold_control_failed`：打印左右手、通道编号/名称、失败前阶段、实际/请求/松开目标、
  原始电流/受力、温度、设备错误码、当前阈值、超限持续时间、调节步数和命令 ACK 上下文。
- `hold_phase_changed`：每个通道的阶段切换带同样的测量及阈值上下文，记录不被其他通道合并。
- `device_error`：ERROR 级别打印非零设备故障，展开 `stall`、`overtemperature`、`overcurrent`、
  `motor_error`、`communication_error`，并保留未知故障位。自动减载关闭时仍输出。
  同时记录当前位置、最近请求位置、最近成功写入目标和指令有效状态；成功写入目标是主机记录，
  不是设备寄存器读回，且负载和故障寄存器分次读取，不能视为同一瞬间的测量。
- `modbus_write_failed` / `feedback_read_failed`：打印设备地址、操作阶段、耗时、错误次数及调用栈；
  写失败还保留本次和上次目标。重复同类错误按通道/设备限频，输出被抑制的次数。

运动期间跳过长期阈值，不等于跳过反馈有效性检查。手册实际受力量程为 `-4000..4000`，
例如 `4014` 会触发 `invalid load feedback: force_raw=4014 outside [-4000,4000]`，
与 `force_limit=300` 无关。错误日志指出具体越界字段、读数及范围，不裁剪或放宽量程。
一次握持故障会让所有通道进入软件 `failed` 阶段，并不代表所有电机都上报故障；
默认文本只打印首发保护原因及全通道停止提示，逐通道 `failed` 转换仅在 JSON 模式保留。

如果由 `aviator_core` 接管控制，把 `config/system.yaml` 中 `core_hand.publisher_id`
改成 `rh56ftp_hand`（或用 `--publisher-id` 设成与其一致），并保证 Core 与本节点使用同一
主机单调时钟。Core 的完成判据仍只使用角度反馈，不把力或温度直接当作抓握确认。

只读状态检查可加 `--feedback-only`；此模式仍发布完整非触觉状态，但拒绝所有控制命令，
不写入角度、速度、力或模式寄存器。

## 手动控制与排查

上述控制模式节点运行后，在同一台电脑另开终端：

```bash
python3 nodes/aviator_hand/hand_command.py
# 在交互提示中输入 both open
```

收到“已接受”表示节点成功写入目标并确认当前会话；实际反馈是否有效另行显示。
发布器重启后需重启手节点，解除旧 control_epoch 绑定（session 仅为文本启动标记，不参与授权）。

若提示“2 秒未收到有效指令确认”，先检查节点启动参数：
`--endpoint tcp://127.0.0.1:5556 --state-endpoint tcp://127.0.0.1:5555`（均为默认值）。
旧版本端点默认值颠倒，更新后须重启节点；自定义总线时也应遵循 SUB 连输出、PUB 连输入。
再检查节点的 `reject command` 日志（发布者/epoch、时钟、命令过期或 Modbus 写入失败）。


## Core 手反馈或 ACK 过期诊断

默认 `--log-format text`：启动时打印一次配置摘要；正常命令、反馈和稳定保持不周期打印。
异常首次出现立即打印，同一事件/设备且原因相同的重复异常每 10 秒最多提醒一次，附带省略次数；
原因改变、反馈异常状态变化和反馈恢复立即提示。减载动作使用 WARNING，设备/I/O/握持保护故障使用 ERROR。
文本模式不输出周期性 `health` 汇总。

`--log-format json` 用于详细排障，保留测量值、阈值、计时器、命令上下文和异常调用栈。
该模式下同类重复异常每秒最多打印一次；持续异常期间默认每秒输出一次 `health` 汇总，
正常保持不再被视为异常。`--diagnostic-interval-s 0` 可关闭 JSON 异常汇总，仍保留事件日志。
将手节点启动命令末尾加上 `2>&1 | tee rh56ftp-hand.log` 可保留日志。

需要进一步排障时，在 JSON 日志中重点查看：

- `hands.left/right.valid`、`feedback_age_ms`、`invalid_reason`、`last_error`：确定是哪侧反馈失效及原因。
- `read_duration_ms`、`read_in_progress_ms`、`write.duration_ms`：区分读取卡住和写入变慢。
- `reader.phase/thread_alive`、`cycle_start_age_ms/cycle_finish_age_ms`：观察读取线程是否存活、
  正在读取还是等待，以及上轮开始/完成距现在多久。
- `reader.start_gap_ms/idle_gap_ms/wait_overrun_ms`：分别为两轮开始之间的间隔、上轮完成到
  本轮开始的空档，以及超出原定等待截止时间的延迟；同时记录最大值。`current_wait_overrun_ms`
  在巡读尚未恢复时也能显示等待超期。它可能包含 OS 调度、Python GIL 或其他停顿，不能单凭
  此字段确定是操作系统调度问题。
- 各侧 `read_started_us/read_finished_us/last_success_finished_us`、`last_success_finish_age_ms`、
  `read_start_gap_ms/read_idle_gap_ms/current_read_idle_ms`：观察每只手最近一次读请求与更新空档。
- `sample_mono_us/sample_timestamp_offset_ms`：该侧读取开始时间与读取完成时间的差值。
  若 `feedback_age_ms > 500` 而 `last_success_finish_age_ms` 很小、`sample_timestamp_offset_ms`
  很大，说明该侧刚完成一次很慢的读取；若两种年龄都很大，说明更新确实中断。
- `ack_command_age_ms`：最后成功写入命令的**原始采样时间**距现在的年龄；Core 只接纳年龄
  小于 100 ms 的命令 ACK。`ack_write_age_ms` 是距写入完成的年龄，两者差距大说明写入耗时
  或命令排队造成延迟。节点无法观察 Core 是否收到了该 ACK。
- `received/accepted/rejected`、`accepted_sequence`、`bound_session`：判断有没有新命令、是否
  被拒绝以及 ACK 是否推进。`reject command` 额外打印被拒绝命令的序号、会话、epoch、
  命令和 origin 年龄、时钟 ID 及具体拒绝原因。
- `state_sent/state_dropped/max_state_gap_ms`：判断状态发布是否出现本地拥塞或长间隔；发送
  成功表示已交给本地 ZMQ，不代表 Core 已收到。

事件 `feedback_read_failed`、`feedback_status_changed`、`modbus_write_failed/slow`、
`feedback_poll_gap`、`feedback_sample_timestamp_offset`、`feedback_reader_failed`、
`command_processing_slow`、`command_batch_slow`、`command_watchdog_expired`、`safe_pose_failed`
会打印相关侧、阶段、耗时或故障原因。日志字段 `mono_us` 与同机 Core 的单调时间戳可对照。
诊断不会改变命令、反馈超时和安全姿态控制。默认反馈周期及握紧保持行为见上文。

## 仿真替代

`simulation --config config/system.yaml` 在一个进程中提供臂服务与兼容的手部消息/ACK；仿真时不启动本节点或 manipulator。Core 手部控制使用相同的 `core_hand` 配置，不再按 backend 跳过。详见 [simulation](../simulation/README.md)。
