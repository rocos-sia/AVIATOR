# rh56ftp_hand — RH56FTP Modbus TCP ZMQ 节点

这个节点与 `aviator_hand` 并列运行，复用 `third_party/RH56FTP/python/pendant/handlink.py` 的
Modbus 寄存器实现，将 RH56FTP 接入 AVIATOR 的两帧 ZMQ 总线：

- 订阅 `hand.command`，支持 `NORMALIZED_POSITION` 和 `GRASP_SETPOINT`；
- 发布 `hand.state`，反馈角度、力、电流、错误码、状态码和温度；
- 不读取、不发布触觉寄存器；
- 控制命令超时、`valid=false` 或进程退出时写入六路安全姿态（全张开）；
- 支持右手单机，也可用 `--left-host` 接入第二台 RH56FTP。

RH56FTP 的自由度顺序是 `[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指侧摆]`，节点会转换为
AVIATOR 的 `[拇指侧摆, 拇指弯曲, 食指, 中指, 无名指, 小指]`。角度寄存器值按 `0..1000`
归一化到 `drive_position_normalized`。

每侧状态同时保留 `angle`、`force`、`current`、`err`、`status_code`、`temp`（以及
`error`、`status_values` 别名），数组均已转换为 AVIATOR 顺序；触觉字段不出现在消息中。

## 启动

先启动 AVIATOR 总线，再启动节点。运行环境需要 `pyzmq` 和 `pymodbus`：

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

控制模式启动时，在写入全张开安全姿态之前，节点向所有已连接手的六路寄存器
写入速度设定和力阈值，**两者默认均为 500**。可用 `--speed`（整数 `0..1000`）
和 `--force`（整数 `0..3000`）调整，参数统一应用于左右手的全部六路：

```bash
python3 nodes/rh56ftp_hand/rh56ftp_node.py \
  --right-host 192.168.21.210 --left-host 192.168.11.210 \
  --speed 500 --force 500
```

这些是设备寄存器设定值，不是实时测得的速度或力。配置成功后，控制、超时回落和
退出沿用这组设定，不随每条角度命令重复写入；写角度失败后的下一次尝试会重新配置
该手。速度或力阈值配置失败时不写入新的角度目标，并在后续控制或安全姿态回落时重试。
节点不调用保存 Flash；重启节点会重新写入参数。

## 握紧停滞后保持当前位置

五个弯曲通道（拇指弯曲、食指、中指、无名指、小指）独立观察握紧停滞，
拇指侧摆不参与。`NORMALIZED_POSITION` 和 `GRASP_SETPOINT` 转换后的目标都适用。
当一个通道满足以下条件时，底层把该通道的实际下发目标固定为触发时的反馈位置：

- 上位机持续发送有效命令，该通道的目标刻度连续至少 5 秒不变；序号和时间戳变化不重置窗口。
- 目标比当前实际位置低超过 10 个刻度，即仍要求继续握紧而且没有到位。
- 至少覆盖完整 5 秒的有效反馈样本中，位置最大值减最小值不超过 10 个刻度。

例如目标为 `0.0`、实际位置停在 `0.42`，满足条件后持续下发 `0.42`；
上位机仍发同一目标时保持值不随反馈抖动变化。其他仍在运动的手指继续执行原目标。
运动停滞不代表已验证接触或达到力阈值，本判断不依赖力值。

该通道目标改变（包括张开）会立即解除保持并重新观察，张开不受停滞逻辑限制。
读取失败、反馈过期或采样间隔达到反馈超时阈值会重新累计观察窗口；重复旧样本不能触发保持。
已经锁定的保持值在暂时缺少反馈时仍保持，新的目标会解除它。
命令中断达到命令超时、`valid=false`、安全姿态或重新连接会清除保持。
无法写入的新目标不会推进指令 ACK 或提交新的保持状态。

可用 `--closing-hold-ms` 调整观察窗口（默认 `5000`，必须为正数），
`--closing-motion-raw` 调整运动范围及目标残差容差（默认 `10`，范围 `0..999`）。
10 个寄存器刻度对应归一化位置 `0.01`，不是关节角度单位。

`hand.state.hands.{side}` 新增以下字段（六路均使用 AVIATOR 顺序）：

| 字段 | 含义 |
|---|---|
| `requested_drive_position_normalized` | 上位机最近成功接受的原始位置目标。 |
| `commanded_drive_position_normalized` | 底层实际成功下发的目标，包含保持后的替换值。 |
| `closing_hold_active` | 六路保持标志；拇指侧摆恒为 `false`。 |
| `closing_hold_position_normalized` | 六路固定保持位置；未保持为 `null`。 |

实际位置反馈仍来自设备，不用保持目标替换反馈值。相同目标的有效新命令仍正常更新 ACK，
Core 不需要为了保持而停止发送命令。

如果由 `aviator_core` 接管控制，把 `config/system.yaml` 中 `core_hand.publisher_id`
改成 `rh56ftp_hand`（或用 `--publisher-id` 设成与其一致），并保证 Core 与本节点使用同一
主机单调时钟。Core 的完成判据仍只使用角度反馈，不把力或温度直接当作抓握确认。

只读状态检查可加 `--feedback-only`；此模式仍发布完整非触觉状态，但拒绝所有控制命令，
不写入角度、速度或力阈值。

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

重启节点后只在异常时打印。正常连接、正常命令、正常反馈、恢复和安全姿态执行成功均不打印。
异常事件立即打印；持续异常期间默认每秒补充一条 JSON `health` 详情汇总，正常时不输出汇总。
同一异常每秒最多打印一次，`suppressed` 表示期间省略的重复次数。
`--diagnostic-interval-s 0` 关闭异常详情汇总，仍保留异常事件日志。将原启动命令末尾加上 `2>&1 | tee rh56ftp-hand.log` 可保留日志。

重点查看：

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
