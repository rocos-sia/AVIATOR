# rh56ftp_hand — RH56FTP Modbus TCP ZMQ 节点

这个节点与 `aviator_hand` 并列运行，复用 `RH56FTP/python/pendant/handlink.py` 的
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
手动发布器的方向相反：向 5555 发布命令，从 5556 订阅状态。启动日志会显示节点连接方向。
单机模式下 `hands.left.valid=false`，
因此 Core 的双手闭合流程需要同时接入 `--left-host`。

节点为每只手建立独立的控制和状态读取连接；状态轮询不会阻塞 ZMQ 命令看门狗。
`--modbus-timeout` 默认 `0.2` 秒，用于限制单次 Modbus 请求等待时间。

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
发布器重启后需重启手节点，解除旧 session/epoch 绑定。

若提示“2 秒未收到有效指令确认”，先检查节点启动日志应显示
`SUB hand.command=tcp://127.0.0.1:5556; PUB hand.state=tcp://127.0.0.1:5555`。
旧版本端点默认值颠倒，更新后须重启节点；自定义总线时也应遵循 SUB 连输出、PUB 连输入。
再检查节点的 `reject command` 日志（会话、时钟、命令过期或 Modbus 写入失败）。
