# inspire_hand_can — 因时手 SocketCAN ZMQ 节点

把 Inspire_Robot（因时手）驱动从 ECanVci USB-CAN 移植到 Linux SocketCAN，并做成
AVIATOR 总线上的 `hand.command` 订阅节点：收到命令 → 校验 → 通过 `can0` 驱动右手、`can1` 驱动左手，
同时周期读取双手实际驱动位置并回发 `hand.state`。

以下命令均在仓库根目录执行；节点配置为
[`examples/inspire_hand_can/config/inspire_hand.yaml`](config/inspire_hand.yaml)。
默认映射：右手 `can0 / id=1`，左手 `can1 / id=2`。

## 文件

| 文件 | 作用 |
| --- | --- |
| `can_writer.hpp` | SocketCAN 非阻塞收发（RAII，扩展帧） |
| `inspire_hand.hpp` | 完整移植 `Inspire` + `InspireAction`：位置/速度/力寄存器、五指动作、动作序列/手势 |
| `position_feedback.hpp` | 六路实际位置的非阻塞读请求、响应校验、完整快照和超时管理 |
| `hand_node.cpp` | ZMQ SUB 控制入口、CAN 反馈采集及 `hand.state` 发布 |
| `hand_command.py` | 手动测试发布器，交互控制/定时控制、节点接受确认、退出失效通知 |
| `config/inspire_hand.yaml` | 配置（双手各自 CAN 接口、双手 id、速度/力、安全姿态、端点、支持模式、超时） |
| `CMakeLists.txt` | 独立构建（libzmq + cppzmq + nlohmann/json + yaml-cpp + pthread） |

## 前置：检查两个 CAN 接口

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

CAN 速率必须与设备设置一致，不应固定假设为 500k。本机此前双手反馈验证使用 **1 Mbps**。
若接口未启用且设备配置为 1 Mbps，可执行：

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

已正常运行的接口无需重新配置。接口名通过 `can.right_interface` / `can.left_interface` 修改，
设备 ID 通过 `hand.right_id` / `hand.left_id` 修改。安装 `can-utils` 后可用
`candump can0` / `candump can1` 被动观察 CAN 流量；节点启动本身不会配置接口速率。

## 构建

```bash
cmake -S examples/inspire_hand_can -B examples/inspire_hand_can/build -DCMAKE_BUILD_TYPE=Release
cmake --build examples/inspire_hand_can/build --parallel 2
```

## 启动总线与手节点

```bash
# 终端 1；已有 bus 时直接复用，不重复启动
./build/bin/aviator_bus

# 终端 2：常规控制模式
./examples/inspire_hand_can/build/inspire_hand_node \
  --config examples/inspire_hand_can/config/inspire_hand.yaml
```

通信独立构建的 bus 位于 `build/communication/bin/aviator_bus`。
省略 `--config` 时，手节点读取可执行文件旁的 `config/inspire_hand.yaml`（构建时复制）；
建议像上面一样明确指定源码配置，避免改错文件。配置修改或重新编译后需重启节点。
常规模式启动会设置速度/力并进入控制循环；只检查反馈时使用下文的 `--feedback-only`。

## 用 ZMQ 手动控制

在同一台电脑上运行 AVIATOR bus 和上述手节点（控制时不要加 `--feedback-only`），
另开终端运行（Python 只需 `pyzmq`；此处复用本机已有环境）：

```bash
conda activate apriltag_realsense  # 这台电脑上已安装 pyzmq 的环境
python examples/inspire_hand_can/hand_command.py
```

程序启动后等待输入，不自动发送动作。在同一次运行中依次输入：

```text
both open
right 0.9
right 1 1 0.8 1 1 1
both open
stop
quit
```

`both open` 指定双手张开，`right 0.9` 把右手六路设为 900，下一条仅把右手食指设为 800、
其余五路设为 1000；左手保持前一次目标。数字为驱动刻度归一化值，0 闭合、1 张开，
不是弧度。六路顺序为拇指旋转、拇指弯曲、食指、中指、无名指、小指。
支持 `open`、`half`、`close`、单个数字或六个数字（空格或逗号分隔）。
首次目标及 `stop` 后必须先指定 `both`，避免为另一只手猜测目标。

目标以 50 Hz 连续发送，输入新目标后更新。`stop`、`quit`、Ctrl+C、SIGTERM 发送
`valid=false` 请求节点回落 `hand.safe_pose`（当前配置为双手张开）；退出不是保持最后位置。
断开发布时节点的 100 ms 看门狗同样会回落安全姿态。
状态输出中的“已接受”表示 `hand.state` 已确认当前会话指令；实际位置与反馈有效性另行显示，
不把发送成功当成到位。2 秒没有新鲜有效确认会停止测试并返回错误。

一次性发布指定目标 3 秒后回落安全姿态：

```bash
python examples/inspire_hand_can/hand_command.py --left open --right '1,1,0.8,1,1,1' --duration 3
# 只检查 JSON，不连接总线、不发送动作：
python examples/inspire_hand_can/hand_command.py --pose open --dry-run
```

手节点会绑定首次接受的发布者会话。本程序每次启动生成新 session/epoch，
因此重新启动本程序前，需要停止原发布者并重启手节点；反复调试优先使用交互模式。
不要同时运行其他手控制发布者。`--endpoint` 默认连接总线输入 `tcp://127.0.0.1:5555`，
`--state-endpoint` 默认订阅总线输出 `tcp://127.0.0.1:5556`；自定义配置需对应修改。
同机时钟标识、单调时间、origin、递增 sequence 和双手完整目标由程序自动生成。

## 消息格式（遵循 AVIATOR `hand.command` 草案协议）

订阅 `hand.command`，Frame0 = `hand.command`，Frame1 = JSON。下面是归一化位置模式的结构示例；
时间戳、clock、session、epoch 和 sequence 必须动态生成，不能直接发送此静态示例。
两种模式的主要差别在 `mode` 与 `hands` 字段：

```json
{
  "msg_type": "HandCommand", "version": "1.0", "sequence": 1,
  "timestamp": 1790121600001000, "sample_mono_us": 12345679000,
  "clock_id": "hostA-boot1", "publisher_id": "aviator_core",
  "session_id": "22222222-2222-4222-8222-222222222222", "valid": true,
  "control_epoch": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
  "mode": "NORMALIZED_POSITION",
  "origin": {
    "publisher_id": "flight_gateway",
    "session_id": "11111111-1111-4111-8111-111111111111",
    "sequence": 182736, "sample_mono_us": 12345678000, "clock_id": "hostA-boot1"
  },
  "hands": {
    "left":  {"drive_position_normalized": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
    "right": {"drive_position_normalized": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}
  }
}
```

### 模式

| mode | 输入 | 直通映射 |
| --- | --- | --- |
| `NORMALIZED_POSITION` | `hands.{side}.drive_position_normalized[6]` ∈[0,1] | `raw[i] = round(norm[i] * 1000)` 逐通道直发 |
| `GRASP_SETPOINT` | `hands.{side}.grasp.closure` ∈[0,1] | `raw[i] = round((1 - closure) * 1000)`；0 张开、1 闭合，映射到 6 路 |

> `NORMALIZED_POSITION` 为驱动刻度直通，`GRASP_SETPOINT` 为闭合程度（方向相反），均无插值。六指顺序恒为
> `[拇指旋转, 拇指, 食指, 中指, 无名指, 小指]`；1000 = 张开，0 = 闭合。

## 输入校验、授权与退出

- 有效运动指令必须同时提供左右手完整目标；先校验双手全部值、再准备 CAN 目标，任一侧非法时两侧均不下发。位置数组必须为 6 个有限数值，范围 `[0,1]`；抓握与驱动位置目标不能混用。
- 消息类型错误、字段缺失、负数/浮点/超出 `2^53-1` 的序号、重复 JSON 键、超限报文等均拒绝，不退出进程。session 与 epoch 必须为小写规范 UUID。
- `clock_id` 与 `origin.clock_id` 必须等于本机启动日志中的 clock；源 `sample_mono_us` 与 origin 时间都必须不晚于当前单调时间，且年龄小于 `node.timeout_ms`。这要求发布者与本节点使用同一主机/启动周期的单调时钟，不能仅修改时钟标识来接入另一台机器。
- 首条通过完整校验并成功写入 CAN 的指令锁定发布者、session、epoch、origin 发布者与 session；后续身份变化或非递增序号均拒绝。更换授权需重启节点。此机制是首次合法来源绑定，不是密码学身份认证。
- 只有校验通过且成功完成写入才提交授权、序号与接收时间；越界目标不能刷新看门狗。看门狗同时检查最后有效接收时间、源采样时间和 origin 时间。
- 新鲜且身份匹配的 `valid=false` 指令立即回落安全姿态；这种失效通知允许省略数值目标。超时、SIGINT/SIGTERM 也执行安全姿态。
- CAN 使用非阻塞写入。初始化或运行中的写入异常会停止继续执行目标，逐通道尝试双手全部安全姿态写入；一个通道/总线失败不会阻止另一侧的尝试。运行异常或安全写入失败时返回非零。
- CAN 写入不是双手原子事务：一侧通信失败时，另一侧可能已经收到部分目标。安全写入是尽力操作，成功提交到 SocketCAN 不证明实体手已到位。
- `hand.state.hands.{side}.drive_position_normalized` 现在发布实际反馈；指令回显迁至 `commanded_drive_position_normalized`。反馈有效性与指令有效性分开，详见下节。
- 配置中的速度、力、安全姿态必须是 6 个 `[0,1000]` 整数；超时必须为正数，未知模式在启动前拒绝。

## 实际位置读取与发布

节点启动后即周期读取双手 `ANGLE_ACT`，无需先收到控制命令。CAN 读操作依据厂家 [CAN 增补协议 PRJ-02-TS-U-005](https://en.inspire-robots.com/wp-content/uploads/2025/01/INSPIRE-ROBOTS-The-Dexterous-HandsCAN-Supplemental-ProtocolV0.0.2.pdf)：扩展数据帧、读标志 0、DLC=1、负载 `02`；响应同 ID、DLC=2、小端位置值。寄存器和量程依据 [RH56 手册](https://en.inspire-robots.com/wp-content/uploads/2023/11/INSPIRE-ROBOTS-THE-DEXTEROUS-HAND-RH56-SERIES-USER-MANUAL.pdf)。

| 发布数组顺序 | 反馈寄存器 |
| --- | --- |
| 拇指旋转、拇指、食指、中指、无名指、小指 | 1556、1554、1552、1550、1548、1546 |

每只手最多保留一个未完成读请求，响应后才发送下一路；六路全部收齐后替换快照，避免把旧通道和新通道拼成一组。通过 `SocketCan::try_recv()` 有界排空接收队列，忽略写应答、读请求回显、标准/RTR/错误帧、其他设备/寄存器和越界值。读取不等待回复，不阻塞命令看门狗。单次读取失败会记录错误并重试，不产生额外运动指令。

配置默认每只手完整读取目标频率 10 Hz、单寄存器响应超时 30 ms、反馈有效期 300 ms；见 `feedback` 配置段。实际频率取决于设备响应和主循环调度。

保持既有 ZMQ 两帧格式：Frame0=`hand.state`，Frame1=`HandState` JSON，共享 `common/protocol.cpp` 解码器已做互通验证，Logger 可直接记录。

| 字段 | 语义 |
| --- | --- |
| 顶层 `valid` | 左右手反馈都新鲜有效；与是否正在接收控制命令无关。 |
| 顶层 `command_valid` | 原指令看门狗有效状态。 |
| `hands.{side}.drive_position_raw` | 最近完整快照的六路 0～1000 实测驱动刻度；从未收到时为 null。 |
| `hands.{side}.drive_position_normalized` | 上述刻度除以 1000；从未收到时为空数组。 |
| `hands.{side}.commanded_drive_position_normalized` | 上次成功下发的运动目标回显。 |
| `hands.{side}.valid/feedback_available` | 该侧完整反馈是否在有效期内。过期时保留数值用于显示，但均为 false。 |
| `hands.{side}.sample_mono_us` | 本快照最早读请求的主机单调时间；从未收到时为 null。 |
| `hands.{side}.sample_time_basis` | `host_read_request`；不是设备硬件采样时间。CAN 无事务序号，无法完全区分跨轮次延迟的同寄存器响应。 |
| `hands.{side}.feedback_age_ms` | 完整快照年龄；发布不会刷新原始快照时间。 |
| `hands.{side}.position_source` | `angle_act_register`。 |
| `hands.{side}.joint_position/joint_velocity` | null：尚无刻度到 rad/rad/s 的标定，不把归一化驱动刻度冒充关节角。 |
| `hands.{side}.status` | 反馈完整时 READY/ACTIVE；从未收到为 OFFLINE，过期/失效为 STALE。 |
| `hands.{side}.enabled` | 软件指令有效状态，并非读取到的硬件使能位。 |
| `hands.{side}.feedback_samples/feedback_timeouts/feedback_io_errors` | 完整快照、读响应超时及收发异常计数。 |

硬件故障寄存器尚未读取，`error_code=0` 不代表已诊断硬件无故障；`grasp_verified` 仍为 false。主程序 open-loop `lock/unlock` 及 Core 对 manipulator 来源的授权策略不受此独立手节点改变。

仅检查反馈时可使用只读模式：

```bash
./examples/inspire_hand_can/build/inspire_hand_node \
  --config examples/inspire_hand_can/config/inspire_hand.yaml --feedback-only
```

只读模式不发送速度/力/位置设置，忽略控制命令，退出也不发送安全姿态；仍正常读取并发布位置。常规控制启动命令不变。更新程序后需重启已有进程才能使用新功能。

同一套 CAN 设备只运行一个手节点，常规模式与只读模式二选一。要观察总线上实际位置，在 bus 与手节点运行时另开终端：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python - <<'PY'
import json
import zmq

context = zmq.Context()
sub = context.socket(zmq.SUB)
sub.setsockopt(zmq.SUBSCRIBE, b"hand.state")
sub.connect("tcp://127.0.0.1:5556")
try:
    while True:
        topic, payload = sub.recv_multipart()
        state = json.loads(payload)
        print("feedback_valid=", state["valid"], "command_valid=", state["command_valid"])
        for side in ("left", "right"):
            hand = state["hands"][side]
            print(side, "raw=", hand["drive_position_raw"],
                  "fresh=", hand["feedback_available"], "age_ms=", hand["feedback_age_ms"])
except KeyboardInterrupt:
    pass
finally:
    sub.close(0)
    context.term()
PY
```

约每秒 10 条状态，无命令时也会更新实际位置；Ctrl+C 退出订阅不会控制手。
`drive_position_raw` 是寄存器实测刻度，不是角度；目前 `joint_position` 仍为 null。
若要一并录制，复用 [相机与 Logger 启动流程](../../nodes/camera/README.md#检测与-mcap-录制)：
业务 MCAP 会记录 `hand.state`、`hand.command` 与 `camera.detection`，图像单独保存。

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| 发布器提示 2 秒没有有效确认 | 检查 bus、手节点、5555/5556 端点以及节点拒绝日志。 |
| 提示只读模式不能运动 | 停止带 `--feedback-only` 的节点，再启动常规控制模式。 |
| 发布器重启后指令被拒绝 | 停止原发布者并重启手节点，解除旧 session/epoch 绑定；同一次交互运行可连续调整目标。 |
| 指令因 clock 或时间被拒绝 | 发布器须与手节点在同一主机运行，使用当前启动周期的单调时钟；勿复用静态 JSON。 |
| 能收到状态，但反馈无效/位置为 null | 检查对应 CAN 接口是否 UP、速率、设备 ID、供电连接及反馈超时/收发错误计数。 |
| 停止发布后手张开 | 当前安全姿态为全 1000；发布中断约 100 ms 后触发看门狗，退出也会请求安全姿态。 |
| 另一只手也动作 | 协议每条运动消息包含双手目标；首次 `both` 会设置双手，之后单侧命令保持另一侧上次目标。 |

## 测试

```bash
cmake -S examples/inspire_hand_can -B examples/inspire_hand_can/build
cmake --build examples/inspire_hand_can/build --parallel 2
ctest --test-dir examples/inspire_hand_can/build --output-on-failure
# 用本机带 pyzmq 的环境跑真实 ZMQ + C++ 内存 CAN 互通测试：
HAND_CONTROLLER_TEST="$PWD/examples/inspire_hand_can/build/inspire_hand_test" \
  /home/rocos/miniconda3/envs/apriltag_realsense/bin/python examples/inspire_hand_can/hand_command_test.py
```

Python 测试使用随机本地端口，不连接运行中的总线；捕获发布器实际消息后交给 C++ 控制器校验，
验证双手目标映射、会话连续性、持续发布、单手调整和停止回落。无 pyzmq 时仅跳过 ZMQ 互通测试。

测试包含 `--help` 和 `inspire_hand_controller`。控制器回归直接运行实际解码、守卫、目标映射及异常收尾代码，注入内存 CAN 写器；不打开 SocketCAN、不驱动硬件。覆盖错误字段/序号、双手完整校验、开合方向与寄存器、源/origin 时间及超时、失效通知、身份切换、部分 CAN 写入失败、单侧总线完全失败、正常退出、安全写入失败、非法配置及多帧报文拒绝；还覆盖 CAN 读帧格式、响应过滤、整组快照、缺帧/超时恢复、实际值与指令值分离、反馈状态共享协议解码及只读模式。

2026-09-29 在 can0/id1、can1/id2 上以只读模式实测：4 秒收到 38 条 `hand.state`，左右手各 38 个完整快照，无读取超时/收发错误；旁路监测未观察到 CAN 写寄存器帧。主程序 `lock/unlock` 到因时手的动作集成仍未实现，open-loop 抓握状态只表示软件阶段。
