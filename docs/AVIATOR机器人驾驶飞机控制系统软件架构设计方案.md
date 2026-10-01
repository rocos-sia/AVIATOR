# AVIATOR 机器人驾驶飞机控制系统软件架构设计方案

文档编号  AVIATOR SAD 001    版本  1.3    日期  2026年10月2日

### 架构决策

系统统一采用 C++17 开发，使用 CMake 维护工程、依赖、构建、测试与安装；系统采用多进程与 ZMQ 统一消息总线。独立 aviator_bus 进程通过 XSUB → zmq::proxy() → XPUB 转发全部连续控制和状态消息。各业务进程通过统一 Topic 与 JSON 协议通信，统一使用 TCP，默认绑定本机回环地址。

AVIATOR Core 负责输入源仲裁、整机状态机、运动目标生成与 FlightState 聚合；Manipulator 负责双臂设备接入及本地安全执行；独立 aviator_hand 进程通过 SocketCAN 接入双手、执行手命令并发布实际驱动位置反馈。实时伺服闭环与 ZMQ 非实时通信域隔离。Monitor、Logger、Plotter 和 Replay 均采用 C++ 实现。当前 aviator_logger 将总线消息记录到数据 MCAP；相机采集进程将无损 PNG 直接记录到独立的图像 MCAP。两份文件共享记录会话 ID，Replay 从 MCAP 提供复现能力。其他原始设备及伺服周期数据的归档仍属后续升级项。

### 适用范围

本方案面向软件开发、控制算法、硬件联调和系统测试人员，规定进程边界、通信契约、数据时效性、部署与验证方法，可作为后续通信 ICD 和详细设计的基础。RS422 线上格式以 [AVIATOR_RS422通信协议规范](AVIATOR_RS422通信协议规范.md) 为准：115200 bit/s、8N1、无流控，下行 14B、上行 65B、20 ms 周期。物理安全动作及规范中的 TBD 项仍须专项冻结；第 09 章的环形缓冲、仲裁策略和阈值为待实现设计，不代表 ICD 已约定或代码已实现。

### 设计基线与评审项

已确定的架构基线包括 C++17、CMake、MCAP 全量数据记录、统一总线、八个主要 Topic、按节点能力配置的 10/30/50/100 Hz 通信频率、Multipart 两帧格式、JSON、latest-value、sequence、timestamp、watchdog 以及实时隔离。本文补充的超时阈值、队列深度、资源预算与验收数值均为台架初始建议，不代表实测结果或已批准的飞行参数。

### 章节导航

| 章节 | 内容 |
| --- | --- |
| 01—03 | 设计目标与总体架构　进程划分　消息总线设计 |
| 04—08 | Topic 定义　JSON 规范　协议示例　FlightState 结构与示例 |
| 09—12 | 线程与实时性（含 9.1 帧映射、9.2 环形缓冲解析、9.3 双路主备、9.4 发送与 9.5 参数）　监测记录回放　资源评估　故障与安全 |
| 13—17 | 启动部署　systemd　目录结构　实施与验收　原则与参考资料 |

## 01 设计目标与总体架构

设计目标是使真实飞控与 USB 飞行摇杆复用同一控制接口，使任意业务环节可被独立监测与记录，并在通信延迟、消息丢失或进程失效时保持明确的控制边界。统一总线简化连接关系，同时也是核心控制通信的单点依赖，必须纳入故障分析。

### 总体数据流

```
真实飞控 ─双路 RS422─ Flight Gateway ─┐
USB摇杆 ─USB─── Joystick Gateway ├─ PUB / SUB ─┐
                               │            │
AVIATOR Core ───────────────────┤            │
                                            ▼
Manipulator  ───────────────────┤    aviator_bus 独立进程
AVIATOR Hand ───────────────────┤    XSUB → proxy → XPUB
Camera  ───────────────────────┘
                                            │
                        ┌───────────────────┘
                        ▼
              Monitor / Logger / Plotter
                        │
              受限 TCP 监控出口（可选）

Manipulator 内部：非实时通信 → 有界快照 → 实时伺服 → 设备
AVIATOR Hand：ZMQ 命令/状态 ↔ SocketCAN 双手，命令与反馈有效性独立
相机采集 → 有界图像副本队列 → CameraPacket :5557 → aviator_logger → MCAP
总线消息 → aviator_logger → 同一 MCAP；设备/伺服原始采集通道待实现
Replay：读取 MCAP，默认连接隔离回放总线，不接入运行中的执行器
```

图中 PUB 均连接总线 XSUB，SUB 均连接 XPUB；同一进程可同时拥有 PUB 与 SUB。原始业务流向保留，连接拓扑统一收敛至总线。执行器之间不建立绕过总线的业务控制链路。

### 逻辑分层

| 层次 | 职责与边界 |
| --- | --- |
| 设备适配 | RS422、USB、机器人驱动和相机 SDK；封装硬件差异与外部协议。 |
| 协议与通信 | 强类型消息模型、JSON Codec、Topic、ZMQ 端点和接收校验。 |
| 业务控制 | 源仲裁、状态机、目标映射、规划、限幅与 FlightState 聚合。 |
| 实时执行 | 本地伺服、轨迹插值、硬件反馈与独立 watchdog。 |
| 观测工具 | 显示、日志、曲线、回放和诊断；不承担伺服调度。 |

部署基线为 Linux 控制计算机。全部交付节点及运行工具统一采用 C++17，使用 CMake 管理；Python 仅可用于离线分析或辅助脚本，不作为运行节点实现或生产运行依赖。编译器、CMake 与第三方库版本在构建清单中锁定。JSON 编解码、传输与控制算法分别抽象，避免算法依赖 ZMQ socket 或 JSON 对象。

## 02 进程划分与控制权

| 进程 | 输入与输出 | 核心职责 |
| --- | --- | --- |
| aviator_bus | PUB 接入 → SUB 分发 | 仅转发消息与订阅，不解析业务、不进行控制仲裁。 |
| flight_gateway | 双路 RS422（或 USB 摇杆） ↔ flight.*；Core 操作服务 | 每路环形缓冲解帧、共享主备仲裁、输入归一化、周期指令幂等处理及 65B 状态回传；USB 用于台架模拟，两个输入模式互斥。 |
| aviator_core | 飞控和设备状态flight.command ↔ 设备命令、flight.state | 唯一业务仲裁者；状态机、规划、联合安全判定和整机状态聚合。 |
| manipulator | arm.command ↔ arm.state；兼容 invalid hand.state 占位 | 管理双臂，负责目标校验、实时插值、设备约束与本地安全；100 Hz 手占位始终 valid=false，不代表真实手反馈。 |
| aviator_hand | hand.command ↔ hand.state；SocketCAN ↔ 双手 | 校验双手完整目标、执行归一化驱动/抓握设定值、命令 watchdog、安全姿态和实际驱动位置采集。 |
| camera | camera.command → camera.detection | 图像采集、方向盘检测、置信度与观测时间输出。 |
| aviator_monitor | 订阅所需 Topic | 整机仪表、频率、数据年龄、告警和连接状态。 |
| aviator_logger | 全量总线 Topic → 数据 MCAP | 独立 C++ 数据记录节点；当前只记录收到的总线 JSON。 |
| aviator_plotter | 订阅或读取日志 | 趋势曲线、目标与反馈对齐、导出分析。 |
| aviator_replay | 日志 → 隔离总线 | 原速、倍速、单步和算法输入回放。 |


## 03 消息总线设计

| 项 | 设计规定 |
| --- | --- |
| 发布入口 | tcp://127.0.0.1:5555；Bus 的 XSUB bind，各业务 PUB connect。 |
| 订阅出口 | tcp://127.0.0.1:5556；Bus 的 XPUB bind，各业务 SUB connect。 |
| 总线实现 | 一对 XSUB/XPUB 与阻塞 proxy 转发循环；context 初始配置 1 个 I/O 线程。[1] |
| 订阅规则 | Frame0 使用 ASCII Topic；ZMQ 为前缀匹配，业务接收后必须再做精确匹配。[2] |
| 消息格式 | 恰好两帧：Frame0=Topic，Frame1=UTF-8 紧凑 JSON；整条接收后才解析。 |
| 生命周期 | 设置选项后 bind/connect；单实例锁；SIGTERM 可经控制线程关闭 context 或使用可控代理退出。 |

### latest-value 的实现

连续控制和状态使用“每个 Topic 仅消费最新有效值”的应用层语义。接收线程持续读取完整消息，在校验通过后按 Topic 与授权生产者更新有界 mailbox；算法定时读取快照。不同 Topic 不共用一个最新值槽，防止高频消息覆盖其他类型。

两帧 Multipart 不启用 ZMQ_CONFLATE，因为该选项不支持 Multipart。HWM 仅限制队列，不能把旧消息自动替换为最新消息，也不能保证最新帧必达。[[3]](#ref-3) 初始可设业务 PUB SNDHWM=8、SUB RCVHWM=8、总线两侧 HWM=64，再依据峰值与故障注入调整；数值按 peer 生效，不是按 Topic 的缓存。

每轮接收设置处理数量或时间预算，达到预算就让出执行机会，下一轮继续排空。过载时统计丢帧、覆盖次数和消息年龄；仅在源序号递增且原始数据未过期时刷新 watchdog。不能通过“刚收到”来把积压旧命令重新变成新命令。

### 启动与重连

PUB/SUB 不提供持久历史或执行确认，新订阅者可能错过启动阶段消息。[[2]](#ref-2) 周期发布、就绪状态与连续有效数据检查共同完成启动同步，不依赖固定 sleep。Bus 重启后允许 ZMQ 重新连接。

### 远程监控出口

[可选] 建议增加可选 telemetry_bridge：本地 SUB 订阅白名单，按需降采样后通过 TCP PUB 提供只读观测。它仍是统一总线的工具订阅者，不形成第二条控制总线。远程端不得接入 XSUB 控制入口；出口使用绑定地址限制、防火墙与 CURVE/ZAP 或已受控隧道，限制客户端数量和带宽。

## 04 Topic 定义与时效预算

表中列出业务主消费端；Monitor、Logger、Plotter 可按权限订阅任意业务 Topic。频率指名义周期发布率，不等于到达期限或硬实时保证。

| Topic 与类型 | Hz / 周期 | 唯一有效生产者 | 业务消费者 |
| --- | --- | --- | --- |
| flight.command<br>FlightCommand | 50 / 20 ms | Flight 或 Joystick Gateway | Core |
| flight.state<br>FlightState | 50 / 20 ms | Core | Flight Gateway；测试端 |
| arm.command<br>ArmCommand | 100 / 10 ms | Core | Arm Controller |
| arm.state<br>ArmState | 100 / 10 ms | Arm Controller | Core |
| hand.command<br>HandCommand | 50 / 20 ms | Core | aviator_hand |
| hand.state<br>HandState | 有效反馈 10 / 100 ms；兼容占位 100 / 10 ms | aviator_hand；Manipulator 仅发布 invalid 占位 | Core |
| camera.command<br>CameraCommand | 30 / 33.3 ms | Core | Camera Detector |
| camera.detection<br>CameraDetection | 30 / 33.3 ms | Camera Detector | Core |

### 消息内容约定

flight.command 表示归一化 roll/pitch 连续目标（RS422 速度限幅需扩展强类型字段和 Schema 后接入，不可丢弃）；arm.command 表示双臂位置或轨迹段目标；当前 hand.command 表示双手六路归一化驱动位置或统一 closure 抓握设定值；camera.command 表示可重复覆盖的检测目标、ROI 与跟踪期望。内部复位、标定、上电、模式切换等动作通过独立服务处理。外部 RS422 周期帧同时携带状态/抓握指令，Gateway 在解帧与主备仲裁后将其分流至操作适配器，避免 latest-value 覆盖动作或每 20 ms 重复派发。

arm.state 包含双臂设备状态与测量快照；aviator_hand 的 hand.state 包含双手实际驱动刻度、归一化位置、命令接纳状态、反馈时效与诊断计数，当前不提供关节角/速度或硬件使能/故障寄存器。Manipulator 另发始终 invalid 的旧形状手占位，仅供未启用真实手链路时表达 OFFLINE/null。camera.detection 包含观测有效性、置信度和方向盘状态；flight.state 是整机对外反馈，不是内部状态的无差别拼接。

| 链路 | 告警 / 超时初始值 | 超时处理 |
| --- | --- | --- |
| flight.command | 60 / 100 ms | Core 撤销飞控目标有效性并触发安全策略。 |
| arm 命令或反馈 | 30 / 50 ms | Manipulator 本地 watchdog 或 Core 判定不可继续控制。 |
| hand.command | 默认 100 ms | aviator_hand 同时检查接收、消息采样和 origin 年龄，超时发送配置的 safe_pose。 |
| hand.state 反馈 | 默认 300 ms | 每侧按反馈快照时间独立失效；Core 不把顶层状态生成时间当作设备采样时间。 |
| Core 手状态链路 | 默认 500 ms | 约束状态接收和命令确认驻留；不能延长设备侧 300 ms 的反馈有效性。 |
| camera.detection | 100 / 200 ms | 视觉失效；是否退出操控由当前模式的视觉依赖决定。 |
| flight.state | 60 / 100 ms | 网关报告机器人状态失效，不以旧状态冒充在线。 |

上述阈值是台架起始配置。最终阈值必须结合最大调度抖动、物理动作风险和容许失控时间确定，并满足“检测时间 + 安全动作完成时间”不超过系统分配的故障处置预算。

扩展 Topic 建议为 system.state（1—10 Hz）、system.diagnostic（1—10 Hz）及 system.event（事件触发）。事件广播仅用于观察；必须可靠执行的请求另行进入可靠服务通道。

## 05 JSON 协议规范

| 公共字段 | 类型 | 语义 |
| --- | --- | --- |
| msg_type / version | string | 固定 PascalCase 类型；version 为 major.minor，例如 1.0。 |
| sequence | integer | 按 publisher_id + session_id + Topic 单调递增，从 1 开始。 |
| timestamp | integer | 本条快照生成时间，Unix UTC 微秒；用于日志关联。 |
| sample_mono_us | integer | 原始采样或输入接纳时刻，Linux CLOCK_MONOTONIC 微秒。 |
| clock_id | string | 主机与启动标识；仅同一时钟域可直接比较单调时间。 |
| publisher_id / session_id | string | 稳定进程标识与每次启动新建的 UUID；区分重启与乱序。 |
| valid | boolean | 业务数据是否有效；无效数据可用于显示但不可用于控制。 |
| source | string | FlightCommand 必填，标识控制源。 |

为兼容通用 JSON 工具，整数约束为 0 至 2^53−1；超过范围前创建新会话，禁止静默溢出。timestamp 名称保留此前接口约定，单位固定为 μs。UTC 时钟校正不影响 watchdog；同机用 sample_mono_us 检查数据年龄，再用本地接收单调时间检查消息是否持续到达。

网关重发同一外部样本时可递增消息 sequence，但必须保留原始 sample_mono_us；设备状态同理。当前 hand.state 是特定例外：顶层 sample_mono_us 表示状态生成时间，每侧 hands.{side}.sample_mono_us 才表示反馈快照最早读请求时间。RS422 没有发送时间戳，sample_mono_us 只能记录该逻辑帧首次完整接收时的本机单调时间，不是飞控采样时间；A/B 迟到副本不能续期，驱动缓存中的驻留时间需另行约束。Core 派生的运动目标还须携带 origin（输入发布者、会话、序号与采样时间），防止 Core 持续发送旧输入而掩盖上游失效。

### 校验与演进规则

接收端依次检查帧数、Topic、长度、UTF-8、JSON 类型、必填字段、版本、来源、会话、序号、时间与数据范围。建议 Payload 上限 64 KiB、嵌套深度 16，并限制数组长度；拒绝重复键、NaN、Infinity、异常未来时间和未支持的主版本。失败消息只计数与限速记录，不更新控制缓存。

minor 版本只能增加可选字段；删除字段、变更类型、单位或语义需要升级 major。未知枚举和未知控制模式拒绝执行；可忽略未知可选字段。Topic 与 msg_type 一一匹配，协议 Schema 由 C++ 节点与离线分析工具共用。

### 数值与坐标

机械臂和可标定为角度的手关节统一使用 rad、rad/s；位置使用 m；TCP 姿态为 qx、qy、qz、qw 单位四元数。tcp_pose.frame_id 指明参考坐标系，安装外参和关节顺序写入带版本的机器人配置。手部若只支持归一化驱动量，应定义独立字段，禁止将驱动刻度标成 rad。指令 roll/pitch 范围为 [−1,1]，滚转映射为 `roll=r_deg/50`，俯仰为 `pitch=(x_mm-85)/85`；反馈沿用同一映射，允许滚转 ±52°（±1.04）、俯仰 −5～175 mm（±18/17，约 ±1.0588235），不得按扩展行程重新归一化或按指令范围截断。正负方向与实际机构及位移基准的对应仍须台架确认。线上两字节字段的字节序、符号和编码缩放仍按 RS422 规范待冻结。

## 06 命令与可靠服务示例

以下为可解析的 JSON 示例，实际发送使用紧凑序列化。Frame0 单独发送 flight.command，Frame1 内容如下。示例中的标识与数值仅用于说明协议。

```json
{
  "msg_type": "FlightCommand", "version": "1.0",
  "sequence": 182736, "timestamp": 1790121600000000,
  "sample_mono_us": 12345678000, "clock_id": "hostA-boot1",
  "publisher_id": "joystick_gateway", "session_id": "joy-session-1",
  "source": "JOYSTICK",
  "valid": true, "control": {"roll": 0.35, "pitch": -0.12}
}
```

arm.command 的完整头部遵循第05章；下例展示业务扩展字段，合并公共头部后发送。position 模式以固定 joint_names 顺序解释数组，左右臂长度必须匹配配置；指令位置还须通过速度、加速度、工作空间和可达性校验。

```json
{
  "mode": "JOINT_POSITION",
  "origin": {"publisher_id": "joystick_gateway",
    "session_id": "joy-session-1", "sequence": 182736,
    "sample_mono_us": 12345678000, "clock_id": "hostA-boot1"},
  "arms": {
    "left":  {"joint_position": [0.1,0.2,0.0,-0.3,0.0,0.2,0.0]},
    "right": {"joint_position": [-0.1,0.2,0.0,-0.3,0.0,0.2,0.0]}
  }
}
```

### 离散可靠命令

后续为 enable、reset_fault、set_source、calibrate 等操作提供独立服务端点。简单单客户端可采用 REQ/REP；多客户端、异步进度或并发请求采用 ROUTER/DEALER。服务仍使用统一消息模型并将请求与结果摘要发布到 system.event，但不宣称 XPUB/XSUB 已具备可靠 RPC。

```
请求：{"request_id":"req-001","operation":"reset_fault",
       "target":"arm_controller","deadline_ms":1000}
响应：{"request_id":"req-001","status":"ACCEPTED"}
结果：{"request_id":"req-001","status":"COMPLETED","error_code":0}
```

ACK 只表示接收或受理，COMPLETED 才表示确认完成；超时表示结果未知。重试必须使用同一 request_id，服务端按请求身份去重并返回已有结果，非幂等动作须持久化执行记录或通过设备状态对账。ACK 本身不保证“恰好执行一次”。

REQ/REP 超时后须恢复请求状态机或重建 socket；ROUTER/DEALER 要实现关联、期限、重试上限、权限和取消语义。可靠服务尚未实现时，相关操作仅通过受控的本地维护流程完成，不以重复广播临时代替。

### 飞控周期指令与 Core 操作服务

本版外部串口只有两个方向的固定周期帧，不再使用 REQUEST/REPLY/CONTROL/STATUS 四类消息。内部保留连续总线与独立操作服务两条路径：

```text
A/B 下行 14B → 各路解帧 → 共享仲裁与计数器检查 → 一条逻辑输入
                                                  ├→ roll/pitch/限幅 → flight.command → Core
                                                  └→ 状态/抓握意图 → 操作适配器 → Core 服务
Core 聚合 flight.state → Gateway 字段映射 → A/B 上行 65B（50 Hz）
原始 RX/TX、仲裁决定、内部请求与结果 → 记录适配器 → Logger → MCAP
```

当前 `flight_gateway` 已通过 DEALER 调用 Core 的 ROUTER 服务，配置 `service` 默认为 `tcp://127.0.0.1:5559`，复用 `common/service.hpp` 的消息信封、会话绑定及 request_id；Core → Manipulator 的 5558 服务保持独立。目前此链路服务于摇杆按钮，Core 授权输入源为 JOYSTICK。RS422 接入还需增加 FLIGHT 源授权、下表操作适配及速度限幅传播，不能只更改配置字符串就视为已支持。新实现继续使用现有 `service` 配置，不另造重复端点。

| ICD 输入 | 建议内部处理（需冻结前置条件与组合语义） |
| --- | --- |
| 状态 0x00 待机 | 目标待机；可复用 `enter_standby`，不隐含松手或强制中断动作。 |
| 状态 0x01 机器人准备 | 新增准备流程语义；不能自动等同于旧 `grasp_wheel`。 |
| 状态 0x02 机器人控制 | 条件满足后调用 `start_control`；目标输入须与本次授权关联。 |
| 状态 0x03 退出控制 | 调用 `exit_control`，由 Core 管理退出过程。 |
| 状态 0x04 自检测 | 新增自检测服务与进度状态；当前六种按钮操作没有对应项。 |
| 状态 0x05 急停/释放 | 优先撤销主动控制；急停与释放如何组合须冻结，不默认松手。需要独立高优先级处置，不排在普通动作之后。 |
| 状态 0xFF 故障/禁止控制 | 禁止主动目标并通知 Core；不是 `reset_error`。 |
| 抓握 0x00 / 脱开 0x01 | 在允许状态下分别映射 `grasp_wheel` / `leave_wheel`；与状态切换指令一起校验优先级，不能把一帧拆成两个相互冲突的无序请求。 |

建议把状态/抓握视为持续意图，适配器保存 `(运行连接, 指令意图代次, state_cmd, grasp_cmd)`、内部 request_id 及 pending/完成/拒绝/未知结果。**意图代次仅在受控接纳的指令组合改变时增加，roll/pitch、速度和帧计数器变化不产生新的动作身份。** 同一意图只派发一次，连续帧、双路副本和切路不重做动作；拒绝或失败后保持结果，不因下一周期内容相同而无限重试。重做需先离开该意图再按受控流程重新进入。启动/恢复时先以当前指令建立观察基准，不能因线上一直保持“控制”就自动启动。

普通操作保留有界待办，不通过 latest-value 覆盖已登记动作；每个新意图在派发前记录，过期或冲突意图显式取消/拒绝，不能恢复后补做。禁止控制、退出与急停类安全意图优先处理；应先阻止新主动目标，再由 Core 决定机械处置。必须冻结在普通动作 pending 时各安全意图的中止规则，现有按钮实现不能直接宣称满足该需求。

服务超时仍表示结果未知。对同一内部事务查询/重试时沿用原 request_id，不生成新的动作身份；是否启用重试由实现配置决定。内部 ACCEPTED/COMPLETED/REJECTED 只用于 Gateway/Core 协调与记录，**不生成额外串口应答帧**。上行指令回绕不等于动作完成，其回绕时点见第 9.1 节建议；执行状态和故障由 65B 上行反馈。

当前摇杆服务与基础服务记录 Topic 已实现；RS422 运行入口、指令适配器、完整原始记录和新增操作仍是 P3 交付。内部 ZMQ 文档第 14.5 节中旧串口 REQUEST/REPLY 映射须后续同步，不能作为本版外部协议；通用内部服务信封继续复用。无线上事务 ID，不能把内部 request_id 的幂等性宣称为跨串口重启的端到端恰好执行一次。

## 07 FlightState 结构与聚合规则

FlightState 由 Core 以 50 Hz 发布，保留 system、arms、hands、vision 四个业务分组及公共头部。其值表示机器人反馈，不能与飞控自身姿态或飞机状态混淆。

| 字段组 | 必需内容 | 约束 |
| --- | --- | --- |
| system | state、current_error_code、last_error_code | 建议同时提供 control_source；错误码为十进制整数。 |
| arms.left / right | joint_position[]、joint_velocity[]、tcp_pose | 两组数组长度相同并匹配机械臂配置。 |
| tcp_pose | frame_id、position{x,y,z}、orientation{qx,qy,qz,qw} | 参考系与外参版本明确，四元数归一化。 |
| hands.left / right | 启用真实手时同 HandState 每侧完整字段；否则为 Manipulator invalid 占位 | 真实手路径直接聚合 aviator_hand 实际驱动反馈；关键字段为 valid、status、sample_mono_us、drive_position_normalized[6]，未标定的 joint_position/joint_velocity 为 null。六路顺序为拇指旋转、拇指、食指、中指、无名指、小指。 |
| hand_control | enabled、error、target_version（启用真实手时） | Core 手控制链路状态；不替代设备反馈或物理抓握验证。 |
| vision | status、confidence、yoke | confidence∈[0,1]；与原始图像采样时间关联。 |
| vision.yoke | detected、roll、pitch | 归一化方向盘状态；反馈允许 roll∈[−1.04,1.04]、pitch∈[−18/17,18/17]；无有效检测时位置为 null。 |
| freshness | arm、hand、camera 的 valid、age_ms、sequence | 建议纳入 v1 基线，避免顶层新时间掩盖旧子状态。 |

FlightState 是内部详细快照，不直接序列化成串口。上行所需的抓握与脱离/锁止/后退机构状态、驾驶盘实际速度、双臂末端速度、操作力、电流及故障等级还需由 Core/设备适配器补齐强类型字段和 Schema；关节数组与视觉置信度数值可留在内部，但当前 65B 帧没有对应数组/数值槽位。软件构建信息由 Gateway 的受控发布元数据提供，具体软件校验算法待冻结。缺少反馈或未知状态的线上表达须按 ICD 补充，不填零冒充正常。

### 运行状态与错误码

运行状态保留 STANDBY（待机）、GRASPING（抓握中）、FOLLOWING（随动）、CONTROL（主动执行飞控目标）、ERROR（故障），建议补充 INIT、SAFE 和 EMERGENCY_STOP。FOLLOWING 表示经授权的柔顺或目标随动，不执行主动飞控操纵量，具体控制律由算法设计规定。

正常转换为 INIT → STANDBY → GRASPING → FOLLOWING → CONTROL；允许的反向转换须显式定义。进入 CONTROL 要求设备就绪、抓握验证通过、输入源授权有效、必要反馈新鲜、标定有效且无阻断故障。安全条件优先于操作请求，故障恢复不得直接返回 CONTROL。

current_error_code 表示当前最高优先级阻断错误，无当前错误为 0；last_error_code 保留最近一次错误，恢复后不自动清零。详细多故障列表放 system.diagnostic。内部错误域建议为 0x1xxx 系统、0x2xxx 左臂、0x3xxx 右臂、0x4xxx 左手、0x5xxx 右手、0x6xxx 视觉、0x7xxx 通信；这些值不直接上线。RS422 使用 0x0000～0x000B 码表，须显式映射；其多故障循环发送和历史规则仍待冻结，不能用内部最高优先级摘要代替完整轮询数据源。

### 数据一致性

Core 在同一聚合周期读取各 Topic 快照并记录各自序号和年龄；10/30/50/100 Hz 数据并非同时采样。重复的视觉帧不伪装成新观测。任何子数据失效时保留最后值用于显示但将对应 valid=false，顶层 valid 按当前模式所需数据计算；零值不能代表未知。手反馈年龄按每侧 sample_mono_us 和 Core 本地接收年龄检查，不由 10 Hz 状态消息的新顶层时间续期。需要严格同步的算法应使用独立时间对齐模块及明确插值规则。

## 08 FlightState 完整 JSON 示例

示例采用双 7 关节机械臂与双 6 关节灵巧手，仅示意数组形状；实际自由度与关节顺序以硬件 ICD 为准。单位及 freshness 语义见前两章。

```json
{
  "msg_type":"FlightState", "version":"1.0", "sequence":10235,
  "timestamp":1790121600000000, "sample_mono_us":12345680000,
  "clock_id":"hostA-boot1", "publisher_id":"aviator_core",
  "session_id":"core-session-1", "valid":true,
  "system": {
    "state":"CONTROL", "control_source":"JOYSTICK",
    "current_error_code":0, "last_error_code":8194
  },
  "arms": {
    "left": {
      "joint_position":[0.12,-0.32,0.54,0.22,-0.11,0.67,0.31],
      "joint_velocity":[0.01,0.02,-0.01,0,0.01,0.03,-0.01],
      "tcp_pose": {
        "frame_id":"robot_base",
        "position":{"x":0.423,"y":0.182,"z":0.631},
        "orientation":{"qx":0,"qy":0,"qz":0.70710678,"qw":0.70710678}
      }
    },
    "right": {
      "joint_position":[-0.12,0.32,-0.54,-0.22,0.11,-0.67,-0.31],
      "joint_velocity":[0.01,0.02,-0.01,0,0.01,0.03,-0.01],
      "tcp_pose": {
        "frame_id":"robot_base",
        "position":{"x":0.423,"y":-0.182,"z":0.631},
        "orientation":{"qx":0,"qy":0,"qz":-0.70710678,"qw":0.70710678}
      }
    }
  },
  "hands": {
      "left":{"valid":true,"status":"ACTIVE","enabled":true,"error_code":0,
        "feedback_available":true,"position_source":"angle_act_register",
        "sample_mono_us":12345675000,"sample_time_basis":"host_read_request",
            "feedback_age_ms":5,"drive_position_raw":[1000,1000,1000,1000,1000,1000],
            "drive_position_normalized":[1,1,1,1,1,1],
            "commanded_drive_position_normalized":[1,1,1,1,1,1],
        "joint_position":null,"joint_velocity":null,"grasp_verified":false,
        "feedback_samples":100,"feedback_timeouts":0,
        "feedback_io_errors":0,"feedback_last_error":""},
      "right":{"valid":true,"status":"ACTIVE","enabled":true,"error_code":0,
         "feedback_available":true,"position_source":"angle_act_register",
         "sample_mono_us":12345675000,"sample_time_basis":"host_read_request",
             "feedback_age_ms":5,"drive_position_raw":[1000,1000,800,1000,1000,1000],
             "drive_position_normalized":[1,1,0.8,1,1,1],
             "commanded_drive_position_normalized":[1,1,0.8,1,1,1],
         "joint_position":null,"joint_velocity":null,"grasp_verified":false,
         "feedback_samples":100,"feedback_timeouts":0,
         "feedback_io_errors":0,"feedback_last_error":""}
  },
    "hand_control":{"enabled":true,"error":"","target_version":3},
  "vision":{"status":"TRACKING","confidence":0.96,
            "yoke":{"detected":true,"roll":0.32,"pitch":-0.15}},
  "freshness": {
    "arm":{"valid":true,"age_ms":4,"sequence":20470},
    "hand":{"valid":true,"age_ms":0,"sequence":20466},
    "camera":{"valid":true,"age_ms":21,"sequence":6141}
  }
}
```

视觉 status 枚举：OFFLINE、INITIALIZING、SEARCHING、TRACKING、LOST、ERROR。检测失效时 detected=false，roll/pitch=null；不能将“未检测到”解释为方向盘回中。

## 09 线程模型与实时性设计

| 进程或域 | 建议线程 | 时序与限制 |
| --- | --- | --- |
| Core | SUB 接收；100 Hz 算法调度；PUB 发送 | 接收解析后写 typed mailbox；调度按最新有效快照生成目标；50/30 Hz 任务使用独立绝对截止时刻。 |
| Arm Controller | SUB 通信；实时伺服；100 Hz 状态发送 | 伺服频率按驱动要求，典型 1 kHz；通信与 RT 之间仅传有界强类型快照。 |
| aviator_hand | 单线程有界轮询 ZMQ 与双路 SocketCAN；10 Hz 状态发送 | 50 Hz 命令由 Core 周期覆盖；每侧按 10 Hz 目标频率依次读取六个 ANGLE_ACT 寄存器，完整快照才替换，命令 watchdog 独立运行。 |
| Camera Detector | 采集；推理；命令与结果通信 | 最新图像优先；推理有界队列；携带图像采集时间而非仅推理完成时间。 |
| Flight Gateway | 一个有界非阻塞事件循环，拥有两路串口、解析器、仲裁器和 ZMQ socket | 轮询两路 RX/TX、定时发布与服务响应；先检查安全期限，再限额处理数据；记录写盘独立，见 9.2～9.4。 |
| 观测工具 | SUB；UI 或磁盘工作线程 | 阻塞 I/O 与业务处理分离，队列有界，慢端不占用控制线程。 |

### 实时与非实时边界

```
SUB → 帧与JSON校验 → typed command mailbox
                                │ 非实时域
------------------------- 有界快照边界 -------------------------
                                ▼ 实时域
       授权和年龄检查 → 插值与限幅 → 伺服 → EtherCAT/设备
                                │
                     typed state snapshot
                                ▼ 非实时域
                   降采样 → JSON → PUB
```

ZMQ PUB、SUB、XSUB、XPUB socket 均由一个固定线程创建、使用并关闭；context 可以在进程内共享。[[2]](#ref-2) 跨线程通过有界 SPSC 队列或经验证的多缓冲快照通信。不得仅交换指针而让写线程覆写读线程正在读取的数据；内存序与缓冲区所有权必须明确。

实时线程中禁止 JSON 编解码、ZMQ 调用、动态内存分配、磁盘日志和无界锁等待。进入实时循环前完成内存预分配、必要的锁页和预热；根据实测设置线程级 SCHED_FIFO 与 CPU 亲和性，避免把整个含通信线程的进程盲目提升为实时调度。

双臂 100 Hz 目标由本地插值器在伺服周期执行，不允许直接阶跃至远端目标。aviator_hand 当前将 50 Hz 归一化目标直接映射为六路 0～1000 驱动刻度，不进行轨迹插值，并独立检查命令、origin 与接收年龄。若没有新有效目标，各执行器执行已验证的有界安全动作，不能无限保持旧速度或持续推进旧轨迹。

记录实际周期、执行耗时、唤醒延迟和 deadline miss。1 kHz 只是目标配置，是否满足最坏情况时间约束必须在目标硬件、驱动与最大干扰负载下验证。

### 9.1 RS422 固定帧与内部数据映射

以 [RS422 规范](AVIATOR_RS422通信协议规范.md) 为字段唯一依据。以下布局偏移均从整帧字节 0 开始；机器人接收下行，发送上行。测试用飞控模拟器则反向使用相同 Codec。

| 方向 | 固定结构与校验 |
| --- | --- |
| 下行 14B | 0～1=`EB 90`；2=帧计数器；3=状态指令；4=抓握；5～8=滚转/俯仰指令；9～12=速度限幅；13=`(-sum(bytes[0:13])) & 0xFF`。无帧尾。 |
| 上行 65B | 0～1=`EB 90`；2=主备；3～12=指令回绕；13～18=机器人/机构状态；19～26=驾驶盘反馈；27～50=机械臂及其他反馈；51～52=相机/检测；53～57=故障；58～61=软件信息；62=`(-sum(bytes[0:62])) & 0xFF`；63～64=`ED 03`。 |

校验含帧头，上行不含帧尾。接收检查从帧头至校验字节之和低 8 位为 0。没有 LEN、TYPE、线上时间戳或序号回显；不能把下行字节 2 当帧长，也不能把上行字节 2 当计数器。Codec 显式按偏移读写，不将接收数组强转为 C++ struct，以免填充、对齐及本机字节序影响线上格式。

两字节字段先由冻结的 `wire_profile` 定义字节序、符号、缩放及无效值。缺少必需定义时可以运行原始帧离线解析测试，但不允许启用实际 FLIGHT 控制或发送伪正常测量。若采用 0.1 个百分点/编码单位，指令除以 1000 转为内部归一化值；这是待冻结解释，不能继承旧版除以 10000。速度限幅需保留来源、单位及有效性，在 Core/Manipulator 中落实；不支持时拒绝相关控制能力，不静默忽略。

运行状态通过显式映射表编码：待机 0x00、准备完成 0x01、抓握状态 0x02、退出状态 0x03、随动 0x04、操控 0x05、故障 0xEE、急停 0xFF；内部 INIT、SAFE、动作过渡阶段的对外表达仍需冻结，不能按内部枚举序号直接转换。当前故障/历史故障映射到 ICD 的 0x0000～0x000B；内部错误明细及链路诊断保留于日志，不占用未定义字节。

**建议的回绕时点（待飞控确认）**：上行字节 3～12 回绕共享仲裁器最近通过完整格式、范围、计数器和主备接纳检查的下行指令原始十字节，不取两路各自最新值；Core 是否受理及动作是否完成另由运行状态/故障表达。同一聚合周期两路使用同一份回绕与反馈快照。启动尚无可回绕指令、测量缺失或 Core 状态过期时的线上表达尚未定义，必须冻结后启用真实回传，不能填零冒充已抓握或正常。

### 9.2 每路环形缓冲与帧解析

#### 所有权、容量与串口读取

A/B 各有独立 UART、文件描述符、接收环形缓冲、解析统计及发送游标；两路字节绝不写入同一个环。建议同一 Gateway 非阻塞事件循环拥有这些对象及共享仲裁器，避免加锁和跨路并发派发；其他线程仅通过有界快照/记录队列交换完整对象，不能持有环内字节指针。

串口设为 raw、115200、8N1，关闭软件/硬件流控及回显/行处理；显式配置并回读确认。采用非阻塞 I/O，`read` 返回实际字节数可能小于请求数；EAGAIN 表示本轮暂无数据，EINTR 重试需受循环预算约束。零字节读取须结合终端配置、挂断/错误事件及期限判断，不等同于收到合法空帧。配置语义见 [termios](https://man7.org/linux/man-pages/man3/termios.3.html)，短读及错误处理见 [read](https://man7.org/linux/man-pages/man2/read.2.html)。

环大小 `N` 取 2 的幂，台架初值 1024 字节；保留一个空槽区分满/空：

```text
head：下一写入位置；tail：下一读取位置；mask = N - 1
used = (head - tail) & mask
free = (tail - head - 1) & mask
peek(k) = data[(tail + k) & mask]     # 0 <= k < used
push(byte): data[head] = byte; head = (head + 1) & mask
pop(n): tail = (tail + n) & mask     # 0 <= n <= used
```

`push` 必须先检查 `free>0`。跨尾部读取可用两段拷贝，也可逐字节 `peek` 拷入固定 `std::array<uint8_t,65>`，只使用前 14/65 字节；不做无界 `memmove`。为每个字节或接收块保留首次 `read` 的单调时间，保证消费部分块后仍能查到候选首尾的原始接收时刻。示例容量按每字节 uint64 时间戳实现需另计约 8 KiB 元数据，不能只算 1 KiB 字节区。

满线速为 `115200/10=11520 B/s`。按调度停顿预算 20 ms、最大帧 65B 估算，`N-1 >= ceil(11520×0.020)+65 = 296B`，选择 1024 提供容量余量；它不意味着允许缓存到约 89 ms 再执行。应用驻留期限仍单独检查。持续满环或读取积压超过预算时，标记本路失效并记录缺口；停止提交该路输入，协调丢弃已失去连续性的缓冲并重新取得备路资格，不静默覆盖旧字节后继续宣称健康。

当前单线程所有权下 head/tail 用普通索引即可。若实测后拆成单生产者/单消费者，须使用经过验证的 C++ 原子索引和 acquire/release 发布顺序；消费者完成拷贝后才释放槽位，跨线程复位必须握手，不由生产者擅自改 tail。环绕与单生产者/单消费者约束参考 [Linux circular buffers](https://www.kernel.org/doc/html/latest/core-api/circular-buffers.html)；不直接将内核屏障宏复制到用户态。

#### 搜索、收齐与重同步

解析器使用三个逻辑步骤 `SEARCH_HEADER → WAIT_FRAME → VERIFY`。模式在构造时固定：机器人 RX 为长度 14、校验偏移 13；模拟飞控 RX 为长度 65、校验偏移 62 且检查帧尾。下列为实现伪代码，`stamp(k)` 返回字节原始入环时间，`copy` 产生环外完整副本：

```text
parse(ring, format, now, budget):
    L = format.length                       # 14 或 65
    while ring.used > 0 and budget > 0:
        budget -= 1                         # 每次必消费、等待或退出
        if ring.peek(0) != 0xEB:
            ring.pop(1); stats.noise += 1; continue
        t0 = ring.stamp(0)
        if ring.used < 2:
            if now - t0 >= half_frame_timeout:
                ring.pop(1); stats.half_timeout += 1; continue
            return                          # 保留末尾 EB，等待下次 90
        if ring.peek(1) != 0x90:
            ring.pop(1); stats.noise += 1; continue
        if ring.used < L:
            if now - t0 >= half_frame_timeout:
                ring.pop(1); stats.half_timeout += 1; continue
            return                          # 不重启候选首字节的计时
        t1 = ring.stamp(L - 1)
        candidate = ring.copy(L)
        if t1 - t0 >= half_frame_timeout or not wire_valid(candidate):
            ring.pop(1); stats.bad_frame += 1; continue
        ring.pop(L)                         # 已确认帧边界，完整消费
        observe_wire_in_window(candidate, port, t0, t1)
        # 先比较可明确关联的短窗原帧副本；冲突锁存，不派发动作
        if now - t0 > max_rx_residence:
            record_stale(candidate); mark_link_suspect(); continue
        if not semantic_valid(candidate):
            record_rejected(candidate); continue
        on_frame(candidate, port, t0, t1)    # 交共享仲裁，尚不执行
```

`wire_valid` 只检查固定帧头、校验和及上行帧尾；`observe_wire_in_window` 仅比较短窗内身份可确认的原帧副本，不刷新健康或授权，冲突后仲裁器禁止业务提交；`semantic_valid` 检查已冻结的枚举、范围与编码。校验失败仅前移一字节，保留可能嵌在损坏候选中的下一合法帧；校验通过但业务非法时消费整帧，不把业务数据重新扫描成命令。噪声末尾的单个 EB 必须保留至下次读取或半帧超时；正常候选内的 EB 90 / ED 03 都是数据，不触发提前分帧。

每轮先处理过期候选再读入新块，读后再次解析；即使无新字节到达也必须运行超时检查。调度以字节/候选数及耗时双重预算轮转 A/B，避免持续噪声饿死备路、发送或 watchdog。若一个非阻塞读返回多帧，逐帧解析和检查指令意图变化，连续轴值可在处理完后只发布最新值；不能仅取最后一帧而漏掉中途退出/禁止控制。

`read` 的时间戳不能看见 USB 转换器或内核中既有积压。入环时看似新鲜仍可能是旧控制，因此还须约束驱动队列、转换器延迟、事件循环最大停顿及发送端旧帧丢弃；未通过端到端延迟验收时，不能仅凭环内时间或递增计数器允许操控。8 位累加校验也无法检出所有多字节错误：下行没有帧尾，损坏后从数据内 EB 90 开始的错位候选可能恰好满足固定长度与累加校验，故不能承诺仅靠搜索帧头必然立即恢复唯一边界。必须继续检查枚举、范围、计数连续性和双路一致性；即使全部通过也不构成完整性保证，残余风险需在链路验收中明确，帧有效不等于动作已完成。

### 9.3 双路冗余主备通信

#### 拓扑与双方必须一致的约定

以下为**本架构建议的可实现方案，待飞控与机器人双方确认后写入 ICD**。当前规范只定义主备枚举，没有冻结复制、计数和角色含义；配置 profile 未确认时禁止自动冗余操控。

```mermaid
flowchart LR
    FC[飞控同一逻辑下行帧] --> A[A 路 UART 与环形解析器]
    FC --> B[B 路 UART 与环形解析器]
    A --> ARB[共享仲裁器：新旧检查、副本比较、主备选择]
    B --> ARB
    ARB --> INTENT[意图适配与连续目标]
    INTENT --> CORE[Core 授权与状态机]
    CORE --> SNAP[同一份上行反馈快照]
    SNAP --> TA[A 路独立编码与发送游标]
    SNAP --> TB[B 路独立编码与发送游标]
```

- 飞控每 20 ms 生成一帧，下行计数器每个新逻辑周期加 1（mod 256）；同帧复制到 A/B，14 个字节完全一致，重复发送不改计数值。若两路独立计数，以下副本比较方案不成立，须先修改双方设计。
- A/B 是固定物理端口标识；主/备是当前角色。初始偏好 A，允许仅 B 合格时以 B 启动通信；能否单路降级操控由 Core 授权策略决定。
- 建议上行字节 2 表示**本发送端口相对当前指定主路的角色**：指定主路发 0x00，另一路发 0x01。角色快照由一个仲裁器生成，切换同时更新两路后续待发帧。故两路上行字节 2 及校验字节可能不同，其余业务快照一致，不能套用旧版双路整帧完全相同的要求。
- `selected_port` 始终有 A/B 值，`active_valid` 单独表示是否有可接纳主路；双路失败/冲突时保留最后指定角色，但撤销主动输入。0x00 仅表示角色，不表示健康或授权；异常以运行状态/故障报告，不能新增“无主”枚举。此解释必须得到飞控确认。
- 两路接收健康、发送健康分别统计；机器人主路选择依据下行，飞控依据两路上行超时及角色观察回传。无计数回显/ACK，不能单靠机器人 write 成功证明飞控收到。飞控上行失联时如何通过剩余下行发禁止控制或停止控制流，须纳入双方故障流程。

#### 每路健康与共享接纳状态

`LinkContext[A/B]` 保存端口代次、最近完整帧、计数器基准、最后递增合法帧时刻、连续好帧数、错误次数、环占用峰值、UART/TX 状态。状态为 `INIT → QUALIFYING → HEALTHY → SUSPECT/FAILED`，恢复进入 QUALIFYING。任意字节到达、重复计数、坏校验或业务非法均不能刷新“递增合法帧”的健康期限；备路收到合法新帧可以保持本路健康，但不自动成为业务输入。

共享 `DualLinkArbiter` 保存 `selected_port`、`active_valid`、运行连接 ID、最后提交的逻辑计数/本地扩展序号、最后提交时刻、最后主动目标时刻、短窗原帧历史及冲突锁存。只有它能调用意图适配器和更新 `flight.command`；任何串口对象不能直接操作 Core。同一帧在备路可能是“对本路新的”，在共享层已是副本，两层判断必须区分。

8 位计数器的局部新旧比较建议使用：

```text
delta = (new_counter - old_counter) & 0xFF
0：重复；1..127：模意义下向前；128..255：旧值或歧义
```

模比较只在已建立且未失效的短时连续流内使用。50 Hz 下半圈是 2.56 s、整圈是 5.12 s；实际资格失效门限应远短于半圈。再限制单次跳号量（台架建议最多 8）及本地间隔，超限标记该路需重新定基，不能把所有 1..127 的跳号都无条件当新帧。丢失帧只计数，不按旧轨迹补执行。`255→0` 是正常前进；计数器变小不是自动清基准的理由。

副本历史建议保存最近 16 个扩展逻辑序号对应的原始 14 字节、首次接收时间和已见端口，TTL 初值 200 ms；跨路关联还须满足最大副本到达差 `T_pair`（初值 40 ms）。扩展序号仅在上述连续性检查下由计数差推导，不按 counter 全局建永久表。比较完整原帧而不是仅比较校验和：同一短窗身份同内容只提交一次；不同内容即使各自校验通过，也锁存冲突、撤销主动输入，不凭主路优先断言正确。迟到副本仍可触发冲突，但不能撤销已发生的运动。

超过配对窗口的旧副本不提交、不刷新共享期限；不同轮次、失联恢复或无法关联的同计数帧不能武断判成同一事务，应将该路置为重新取得资格。仅凭这些字段仍无法可靠识别“上一圈延迟帧”或所有远端重启；清理发送/驱动缓存及受控恢复是前提，不能声称消除了全部陈旧帧风险。

#### 主备切换、冲突与恢复

| 场景 | 共享仲裁处理 |
| --- | --- |
| 主路健康 | 只提交主路新的合法帧；备路持续解帧、校验与副本比较，不重复派发意图。主路提交不必等待另一副本。 |
| 主路异常、备路合格 | 备路连续好帧达到资格门限且未超时；备路帧须满足应用驻留期限、晚于共享最后提交计数、无冲突。以此新帧切换并提交一次；备路只有已提交副本时等待下一帧，不重用旧目标。 |
| 主路异常、无合格备路 | 立即 `active_valid=false`，内部发布 invalid 并通知 Core；不得等满控制超时才继续承认失效主路。Core 独立 watchdog 作为通知未送达时的后备。 |
| 原主路恢复 | 与当前主路短窗帧对齐计数相位，重新累计合格帧，成为备路；不自动抢占回切。 |
| 双路同身份内容冲突 | 锁存冲突，停止普通操作和主动目标接纳；Core 按冻结策略安全处置，人工/受控恢复前不因下一帧一致自动解锁。 |
| 全局目标期限届满 | 即使备路仍有字节或重复帧，也撤销控制。切路、收到副本、更新 JSON sequence 都不能重置期限。 |
| Gateway/Core/飞控重启或长期断流 | 清理旧待办和缓存，保留未决事务供核对；建立新内部连接/授权代次，重新取得通信资格，不把旧“控制”意图当新开始。 |

状态为待机、准备或未授权时，仍可观察合法周期流以维护通信健康，但不缓存目标供以后执行。进入操控后，只接纳晚于本地授权建立时逻辑序号基准的新帧；共享主动目标期限只由当前授权下被接纳的新目标刷新，其他状态帧及 A/B 副本不能续期。这是内部接纳边界，并不能证明线上延迟帧的真实生成时刻。

切路只改变输入来源，不重置共享计数、意图代次、动作事务、Core 授权或全局目标期限。软切换能否保持操控取决于 Core 仍有有效授权、全部前置条件满足且新备路目标在期限内；否则走安全退出并重新使能。候选目标的内部 `sample_mono_us` 保留该逻辑帧首次完整接收时间，不能改成切换时刻来延寿。

热备资格建议要求至少 10 个连续递增合法帧，并与当前主路历史在配对窗口内有一致副本以证明相位一致；发生错误、跳号或超时重置连续好帧计数。主路已失效时，曾合格备路仍可在自己的短时有效窗口内切换；未合格备路不得绕过资格立即接管。启动或双路均无可信基准时，只在禁控的受控初始化阶段选择一条候选流建立新基准，再累计资格；该过程不恢复操控授权。

任何路上传来的急停/禁止控制与主路内容冲突时，冲突本身即触发撤销主动输入；不把备路当第二个可独立发运动命令的源。物理释放不是冲突处理的默认动作，必须由 Core 的冻结安全策略决定。失联原因可映射通信超时 0x0001，已判定控制权丢失可映射 0x000B；双路不一致没有专有码，是否归入控制权丢失须冻结，详细原因始终留在内部诊断，不自行占用新码。

### 9.4 上行周期调度与短写处理

每 20 ms 从 Core 获取一份最新聚合快照，检查子数据时效，编码成两路 65B 帧。不得把旧采样值仅更新时间后报为新测量。错过周期时跳过旧发布，不补发历史帧。每路仅保留一个不可变的正在发送帧 `(bytes, offset, deadline)` 及一个可覆盖的最新待发帧，最大应用载荷 130B；两路发送进度独立，A 阻塞不阻止 B。

`write(fd, bytes+offset, remaining)` 返回正数时仅增加实际写入数量；EAGAIN 等待下次可写，EINTR 在预算内重试。部分写入属于正常需处理的情况，不能当作整帧成功；行为依据见 [write](https://man7.org/linux/man-pages/man2/write.2.html)。一旦开始发某帧就不能用新帧覆盖其余字节，否则会拼接出损坏帧。先完成当前帧，再取最新待发帧；尚未开始且已过期的帧直接丢弃。

开始发送后超过期限或遇到设备错误，应标记本路 TX 失败，暂停继续拼接；按受控端口恢复清理未发送数据并重新同步。已进入驱动或线路的字节无法撤回，必须记录“可能部分送达”，不能把冲刷缓冲当成对端未收到的证明。`write` 全部返回也只表示驱动接纳，不等于飞控收齐。驱动排队时长及 USB 延迟需验收，不能无限提交完整旧帧到内核队列。

主备切换时，两路下一份快照按新角色重新编码并重算校验；已部分发送的旧角色帧仍按原字节完成。由于两路物理进度不同，角色回报可能短暂跨周期不一致，飞控应按冻结的过渡处理，而不是据两个异步帧瞬间认为出现双主。没有上行计数器，飞控无法严格逐周期配对两路反馈，须明确这一边界。

### 9.5 配置、期限与执行顺序

下列名称是计划新增的配置/常量，**数值仅为台架初始建议，不是 RS422 ICD 已冻结要求**。上位机、驱动及机械处置预算验证后再定稿，不直接写入当前尚不支持 RS422 的配置加载器。

| 参数 | 建议值 | 用途 |
| --- | ---: | --- |
| `rx_ring_bytes` | 每路 1024 | 2 的幂，预分配，含 1 个空槽；元数据另计。 |
| `half_frame_timeout_ms` | 20 | 首个 EB 入环起的总收齐期限，不因新字节续期。 |
| `max_rx_residence_ms` | 20 | 首字节入环至提交的最大应用驻留；不可覆盖驱动既有积压风险。 |
| `link_timeout_ms` | 60 | 无递增且格式/语义合法帧，本路失效。 |
| `control_timeout_ms` | 100 | 无新的合法主动目标的共享期限；也约束 Core。 |
| `qualify_frames` | 10 | 启动/恢复合格帧数；热备需短窗相位对齐。 |
| `max_counter_step` | 8 | 连续流最大前进跨度；须同时检查时间间隔。 |
| `pair_window_ms` / `history_ttl_ms` | 40 / 200 | 跨路副本最大到达差及历史保留期，均远小于计数半圈。 |
| `history_entries` | 16 | 有界完整原帧历史；覆盖上述窗口与调度裕量。 |
| `tx_deadline_ms` | 20 | 单帧发送任务期限，部分写入超限使该路 TX 失效。 |
| `io_byte_budget` / `parse_budget` | 每路每轮 256 / 256 | 限额读写及候选解析，另设每轮执行时间预算。 |

启动检查半帧/驻留门限覆盖实测合法帧收齐与调度延迟，`link_timeout < control_timeout`，副本差小于历史 TTL，历史窗口/最长允许比较间隔小于 128 个周期。容量不能被用来放宽时效；恢复帧数是防抖资格条件，不是让失控目标多保持 200 ms 的理由。

循环顺序建议为：检查全局及各路期限 → 接收/处理 Core 安全状态 → 轮转读取和解析 A/B → 比较副本并仲裁 → 处理安全意图及有界普通意图 → 发布最新连续目标 → 到期生成上行快照 → 分路推进短写 → 提交非阻塞记录副本。循环不能等待 Core RPC、串口排空、磁盘或 Logger 确认。各步骤结束后必要时再次检查期限，Core 与设备本地 watchdog 独立运行。


## 10 Monitor Logger 与 Replay

### Monitor 与 Plotter

Monitor 默认只读，使用订阅线程持续收取消息，UI 按 10—20 Hz 刷新最新快照，显示控制源、运行状态、当前及历史错误码、双臂双手状态、视觉置信度与方向盘位置。通信面板同时显示接收频率、原始数据年龄、序号缺口、会话重启和无效消息计数。

连接正常不等于业务健康；显示过期数据时必须明确标记 STALE 和年龄。Plotter 使用有界环形历史，按时间戳对齐指令、接受目标与反馈，显示采样间隙并保留 min/max 降采样，不能用平滑曲线掩盖缺失数据。

### 数据记录节点 aviator_logger（C++ / MCAP）

aviator_logger 是独立部署的基础节点，统一使用 MCAP 作为机器人数据归档格式，不再以 JSONL 或自定义分块文件作为主记录格式。使用 MCAP 官方 C++ 库封装 Writer/Reader，供 Logger、Replay 和 Plotter 共用。[[7]](#ref-7) 本节中的采集路径、数据命名、队列和文件管理策略均为 AVIATOR 工程约定。

#### 全量记录范围

“全部机器人数据”指本次配置启用的全部设备与软件数据源，在采集源原始频率下记录每个样本，禁止默认降采样或 latest-value 覆盖。建立 recording_manifest，列明每个数据源的生产者、Schema、单位、采样率、时钟域、启用状态及预计带宽；新增数据源必须同步注册采集与验收项。未安装或硬件不提供的数据显式标记 unavailable，不能将总线周期聚合反馈当作全部原始反馈。

| 数据类别 | 必须记录的内容 | 采集路径 |
| --- | --- | --- |
| 控制与状态 | 八个主 Topic、system.state、system.diagnostic、system.event 及全部已注册扩展 Topic | Logger 订阅本地总线全部 Topic，逐条保存收到的消息。 |
| 设备原始输入输出 | RS422 收发字节及校验结果、USB 原始报告、驱动实际提供的关节位置/速度/力矩/电流/温度/故障、传感器数据 | 设备适配层在解析或聚合前后提供带方向和样本标识的记录副本。 |
| 实时控制数据 | 每伺服周期的输入目标、插值目标、实际下发量、反馈、限幅/安全判定、周期耗时和 deadline miss | RT 写预分配有界 SPSC 环形队列，非实时采集线程取出并传输；不只记录总线降采样状态。 |
| 视觉与媒体 | 已启用图像记录时的采集帧、相机参数、帧号、采样时间、检测结果及其关联帧号；已配置的深度/点云 | 相机采集侧向有界队列提交原始 RGB8/Z16 副本，经独立 5557 入口交给 Logger；检测结果仍走控制总线。 |
| 服务与事件 | 可靠服务完整请求、响应、执行结果、授权变化、状态转换、故障与恢复 | 服务两端通过记录适配器采集；system.event 摘要不能替代完整服务记录。 |
| 运行上下文 | 构建哈希、依赖清单、协议 Schema、设备清单、配置、标定、坐标变换及其版本、时钟同步状态 | 会话开始保存快照，运行中变更保存新版本及生效时间；剔除密码、密钥等凭据。 |

原始媒体和高频伺服数据不经过控制 XSUB/XPUB，不受业务 JSON 的 64 KiB 上限约束，也不以 Base64 图像挤占控制总线。当前相机把 BGR8 转成 RGB8，连同 Z16 深度、相机 ID、帧号、源会话和采样单调时间，通过 `5557` 送入 Logger；Logger 按配置原样保存，或将 RGB 有损编码为 H.264/H.265、深度无损编码为 Zstd。设备/伺服原始记录仍待实现。控制线程和相机检测线程均不得等待图像压缩或写盘。

#### 离散指令的完整记录链路（待实现）

Gateway 与 Core 的非实时记录适配器在实际收发点保存服务原始 JSON，并记录受理决策、状态迁移、最终结果和超时；Gateway 另记录串口原始 RX 块/TX 帧、A/B 端口、解析拒绝原因、环形缓冲溢出、计数器比较、主备变化及内部事务映射。发送尝试、实际交给串口驱动的字节和发送失败须分别标明；本机 TX 不证明飞控已收到。A/B 物理副本、每次内部服务重试和 TX 部分写入分别留痕；每个已派发意图关联一个内部事务，串口周期反馈不伪装成事务应答。

记录身份包括 recording_session_id、producer_id、producer_session_id、逐源 record_sequence；事务关联包括本地串口运行连接 ID、链路代次、端口、8 位帧计数器、逻辑接纳序号、首次完整接收单调时间、完整原帧、指令意图代次、client_id/client_session_id/request_id，以及 Core server_session_id。线上没有 timestamp_ms 或请求 ID；8 位计数值不能跨回绕/重启单独作为日志主键。Core 发起设备服务时，另记录父事务与设备子请求的关联，不复用父 request_id 代替全部子动作。记录本地单调时间、clock_id 和可用 UTC，不把本机接收时间解释为飞控采样时间或端到端延迟。

普通采集模式通过独立记录通道发送有界副本；溢出或发送失败必须记录缺口并将会话标记 incomplete，不能承诺每条指令必达。要求故障后可补齐的试验启用 ZMQ 协议第 15.5 节的确认补传模式：非实时持久化缓存、Logger 持久化确认、按记录身份补传去重。记录确认只证明记录持久化，不证明业务受理或物理动作完成。现有相机 5557 PUSH/PULL 入口不具备此能力，不得混入不同线格式。

Logger、磁盘或缓存故障不得阻塞实时线程、串口收发及安全退出；严格记录模式下缺少记录就绪条件禁止新的普通试验使能/动作，已运行任务按冻结的安全策略处理，安全停止始终优先。仍可能发生的入队前崩溃或双盘故障须标记未知/不完整，不宣称任意故障下零丢失。采集字段、记录主题和验收规则以 ZMQ 协议第 15.5 节为准。

#### MCAP 数据映射

MCAP 提供 Schema、Channel、带时间戳的 Message、Metadata、Attachment 及索引结构；Message 的时间单位为纳秒，sequence 为 uint32。[[6]](#ref-6) 本项目规定以下映射：

| MCAP 对象 | AVIATOR 映射规定 |
| --- | --- |
| Schema | JSON 消息使用类型名与协议版本命名，encoding=jsonschema，内容为对应 JSON Schema；二进制记录信封使用冻结的 Protobuf Schema，并嵌入含依赖的描述符。 |
| Channel | 总线消息保留原 Topic；按 Topic、Schema 版本、publisher_id、session_id 区分 Channel。原始数据使用 record.raw.*、record.servo.*、record.camera.* 命名。 |
| Message.data | 合法 JSON 消息直接保存原始 Frame1 字节，message_encoding=json；媒体与原始设备数据使用 message_encoding=protobuf 的记录信封，内含原始字节及解码参数。 |
| Message.log_time | 数据 MCAP 使用 Logger 接收时的 Unix UTC 纳秒；图像 MCAP 使用相机写入时的 Unix UTC 纳秒，原始采样单调时间保留在 Protobuf 消息中。后续 record.ingest 用于保存全局接收顺序。 |
| Message.publish_time | 有明确发布 UTC 时使用该时间；当前 timestamp 是快照生成时间，不能直接冒充发布时间，故缺失发布时刻时取 log_time，采样时间仍保留于原消息。微秒转纳秒使用检查溢出的 uint64 运算。 |
| Message.sequence | 使用源 sequence 的低 32 位；原始完整序号保留在负载或采集信封中。完整性核对使用完整序号与源会话，不能仅凭该字段判断。 |
| Metadata / Attachment | 元数据保存会话 ID、构建与配置哈希、分卷序号及完整性状态；附件保存配置、标定和数据清单快照。 |

json、protobuf 与 jsonschema 的编码名称遵循 MCAP 官方注册表。[[8]](#ref-8) 未识别 Topic、格式错误或不符合 Schema 的消息保存至 record.invalid 的二进制信封，保留 Topic、原始各帧及拒绝原因，不伪装成合法 JSON；超过记录入口安全上限时显式记录丢弃计数和原因。

#### 线程、文件与完整性

Logger 的接收线程负责总线时间戳和有界入队，写盘线程独占数据 MCAP Writer。相机的后台线程独占图像 MCAP Writer，图像待编码队列最多两帧；压缩、索引和磁盘 I/O 不进入检测线程。Logger 不使用控制端的 latest-value mailbox，也不直接照搬控制端 HWM=8。

当前分别使用 `data.mcap` 和 `camera.mcap`，以相同记录会话 ID 关联；图像 MCAP 的 Protobuf 消息用源会话、相机 ID 和帧号与检测结果关联。两份文件均先写 `.partial`，正常关闭后才发布最终文件。图像使用 4 MiB Chunk、索引与 CRC，PNG 负载不再叠加 Chunk 压缩。自动分卷、会话清单和周期性持久化尚未实现；两小时记录前必须实测图像体积、编码吞吐和允许的尾部丢失窗口。

正常关闭时先让生产者结束采集并提交结束序号，Logger 限时排空队列、写完索引和文件尾，再退出。崩溃遗留文件保留原件，恢复工具仅将可校验的完整记录导出为新 MCAP，重建索引并报告丢失区间；不保证恢复未落盘缓存或损坏 Chunk。

全量是采集范围与正常负载验收目标，MCAP 格式不使 ZMQ PUB/SUB 获得可靠传输。按数据源、源会话和完整序号核对生产、接收、写入计数；记录重复、缺口、未知起始区间、溢出和会话中断。生产者开始/结束清单用于发现首尾丢失，不能仅凭中间序号连续就宣称完整。Logger 输出 READY、RECORDING、DEGRADED、ERROR 状态，并发布队列字节数、最老等待时间、写入速率、剩余磁盘与缺口计数；自身状态本地记录并避免订阅回写形成递归。

慢盘或磁盘满时不阻塞控制线程或总线，超限丢弃必须将会话标记 incomplete，并经独立健康通道告警；恢复后补写故障信息。若试验要求故障期间仍可补齐，应增加生产者非实时持久化缓存与可靠补传，以源身份和完整序号去重；实时线程仍只进行有界入队，缓存耗尽须报告缺失。未部署补传能力时不能承诺无损记录。

### Replay 模式与隔离

| 模式 | 处理方式 | 边界 |
| --- | --- | --- |
| 查看回放 | 只读取日志，以原时间轴展示 | 不发布控制消息。 |
| 算法回归 | 仅重放选定输入 Topic，Core 输出另行记录比较 | 独立 TCP 端口、模拟执行器；不同时回放历史 Core 输出。 |
| 消息复现 | 复现接收顺序、缺口、延迟、倍速或单步 | 源时间保持为元数据，使用可注入虚拟时钟或重映射时间。 |
| 硬件在环 | 明确选择测试配置与允许的设备 | 显式测试授权、限幅和现场使能；禁止自动切入。 |

Replay 与 Plotter 通过共用 MCAP Reader 按 Topic、时间和源会话读取；原始图像依帧号与检测结果关联，源采样时间用于分析，全局接收顺序用于消息复现。Schema 不兼容或会话存在缺口时显式提示，不能静默补齐数据。

回放总线使用独立 TCP 端点：发布入口 tcp://127.0.0.1:6555，订阅出口 tcp://127.0.0.1:6556；默认拒绝生产端点。需要重发时分配新的 session_id 和 sequence，保留 original_header；不得沿用历史 control_epoch。倍速和单步回放必须统一算法时钟，不能一边暂停回放一边误用真实时钟触发全部超时。

## 11 资源开销与性能评估

以下为容量规划假设，单位 kB=1000 字节，不含 ZMQ 封装、内核缓存、JSON 对象和相机图像。实际大小随手部自由度、数值精度和扩展字段变化，应以抓取的紧凑 JSON 为准。

| Topic | 假设 kB/帧 | 频率 Hz | 输入流量 kB/s |
| --- | --- | --- | --- |
| flight.command | 0.6 | 50 | 30 |
| flight.state | 4 | 50 | 200 |
| arm.command | 1.5 | 100 | 150 |
| arm.state | 2 | 100 | 200 |
| hand.command | 1 | 50 | 50 |
| hand.state | 1.5 | 10 | 15 |
| camera.command | 0.5 | 30 | 15 |
| camera.detection | 1 | 30 | 30 |
| 合计 | — | 420 | 690 |

总线输入约 0.690 MB/s，即 5.52 Mbit/s；若每条消息平均有 4 个接收者，输出约 2.76 MB/s。Bus 入站与出站逻辑数据总量约 3.45 MB/s，不等于内存复制量或实际网卡流量。远端全量订阅约需 5.52 Mbit/s 纯业务带宽，应另留协议与突发裕量。

Logger 原始有效负载约 2.484 GB/小时、19.872 GB/8小时；按 1.5 倍包装与索引预算为 3.726 GB/小时、29.808 GB/8小时。压缩收益需实测，不预先抵扣容量。上述数字仅覆盖八个主 Topic，不代表全量记录容量。相机原图、点云和伺服数据通过独立记录通道写入 MCAP。

全量原始速率按 R_total = R_topics + Σ(采样率 × 样本字节数) + R_events 估算。例如单路 1920×1080、RGB8、30 Hz 原图约 186.624 MB/s，即 671.85 GB/小时；若伺服每周期样本 2 kB、1 kHz，再增加 2 MB/s、7.2 GB/小时。相机数量、像素格式、实际驱动采样率和信封开销必须纳入预算。默认保留原始像素或无损编码，有损视频不作为原图的等价替代。磁盘持续写入初始按实测峰值至少 1.5 倍预留，容量按试验时长加分卷/索引与保留余量计算，压缩率不得预先假定。

### 内存与 CPU

队列有效负载可按 Σ(HWM × 平均消息大小 × peer 数量)估算。例如 12 条队列、深度 64、平均 2 kB，约 1.54 MB，仅为消息体下限。还需计入双端队列、对象分配、I/O 缓冲和日志队列；64 MiB 仅可作为低速 Topic 队列的台架起点；上述单路原图速率下仅能缓冲约 0.36 s。媒体与伺服队列分别按“峰值字节率 × 允许写盘阻塞时间 + 最大单条记录”计算，并计入复制与 Chunk 工作区。

CPU 占用不能仅凭频率给出百分比。应分别测量 encode/decode、转发、算法、视觉和 UI 耗时；计算 Σ(每秒调用数 × 单次耗时)得到核秒预算，再测突发与长尾。当前通信量不构成改用二进制协议的充分理由，但也不保证指定硬件必然满足时序。

### RS422 必须独立核算

串口编码使用固定二进制帧，不发送约 4 kB 的内部 FlightState JSON。每路每方向独立按 115200 bit/s、8N1（每字节 10 bit）、50 Hz 计算：

| 方向 | 帧长 | 带宽 | 占串口速率 | 单帧线路时间 |
| --- | ---: | ---: | ---: | ---: |
| 飞控→机器人 | 14B | 7000 bit/s | 6.08% | 约 1.22 ms |
| 机器人→飞控 | 65B | 32500 bit/s | 28.21% | 约 5.64 ms |

双路各自承载完整周期流，不分摊数据；没有额外 REQUEST/REPLY 预算。不得分页、动态改长度或单方协商降频。第 9.2 节环形缓冲按线路峰值而非平均业务流量预算；应用排队、USB/驱动缓存与调度抖动另行测量。

## 12 故障检测与安全机制

| 故障场景 | 检测路径 | 规定响应 |
| --- | --- | --- |
| 总线退出或卡死 | 消息年龄、收帧间隔；外部健康检查 | Core 与控制器本地 watchdog 生效；Bus 重启不解除安全锁存。 |
| Core 退出或卡住 | 执行器检测命令与 origin 过期 | 本地停止推进新目标，执行经验证的安全动作。 |
| RS422 单路异常 | 每路递增有效帧期限、解帧/UART/溢出统计 | 仅切换至已合格且有新鲜未提交帧的备路；不重置共享操控期限，不重复动作。 |
| RS422 双路断流、冲突或计数器歧义 | 第 9.3 节共享仲裁器及 Core 独立 watchdog | 撤销主动输入，内部 invalid，Core 执行冻结安全策略；冲突锁存，恢复不自动授权。 |
| RS422 上行单向故障 | Gateway 本地 TX 错误及飞控各路接收期限 | 本地 write 成功不证明飞控收到；按双方冻结的闭环策略禁止控制/恢复，不仅凭下行健康断言全双工正常。 |
| 摇杆拔出 | Gateway 检测设备检查时效 | 发布 invalid 或停止有效输出；禁止以重复旧值维持“健康”。 |
| 乱序、重复或旧会话 | 按 Topic 与生产者检查 sequence/session | 丢弃；不刷新有效输入期限；新会话须重新校验授权。 |
| 反馈失效或视觉丢失 | freshness、置信度和设备状态 | 退出依赖该数据的控制模式；不得将失效数据归零后继续控制。 |
| 多个源同时发布 | 部署互斥与 Core 白名单 | 只接受已授权源；冲突上报，禁止按消息到达时间抢控制。 |
| JSON 异常或越界 | Schema、长度、范围和状态机检查 | 拒绝执行，限速记录，保留本地 watchdog。 |
| 慢 UI、慢盘或远程端 | 队列、处理延迟和磁盘余量 | 丢弃观测消息并标记缺口，不阻塞控制环。 |
| 驱动故障或主机失效 | 驱动 watchdog、硬件安全链路 | 执行设备层安全机制；软件总线不承担唯一急停路径。 |

### 安全动作的确定

“失联后保持、回中、卸力或松手”没有通用安全答案。应依据飞机操纵机构、当前任务、抓握方式和可用接管路径定义模式化安全策略，并由系统风险分析和台架试验验证。软件必须提供可配置且有界的安全状态转换；在策略未冻结前，禁止进入实际操控使能状态。

优先级为硬件急停与驱动保护高于本地安全限制，高于 Core 状态机，高于正常目标。安全检测不依赖 Monitor 或 Logger；独立硬件急停和驱动失联保护覆盖进程、内核或主机失效。

### 故障后的恢复

恢复顺序为故障原因解除、设备自检通过、当前状态与物理位置核对、缓存清理、新会话及新 control_epoch 生效、连续新鲜数据验证、显式重新使能。RS422 的连接 ID、链路代次和 control_epoch 仅为内部身份；恢复须协调飞控停止旧流、清理队列、观察待机/禁止控制并重新授权，不能从 8 位计数器自动识别全部远端重启。重连、systemd 重启、错误码清零均不等于允许运动。上次错误保留并记录恢复事件，便于追溯。

## 13 启动流程与部署方案

### 运行配置

提供 flight、joystick、simulation、replay 四种配置。flight 默认只启动 Flight Gateway；joystick 只启动 Joystick Gateway，使用测试限值；simulation 使用虚拟设备；replay 使用独立端点、日志时钟和模拟执行器。配置文件包含端点、源白名单、频率、超时、HWM、设备映射、标定版本和日志路径。RS422 模式新增两路稳定设备路径、协议 profile、环形缓冲容量、主备偏好及第 9.5 节参数；启动时校验路径不同、格式固定和必需语义已冻结。当前 `config/flight.yaml` 的 RS422 模式仍被明确拒绝，新增配置项须与运行代码和 Schema 一并交付。

### 有条件的启动顺序

| 步骤 | 启动内容 | 进入下一步的条件 |
| --- | --- | --- |
| 1 | 加载配置与权限 | Schema、标定、关节数、设备映射和运行模式一致。 |
| 2 | aviator_bus | 单实例锁与端点 bind 成功；报告就绪。 |
| 3 | 控制器、相机与 Logger | 设备自检完成；执行器保持未使能；关键日志会话已建立。 |
| 4 | 选定 Gateway 与 Core | 订阅已建立且获得连续有效样本；所有必须反馈新鲜。 |
| 5 | 授权与业务状态转换 | 源身份、抓握与安全条件满足；明确进入运行状态。 |

“进程已启动”与“业务已就绪”分开。systemd 的 After 只用于顺序，业务仍须通过健康状态和样本新鲜度判断可用性。先启动接收侧可以降低启动丢帧，但不能替代协议层就绪检查。

### 路径与权限

程序安装到 /opt/aviator/bin，配置位于 /etc/aviator，日志位于 /var/log/aviator。使用专用非 root 服务账户和设备访问组；为串口、USB、EtherCAT 设备分配最小权限。控制总线默认仅绑定 127.0.0.1，发布入口与订阅出口地址、端口由配置统一维护；端口占用时启动失败并报告错误。跨主机部署时显式配置 Bus 的受控网卡地址及客户端连接地址，沿用第03章的访问控制要求。独立记录通道同样采用 ZMQ TCP，默认使用 tcp://127.0.0.1:5557，与控制总线端口分离；地址与端口统一纳入配置。TCP 访问不依赖文件系统路径权限，跨主机接入按第03章配置网络访问限制与认证。

Bus 持有单实例文件锁，两个 TCP 端点均 bind 成功后才报告就绪；任一端点绑定失败则释放已占用资源并退出。多个业务服务不要各自删除公共运行目录。生产与回放端口、目录、服务目标和配置分别隔离。

### 关闭与升级

先停止新操作并撤销控制授权，等待执行器进入允许的安全状态，再停 Gateway/Core、设备进程、Logger，最后停止 Bus。进程收到 SIGTERM 后执行有限时退出，socket 采用受控 LINGER；不能依赖无限等待来清空队列。

升级采用固定依赖与可回滚的软件包；启动记录构建哈希和配置哈希。控制软件升级应在未使能状态进行，禁止以热重启掩盖控制中断。远程监控连接数、CPU 和 I/O 配额应经过峰值负载验证。

## 14 systemd 服务建议

下例为 Bus 服务建议，前提是程序实现 sd_notify 就绪通知，并在绑定端点成功后发送 READY=1。[[5]](#ref-5) 若尚未实现，可暂用 Type=exec，但业务就绪检查仍不可省略。

```ini
# /etc/systemd/system/aviator-bus.service
[Unit]
Description=AVIATOR message bus
PartOf=aviator.target
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=notify
NotifyAccess=main
User=aviator
Group=aviator
RuntimeDirectory=aviator
RuntimeDirectoryMode=0750
UMask=0007
ExecStart=/opt/aviator/bin/aviator_bus --config /etc/aviator/bus.yaml
Restart=on-failure
RestartSec=1
TimeoutStartSec=10
TimeoutStopSec=5
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/run/aviator

[Install]
WantedBy=aviator.target
```

### 业务服务依赖与守护

Core、Gateway、Controller 服务建议使用 Wants=aviator-bus.service 与 After=aviator-bus.service。Bus 意外退出时保留控制器进程，使本地安全处理可以持续；是否使用 Requires、BindsTo 或整体停机策略必须按故障处置方案明确，不要让依赖关系意外杀死本地安全执行。

业务服务使用 Restart=on-failure 与限频重启；每次重启都从未使能状态开始。真实网关与摇杆网关增加双向 Conflicts，并由 Core 再次检查源授权。aviator.target 的 Wants 依据运行配置选择，UI 可独立启停；Logger 默认随采集任务启动，全量记录任务在 Logger 未就绪时禁止进入试验使能，运行中记录失效按冻结的模式策略处置，不直接杀死控制器。Logger 使用专用可写记录目录和磁盘配额；ProtectSystem=strict 下配置 ReadWritePaths，停止超时应覆盖限时排空与文件关闭预算。

仅在程序真实实现健康通知后配置 WatchdogSec。例如 2 s 的 systemd watchdog 用于秒级进程失活检测，不替代 50—100 ms 级控制 watchdog。不得由无条件定时器发送“健康”而掩盖业务线程卡死；Bus 可用外部探测往返验证转发链路。[[5]](#ref-5)

实时控制器单独配置允许的 memlock、实时优先级上限和设备权限，并由程序仅提升伺服线程。CPU 与 I/O 隔离经测量后配置；严苛 CPUQuota 可能引入调度停顿。部署前执行服务配置校验和总线故障恢复演练。

## 15 精简工程目录与模块接口

按“公共代码 + 运行节点 + 配置 + 测试”组织，目录与第02章进程一一对应。只提取多个节点确实共用的代码，节点私有的算法、驱动与安全逻辑留在节点目录，不预建多层框架。以下是目标工程结构，不表示当前仓库已完成迁移。

```text
aviator/
├── CMakeLists.txt                 # 统一管理公共库、节点、测试与安装
├── CMakePresets.json              # 构建与测试配置
├── README.md
├── cmake/                        # 依赖版本与构建配置
├── third_party/                  # 需要随工程保存的第三方源码
│   ├── README.md                 # 来源、版本/提交、许可证与本地补丁说明
│   ├── pinocchio/                # 运动学与动力学
│   └── pin_ik/                   # 逆运动学；其他源码依赖按需加入
├── common/                       # 共用代码；头文件与源文件就近存放
│   ├── CMakeLists.txt
│   ├── protocol.hpp / .cpp        # 强类型消息、Topic、JSON 编解码与校验
│   ├── transport.hpp / .cpp       # ZMQ context、收发与 TCP 端点
│   ├── runtime.hpp / .cpp         # 时钟、有界队列、快照与 watchdog
│   └── recording.hpp / .cpp       # 非实时采集适配、MCAP 读写与会话管理
├── nodes/                        # 每个子目录对应一个可执行程序
│   ├── aviator_bus/               # XSUB → proxy → XPUB，仅转发
│   ├── flight_gateway/            # RS422 或 USB 摇杆，通过配置选择
│   ├── aviator_core/              # 仲裁、状态机、规划与状态聚合
│   ├── manipulator/               # 双臂设备接入、轨迹执行与状态
│   ├── aviator_hand/              # 双手 SocketCAN、命令保护与实际位置反馈
│   ├── camera/                    # 图像采集与检测
│   ├── aviator_logger/            # 全量数据接入、MCAP 写盘与分卷
│   ├── aviator_monitor/           # 状态监测
│   ├── aviator_plotter/           # 实时与离线曲线
│   └── aviator_replay/            # MCAP 读取与隔离回放
├── config/
│   ├── system.yaml               # 运行模式、输入源、端点、频率与超时
│   ├── robot.yaml                # 臂手设备、关节、限位与标定
│   ├── inspire_hand.yaml          # 双手 CAN、驱动参数、端点、频率与超时
│   ├── camera.yaml               # 相机与检测参数
│   └── recording.yaml            # 全量数据清单、队列、MCAP 与磁盘配置
├── schemas/                      # 消息 Schema 与记录信封定义
├── tests/                        # 协议、集成、记录回放与故障测试
├── deploy/                       # systemd 服务与必要的设备权限规则
└── docs/                         # 架构、接口与使用说明
```

图中 `protocol.hpp / .cpp` 等表示同名头文件与源文件。每个节点初期只需 `CMakeLists.txt`、`main.cpp` 及少量职责明确的 `.hpp/.cpp`；代码规模增长后再拆子目录。构建产物位于 `build/`，MCAP 数据写入配置指定的运行目录，均不纳入源码管理。

flight_gateway 内规划保留 RS422 与 USB 两种输入适配，配置选择其中一种，不再单设 joystick_gateway。RS422 新增节点私有模块：`serial_port`（非阻塞收发与配置）、`byte_ring`（有界字节与接收时间存储）、`rs422_codec`（14/65B 固定布局与校验）、`dual_link`（共享仲裁与短窗副本比较）、`flight_intent`（意图幂等及 Core 服务分流）。这些是目标模块名，不代表已有文件；先复用现有 `gateway`、`common/service` 和 `InputGuard`，不新建进程或通用通信框架。manipulator 保留 arm.* 目标缓存、实时执行和状态发布；aviator_hand 独立处理 hand.* Topic、SocketCAN 双手和本地安全姿态。camera 统一使用第02章名称。

Logger 是正式运行节点；Monitor、Plotter 和 Replay 也统一放入 nodes，避免 apps、nodes、tools 三套目录。暂不增加独立的 topic_echo、topic_hz、command_sender、启动脚本或多层公共库目录；确有需求时再新增。部署服务名称与节点名称对应，统一由 systemd 管理启停。

### 第三方依赖与用途

依赖按实际使用的模块链接，不要求每个节点加载全部库。下表为目标架构选型，现有工程中的其他实现按迁移计划替换，不表示当前代码已完成切换。

| 依赖库 | 用途 | 使用位置与约束 |
| --- | --- | --- |
| Pinocchio | 机器人模型、正运动学、雅可比与动力学计算。 | aviator_core 的运动计算，以及 manipulator 按需使用的模型计算；源码放 third_party/pinocchio-3.9.0。能力依据见 [Pinocchio 官方项目](https://github.com/stack-of-tasks/pinocchio)。 |
| pin_ik | 逆运动学：将末端位姿目标求解为关节目标。 | aviator_core 的运动规划；源码放 third_party/pin_ik-2.2.0。以当前仓库 pin_ik-main 为迁移来源，其 CMake 声明依赖 Pinocchio、Eigen3、NLopt 和 Threads；求解设置时间/迭代预算，失败时不发布为有效目标。 |
| Eigen3 | 向量、矩阵、位姿及数值计算基础。 | 运动算法及 Pinocchio、pin_ik 的共用依赖，统一版本。 |
| NLopt | pin_ik 使用的数值优化求解依赖。 | 按 pin_ik 的构建要求链接，不向所有节点扩散。 |
| spdlog | 异步输出运行日志，支持控制台与滚动文本文件。 | 各节点的非实时运行诊断，经 common/runtime 统一初始化队列、日志级别与输出位置；见 [spdlog 官方项目](https://github.com/gabime/spdlog)。 |
| yaml-cpp | 读取 YAML 配置并转换为强类型配置对象。 | 启动时读取 config/*.yaml，检查必填项、类型和范围；不在伺服周期解析配置。见 [yaml-cpp 官方项目](https://github.com/jbeder/yaml-cpp)。 |
| nlohmann/json | 总线 JSON 消息的解析与序列化。 | common/protocol 使用；JSON 编解码与业务字段/Schema 校验分别实现，不将解析成功等同于协议有效。见 [JSON for Modern C++ 官方项目](https://github.com/nlohmann/json)。 |
| libzmq + cppzmq | ZMQ 通信实现与 C++ 接口。 | aviator_bus 及 common/transport，实现前述 TCP 消息总线与独立记录通道。 |
| MCAP C++ | 机器人全量数据文件的写入、读取与索引。 | common/recording，供 aviator_logger、aviator_replay 与 aviator_plotter 使用，见第10章。 |
| Protobuf、Zstd / LZ4 | 二进制记录信封编码与 MCAP Chunk 压缩。 | common/recording；压缩库仅链接实际启用项，控制总线仍使用 JSON。 |
| 设备与相机 SDK | 设备接入、命令执行及原始数据采集。 | 分别封装在 manipulator、aviator_hand、camera、flight_gateway 内，按部署硬件启用。 |

spdlog 负责面向开发与运维的文本日志，MCAP 负责带时间戳的全量机器人数据，两者独立配置。异步日志仍有入队、格式化与内存开销，不能视为硬实时安全接口：伺服线程只写预分配诊断队列，由非实时线程调用 spdlog。日志队列设置容量和溢出策略，实时相关路径不得等待文本日志落盘；必要事件同时进入 MCAP 记录链路，不能以文本打印替代数据记录。

third_party 只存放确需保留的源码，例如需要本地修改、固定提交或离线构建的 Pinocchio、pin_ik；其余依赖优先使用锁定版本的安装包，经 find_package 接入，需要源码时再增加同名子目录。保留上游目录及许可证，来源、提交号、校验值、构建选项和补丁记录在 third_party/README.md，避免在多个节点下复制同一库。Pinocchio 的模型解析与碰撞等可选依赖按启用功能补齐，不强制引入全部组件。

源码依赖由 cmake/ 统一接入：支持作为子工程的库使用 add_subdirectory；需要独立配置的库先构建并安装到 build/third_party/install，再通过指定前缀 find_package。依赖构建产物不写回 third_party 源码目录；同一依赖只选择一种来源，Pinocchio 与 pin_ik 使用同一套 Eigen、编译器及 ABI 配置。默认关闭不需要的第三方示例、测试与语言绑定。源码目录纳入版本管理或固定提交的子模块，构建与安装目录不入库。

### C++ 与 CMake 工程维护规定

统一采用 C++17、RAII 和强类型接口；实时路径避免异常传播及不可预测分配，错误显式返回。业务算法不依赖 MCAP、ZMQ 或 UI 类型。共用协议、通信、运行基础与记录功能分别放在 common 的对应文件中；控制算法和设备适配留在所属节点。RT 仅依赖强类型数据与预分配采样接口，不调用 MCAP Writer。

common 中 protocol、transport、runtime、recording 按依赖建立小型库 target，由同一个 CMakeLists.txt 维护；每个节点建立独立可执行 target，使用 target_link_libraries、target_include_directories 和 target_compile_features 表达依赖、头文件路径与 cxx_std_17，不使用全局目录堆叠。MCAP 官方 C++ 实现由项目适配 target 封装并链接 Zstd/LZ4 等实际启用依赖；具体 target 名称与集成方式以锁定源码版本为准。Pinocchio、pin_ik、Eigen3、NLopt、spdlog、yaml-cpp、nlohmann/json、libzmq/cppzmq、MCAP、Protobuf、压缩库和硬件 SDK 统一锁定版本与校验值，禁止构建时追踪浮动分支；支持受控依赖缓存与离线构建。

工程维护 CMakePresets.json，包含开发 Debug、发布 Release、仿真与回放配置及对应 build/test presets；本地路径放不入库的 CMakeUserPresets.json。Preset 用于共享可复现配置。[[9]](#ref-9) CMake 最低版本建议 3.24，采用其支持的 Preset 格式，最终编译器与工具版本在发布清单冻结。所有配置使用源码外构建目录；交叉编译使用 cmake/toolchains 下的 toolchain 文件。

下面为规划中的标准命令接口，需在实现阶段落地相应 Preset；不表示当前仓库已提供这些配置：

```bash
cmake --preset linux-release
cmake --build --preset linux-release --parallel
ctest --preset linux-release --output-on-failure
cmake --install build/linux-release --prefix /opt/aviator
```

CMake 统一维护安装规则、配置/Schema/服务文件和 CPack 发布包；设备 SDK 用显式构建选项隔离，仿真构建不要求真实硬件库。CI 至少执行干净配置与构建、CTest 协议/记录读写/集成测试、安装目录检查；调试配置按需运行 Sanitizer，实时性能在目标硬件 Release 构建上单独验证。发布产物保存编译器、依赖版本、Git 提交、构建选项和配置哈希，写入 MCAP 会话元数据。

### 依赖方向

nodes 中各节点负责组装，按需依赖 common 的库，不互相链接节点实现。aviator_core 内的控制算法只依赖强类型协议对象、时钟与安全接口，不依赖 JSON、ZMQ 或设备 SDK；节点内的设备适配代码不反向调用 Core 状态机。transport 负责完整消息收发，protocol 负责协议与编解码，recording 负责 MCAP 存储。manipulator 的双臂实时执行路径只使用强类型数据和 runtime 的有界结构；aviator_hand 独占其 SocketCAN 与 ZMQ socket，按有界轮询执行命令保护和反馈采集。

### 公共接口约定

Clock 提供 UTC 与 monotonic 时间，Replay 可注入虚拟时间；LatestMailbox<T> 提供有界、线程安全快照；CommandValidator 统一来源、授权、范围与时效检查；StateAggregator 输出 FlightState；SafetySupervisor 负责故障锁存、状态转换和恢复条件。

错误码、枚举、Topic 与 Schema 建立单一来源，生成或校验 C++ 定义与离线分析工具的一致性。配置启动时校验，运行中不静默更改关节顺序、单位、坐标系或安全阈值。测试夹具覆盖正确帧、错误帧、重启、乱序和旧数据。

## 16 实施阶段与验收计划

| 阶段 | 交付内容 | 退出条件 |
| --- | --- | --- |
| P0 接口冻结 | ICD 剩余 TBD、状态机、配置、硬件映射和安全策略 | 在 14/65B、115200/8N1 基线上冻结字节序、缩放、双路计数/主备语义、无效反馈、坐标及超时预算。 |
| P1 总线与仿真 | Bus、协议库、模拟生产者、Monitor、Logger | C++/CMake 干净构建通过；八 Topic 写入 MCAP 并可读回；启动丢帧、重连、队列限制符合预期。 |
| P2 摇杆台架 | Joystick、Core、双臂双手接入与本地 watchdog | 拔出、源冲突、旧数据和 Core 停止均进入规定安全状态。 |
| P3 真飞控与视觉 | RS422 双向编解码、环形解析器、双路仲裁、既有 Core 服务的 ICD 扩展、原始记录、Camera 与 FlightState | 完成下述 RS422 专项故障注入及 10/30/50/100 Hz 实测；所需原始设备、伺服与图像记录覆盖清单核对完成。 |
| P4 回放与部署 | Replay、systemd、发布包和运维手册 | MCAP 隔离回放、分卷、索引、崩溃恢复、慢盘、磁盘满与远程干扰测试通过。 |
| P5 可靠服务与优化 | 通用服务扩展与按实测优化（P3 飞控必需服务不延后） | 重试去重、结果未知对账及重新授权验证完成。 |

### 建议台架验收基线

对八个 Topic 同时运行至少 8 小时，接入全量 Logger、Monitor、Plotter 和一个远程观测端；记录发布与接收计数、P50/P95/P99 与最大消息年龄、进程 CPU/RSS、磁盘吞吐、控制周期抖动和所有丢弃原因。不同 clock_id 的时间差不得直接当作单向延迟。

建议初始指标：10 s 窗口发布平均频率偏差不超过 ±2%；同机从发布到接收校验完成的 P99 延迟小于 5 ms；正常负载下不得有未解释的有效命令超时。该指标不包含控制算法与机械响应，最终值由系统时序预算评审确定。

硬性功能判据：所有过期、重复、未授权与范围非法命令均不能进入执行；Bus/Core 被终止后，控制器在配置期限内检测并于下一可用伺服周期启动安全处置；重启不自动使能；回放默认不能向生产端点发布。安全动作完成时长另按物理系统预算验收。

实时验收单独统计伺服 deadline miss 和最坏执行时间。在指定测试窗口内零 deadline miss 是必要测试条件，但不能替代最坏情况分析。满 CPU、磁盘抖动、相机高负载、重连风暴及调度延迟都要纳入干扰测试。

### RS422 专项验收（P3 必需）

先用虚拟串口/字节注入和可控单调时钟验证软件，再以两路真实串口和实际转换器测量。软件解析测试不代表电气层或机械安全验收。

| 场景 | 通过条件 |
| --- | --- |
| 固定布局与校验 | 14/65B、EB 90、上行 ED 03、校验范围准确；软件校验和两字节参与帧校验，帧尾不参与；逐字段编码与规范向量一致。 |
| 环尾及拆包 | 对所有环内起点、逐字节输入和所有拆分点验证；帧头跨环尾、校验字节跨尾及多个粘包均只输出一次完整帧。 |
| 噪声与重同步 | 前后噪声、单 EB、EB EB 90、丢/插字节、坏校验及候选内嵌下一帧正确处理；坏候选前移一字节，业务非法完整消费。 |
| 校验碰撞 | 注入内嵌帧头导致的错位候选且累加校验通过，确认后续语义/连续性检查不被绕过；不将该弱校验宣称为唯一帧边界或完整性保证。 |
| 帧内标记与半帧 | DATA 含 EB 90 / ED 03 不提前截断；无后续输入、慢速滴入均在原候选期限超时，不无限续期。 |
| 容量与时效 | 满环、满速噪声、驱动积压、事件循环暂停和消费过慢显式失效/记录缺口；旧数据不因刚读到或刚切路而重新有效；B 与 watchdog 不被 A 饿死。 |
| 副本与计数 | A/B 任意先后及重复仅一次逻辑提交；255→0 正常；跳号、重复、旧帧、半圈差及计数复位按规则拒绝或重新取得资格。 |
| 身份歧义 | 持续运行跨多次 5.12 s 回绕；延迟副本超配对窗、上一圈同值、端口重开和远端重启不自动恢复授权；无法识别的延迟风险作为链路验收限制记录。 |
| 主备切换 | 断 A/断 B、备路尚未合格、恢复抖动、旧副本及备路新帧分别注入；只以新鲜未提交帧切换，不重置全局期限，原主路不自动抢占。 |
| 内容冲突 | 构造同计数、不同内容但校验均正确的短窗帧，包括迟到副本；锁存冲突并撤销主动输入，恢复一致帧不自动解锁。 |
| 周期意图 | 同一状态/抓握维持多个周期、计数变化及切路不重复派发；短暂退出/禁止控制不被 latest-value 吞掉；失败、拒绝、未知结果不自动重做。 |
| 部分写入与单向故障 | 注入短写、EAGAIN、写入错误/超时；不拼接新旧帧；A 阻塞不阻塞 B；单向断线不能以另一方向健康冒充全双工正常。 |
| 反馈与授权 | 角色字节变更后重算校验；反馈与指令回绕分开；±52°、−5～175 mm 按原比例编码；缺失机构状态及过期 Core 不伪装正常，未授权输入不驱动。 |
| 端到端时序 | 分别记录接收、解帧、仲裁、服务受理、撤销授权及物理安全动作时间；阈值通过目标硬件实测后冻结，不能用检测时间代替停止时间。 |


### MCAP 全量记录验收

在目标硬件以全部已启用数据源的最高配置速率运行至少 8 小时，包含原始图像与完整伺服采样；按 recording_manifest 对账源端开始/结束序号、发送计数与各卷写入计数，正常负载零缺口、零未说明降采样。逐卷验证 Schema 可解析、CRC、索引检索、原始负载字节一致性、采样时间及图像关联；多卷边界无遗漏、无重复。

注入磁盘降速/满盘、Logger 被终止、Bus 重启、UTC 跳变、生产者重启及强制截断文件，确认缺口和 incomplete 状态可见、完整记录可恢复、队列与内存有界，且控制与伺服时序仍满足预算。单独验证 sequence 超过 uint32 后的完整序号对账，以及启动/退出边界的缺失检测；已配置补传时验证重传去重和缓存耗尽处理。

### 必须冻结的开放参数

RS422 两字节字节序/符号/缩放、TBD 反馈与无效值、状态/抓握组合及重做语义、回绕时点、计数器生成/重启规则、主备字段含义、单向故障处置及第 9.5 节门限（波特率/8N1/14B/65B 已确定）；双臂和双手自由度及反馈能力；坐标与 roll/pitch 正方向；允许操纵范围及变化率；视觉依赖条件；各模式安全动作与接管方式；最终 watchdog 阈值；可靠服务上线范围；原始数据覆盖清单、相机格式与数量、伺服记录样本结构、峰值记录带宽、持久化间隔、补传需求、日志保留量与远程接入策略。

## 17 关键设计原则与参考资料

### 必须保持的架构约束

| 原则 | 实现约束 |
| --- | --- |
| C++ 与 CMake | 交付节点统一 C++17；target 化依赖、版本锁定、Preset、测试与安装统一维护。 |
| MCAP 全量记录 | 全部启用数据源按源频率记录，逐源对账；异常缺口显式标记，原始大数据不占控制总线。 |
| 统一总线 | 连续业务消息全部经过 aviator_bus；不在模块间添加隐式控制旁路。 |
| 最新且有效 | latest-value 按 Topic 管理；新到达不等于新采样，年龄和源会话必须检查。 |
| 可靠性分层 | 周期目标可以覆盖；内部离散动作有身份、期限和结果；RS422 无独立应答及事务 ID，回绕不等于动作完成。 |
| 实时隔离 | ZMQ 与 JSON 不进入伺服闭环；设备层安全不依赖总线存活。 |
| 可观测但不反压 | 慢工具不阻塞核心链路；记录缺口显式呈现，不能宣称全量必达。 |
| 恢复不等于使能 | 重连、重启和复位后回到安全状态，重新确认运行条件。 |
| 用测量决定优化 | 带宽、CPU、内存和延迟分别评估；协议演进不破坏控制语义。 |

### 技术依据

以下官方资料用于核对通信库、MCAP 格式与 C++ API、CMake 及服务管理器的行为；业务 Topic、数据字段、资源假设和安全策略属于本方案的工程设计。查阅日期为 2026年9月23日，开发时应以锁定版本的文档和测试结果为准。

<a id="ref-1"></a>

[1] [ZeroMQ 官方手册  zmq_proxy](https://libzmq.readthedocs.io/en/latest/zmq_proxy.html)

<a id="ref-2"></a>

[2] [ZeroMQ 官方手册  zmq_socket](https://libzmq.readthedocs.io/en/latest/zmq_socket.html)

<a id="ref-3"></a>

[3] [ZeroMQ 官方手册  zmq_setsockopt](https://libzmq.readthedocs.io/en/latest/zmq_setsockopt.html)

<a id="ref-4"></a>

[4] [ZeroMQ 官方手册  zmq_ipc](https://libzmq.readthedocs.io/en/latest/zmq_ipc.html)

<a id="ref-5"></a>

[5] [systemd 官方手册源码  systemd.service](https://github.com/systemd/systemd/blob/main/man/systemd.service.xml)

<a id="ref-6"></a>

[6] [MCAP 官方格式规范](https://mcap.dev/spec)

<a id="ref-7"></a>

[7] [MCAP 官方 C++ API 文档](https://mcap.dev/docs/cpp/)

<a id="ref-8"></a>

[8] [MCAP 官方编码注册表](https://mcap.dev/spec/registry)

<a id="ref-9"></a>

[9] [CMake 官方手册 CMake Presets](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html)

## 当前双臂迁移实现说明

aviator_core 与 manipulator 已按第 15 章职责拆分；原示例的流程、规划和 Demo 在 Core，设备与本地执行在 Manipulator，连续数据经过 aviator_bus。细化协议见 ZMQ 文档第 19 节。

本次交付范围包括原示例的本地开环双臂任务及独立 aviator_hand 的双手开合与实际驱动位置反馈。按当前需求不启用 TCP/关节跟踪偏差及抓握验证准入，软件锁定和手命令接纳都不能解释为实际抓握已验证；这是当前任务模式相对完整系统 CONTROL 准入的明确差异。手关节角/速度、硬件故障寄存器和视觉能力诚实报告为缺失或未验证。设备故障、命令/反馈时效、指令连续性和规划检查独立保留。

100 Hz 窗口传输与 1 ms 本地执行分离，双臂使用共同执行游标；它不提供硬件同步或未经实测的硬实时承诺。aviator_hand 以 50 Hz 接收目标、10 Hz 发布状态，当前 Core 接入已通过模拟通信测试，实体手开合仍需专项验收。全量记录、飞控标定映射和视觉闭环等后续阶段仍需独立实现与验收。
