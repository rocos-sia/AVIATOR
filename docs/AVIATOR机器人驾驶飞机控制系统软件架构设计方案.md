# AVIATOR 机器人驾驶飞机控制系统软件架构设计方案

文档编号  AVIATOR SAD 001    版本  1.1    日期  2026年9月23日

### 架构决策

系统统一采用 C++17 开发，使用 CMake 维护工程、依赖、构建、测试与安装；系统采用多进程与 ZMQ 统一消息总线。独立 aviator_bus 进程通过 XSUB → zmq::proxy() → XPUB 转发全部连续控制和状态消息。各业务进程通过统一 Topic 与 JSON 协议通信，同机优先使用 IPC，远程监控按需通过受限 TCP 出口接入。

AVIATOR Core 负责输入源仲裁、整机状态机、运动目标生成与 FlightState 聚合；Arm Controller 和 Hand Controller 负责设备接入及本地安全执行。实时伺服闭环与 ZMQ 非实时通信域隔离。Monitor、Logger、Plotter 和 Replay 均采用 C++ 实现。独立数据记录节点 aviator_logger 使用 MCAP 统一记录全部机器人数据，覆盖总线消息、原始设备数据、原始图像、伺服周期数据、事件及配置；Replay 从 MCAP 提供复现能力。

### 适用范围

本方案面向软件开发、控制算法、硬件联调和系统测试人员，规定进程边界、通信契约、数据时效性、部署与验证方法，可作为后续通信 ICD 和详细设计的基础。物理操纵限值、飞行阶段相关安全动作、RS422 外部协议及最终硬件参数由对应专项设计冻结。

### 设计基线与评审项

已确定的架构基线包括 C++17、CMake、MCAP 全量数据记录、统一总线、八个主要 Topic、50/100/30 Hz 通信频率、Multipart 两帧格式、JSON、latest-value、sequence、timestamp、watchdog 以及实时隔离。本文补充的超时阈值、队列深度、资源预算与验收数值均为台架初始建议，不代表实测结果或已批准的飞行参数。

### 章节导航

| 章节 | 内容 |
| --- | --- |
| 01—03 | 设计目标与总体架构　进程划分　消息总线设计 |
| 04—08 | Topic 定义　JSON 规范　协议示例　FlightState 结构与示例 |
| 09—12 | 线程与实时性　监测记录回放　资源评估　故障与安全 |
| 13—17 | 启动部署　systemd　目录结构　实施与验收　原则与参考资料 |

## 01 设计目标与总体架构

设计目标是使真实飞控与 USB 飞行摇杆复用同一控制接口，使任意业务环节可被独立监测与记录，并在通信延迟、消息丢失或进程失效时保持明确的控制边界。统一总线简化连接关系，同时也是核心控制通信的单点依赖，必须纳入故障分析。

### 总体数据流

```
真实飞控 ─RS422─ Flight Gateway ─┐
USB摇杆 ─USB─── Joystick Gateway ├─ PUB / SUB ─┐
                               │            │
AVIATOR Core ───────────────────┤            │
Arm Controller ────────────────┤            ▼
Hand Controller ───────────────┤    aviator_bus 独立进程
Camera Detector ───────────────┘    XSUB → proxy → XPUB
                                            │
                        ┌───────────────────┘
                        ▼
              Monitor / Logger / Plotter
                        │
              受限 TCP 监控出口（可选）

Arm / Hand 内部：非实时通信 → 有界快照 → 实时伺服 → 设备
设备/相机/伺服采集 → 独立只读记录通道 → aviator_logger → MCAP
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
| flight_gateway | RS422 ↔ flight.* | 帧校验、百分比归一化、链路时效、状态编码回传。 |
| joystick_gateway | USB → flight.command | 标定、死区、轴映射、使能键与拔出检测；测试配置启用。 |
| aviator_core | 飞控和设备状态 → 设备命令、flight.state | 唯一业务仲裁者；状态机、规划、联合安全判定和整机状态聚合。 |
| arm_controller | arm.command ↔ arm.state | 管理双臂，目标校验、实时插值、设备约束与本地安全。 |
| hand_controller | hand.command ↔ hand.state | 管理双手，抓握控制、驱动映射与本地安全。 |
| camera_detector | camera.command → camera.detection | 图像采集、方向盘检测、置信度与观测时间输出。 |
| aviator_monitor | 订阅所需 Topic | 整机仪表、频率、数据年龄、告警和连接状态。 |
| aviator_logger | 全量总线 Topic + 原始设备/媒体/伺服记录通道 → MCAP | 独立 C++ 数据记录节点；全量采集、异步写盘、分卷、索引、完整性统计与会话管理。 |
| aviator_plotter | 订阅或读取日志 | 趋势曲线、目标与反馈对齐、导出分析。 |
| aviator_replay | 日志 → 隔离总线 | 原速、倍速、单步和算法输入回放。 |

### 唯一控制源规则

flight.command 允许 Flight Gateway 与 Joystick Gateway 使用相同协议，但任意时刻只允许一个获授权输入源影响控制。部署层默认互斥启动；Core 仍须校验 source、publisher_id、session_id 和控制授权，不能以“最后到达者优先”进行源选择。消息中的 source 字段只是身份声明，不构成认证。

控制源取值为 FLIGHT_CONTROLLER、JOYSTICK、TEST、REPLAY、NONE。切换必须先退出 CONTROL、撤销旧授权、清空缓存、检查新源连续有效数据并重新使能；不得在真实飞控掉线后自动切到摇杆。初期可仅支持停止状态下修改配置并重启，后续通过可靠服务完成切换。

双臂、双手各采用一个逻辑生产者，保证 arm.*、hand.* 内含 left 与 right 的一致结构。如未来拆成多个硬件进程，应由明确的聚合者发布统一 Topic，禁止多个设备向同一 Topic 发布无法区分的局部结构。

## 03 消息总线设计

| 项 | 设计规定 |
| --- | --- |
| 发布入口 | ipc:///run/aviator/pub.ipc；Bus 的 XSUB bind，各业务 PUB connect。 |
| 订阅出口 | ipc:///run/aviator/sub.ipc；Bus 的 XPUB bind，各业务 SUB connect。 |
| 总线实现 | 一对 XSUB/XPUB 与阻塞 proxy 转发循环；context 初始配置 1 个 I/O 线程。[1] |
| 订阅规则 | Frame0 使用 ASCII Topic；ZMQ 为前缀匹配，业务接收后必须再做精确匹配。[2] |
| 消息格式 | 恰好两帧：Frame0=Topic，Frame1=UTF-8 紧凑 JSON；整条接收后才解析。 |
| 生命周期 | 设置选项后 bind/connect；单实例锁；SIGTERM 可经控制线程关闭 context 或使用可控代理退出。 |

### latest-value 的实现

连续控制和状态使用“每个 Topic 仅消费最新有效值”的应用层语义。接收线程持续读取完整消息，在校验通过后按 Topic 与授权生产者更新有界 mailbox；算法定时读取快照。不同 Topic 不共用一个最新值槽，防止高频消息覆盖其他类型。

两帧 Multipart 不启用 ZMQ_CONFLATE，因为该选项不支持 Multipart。HWM 仅限制队列，不能把旧消息自动替换为最新消息，也不能保证最新帧必达。[[3]](#ref-3) 初始可设业务 PUB SNDHWM=8、SUB RCVHWM=8、总线两侧 HWM=64，再依据峰值与故障注入调整；数值按 peer 生效，不是按 Topic 的缓存。

每轮接收设置处理数量或时间预算，达到预算就让出执行机会，下一轮继续排空。过载时统计丢帧、覆盖次数和消息年龄；仅在源序号递增且原始数据未过期时刷新 watchdog。不能通过“刚收到”来把积压旧命令重新变成新命令。

### 启动与重连

PUB/SUB 不提供持久历史或执行确认，新订阅者可能错过启动阶段消息。[[2]](#ref-2) 周期发布、就绪状态与连续有效数据检查共同完成启动同步，不依赖固定 sleep。Bus 重启后允许 ZMQ 重新连接，但 Core 和执行器维持安全锁存，等待重新授权。

### 远程监控出口

建议增加可选 telemetry_bridge：本地 SUB 订阅白名单，按需降采样后通过 TCP PUB 提供只读观测。它仍是统一总线的工具订阅者，不形成第二条控制总线。远程端不得接入 XSUB 控制入口；出口使用绑定地址限制、防火墙与 CURVE/ZAP 或已受控隧道，限制客户端数量和带宽。

## 04 Topic 定义与时效预算

表中列出业务主消费端；Monitor、Logger、Plotter 可按权限订阅任意业务 Topic。频率指名义周期发布率，不等于到达期限或硬实时保证。

| Topic 与类型 | Hz / 周期 | 唯一有效生产者 | 业务消费者 |
| --- | --- | --- | --- |
| flight.command<br>FlightCommand | 50 / 20 ms | Flight 或 Joystick Gateway | Core |
| flight.state<br>FlightState | 50 / 20 ms | Core | Flight Gateway；测试端 |
| arm.command<br>ArmCommand | 100 / 10 ms | Core | Arm Controller |
| arm.state<br>ArmState | 100 / 10 ms | Arm Controller | Core |
| hand.command<br>HandCommand | 100 / 10 ms | Core | Hand Controller |
| hand.state<br>HandState | 100 / 10 ms | Hand Controller | Core |
| camera.command<br>CameraCommand | 30 / 33.3 ms | Core | Camera Detector |
| camera.detection<br>CameraDetection | 30 / 33.3 ms | Camera Detector | Core |

### 消息内容约定

flight.command 表示归一化 roll/pitch 连续目标；arm.command 表示双臂位置或轨迹段目标；hand.command 表示双手连续关节或抓握设定值；camera.command 表示可重复覆盖的检测目标、ROI 与跟踪期望。复位、标定、上电、模式切换等边沿触发动作不得混入周期命令。

arm.state、hand.state 包含设备状态与测量快照；camera.detection 包含观测有效性、置信度和方向盘状态；flight.state 是整机对外反馈，不是内部状态的无差别拼接。

| 链路 | 告警 / 超时初始值 | 超时处理 |
| --- | --- | --- |
| flight.command | 60 / 100 ms | Core 撤销飞控目标有效性并触发安全策略。 |
| arm 与 hand 命令或反馈 | 30 / 50 ms | 对应本地 watchdog 或 Core 判定不可继续控制。 |
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
| control_epoch | string | 运动命令必填，匹配本次明确使能授权；旧授权禁止执行。 |

为兼容通用 JSON 工具，整数约束为 0 至 2^53−1；超过范围前创建新会话，禁止静默溢出。timestamp 名称保留此前接口约定，单位固定为 μs。UTC 时钟校正不影响 watchdog；同机用 sample_mono_us 检查数据年龄，再用本地接收单调时间检查消息是否持续到达。

网关重发同一外部样本时可递增消息 sequence，但必须保留原始 sample_mono_us；设备状态同理。Core 派生的运动目标还须携带 origin（输入发布者、会话、序号与采样时间），防止 Core 持续发送旧输入而掩盖上游失效。

### 校验与演进规则

接收端依次检查帧数、Topic、长度、UTF-8、JSON 类型、必填字段、版本、来源、会话、序号、时间与数据范围。建议 Payload 上限 64 KiB、嵌套深度 16，并限制数组长度；拒绝重复键、NaN、Infinity、异常未来时间和未支持的主版本。失败消息只计数与限速记录，不更新控制缓存。

minor 版本只能增加可选字段；删除字段、变更类型、单位或语义需要升级 major。未知枚举和未知控制模式拒绝执行；可忽略未知可选字段。Topic 与 msg_type 一一匹配，协议 Schema 由 C++ 节点与离线分析工具共用。

### 数值与坐标

机械臂和可标定为角度的手关节统一使用 rad、rad/s；位置使用 m；TCP 姿态为 qx、qy、qz、qw 单位四元数。tcp_pose.frame_id 指明参考坐标系，安装外参和关节顺序写入带版本的机器人配置。手部若只支持归一化驱动量，应定义独立字段，禁止将驱动刻度标成 rad。roll/pitch 范围为 [−1,1]，正方向与机械行程映射须经台架标定冻结。

## 06 命令与可靠服务示例

以下为可解析的 JSON 示例，实际发送使用紧凑序列化。Frame0 单独发送 flight.command，Frame1 内容如下。示例中的标识与数值仅用于说明协议。

```json
{
  "msg_type": "FlightCommand", "version": "1.0",
  "sequence": 182736, "timestamp": 1790121600000000,
  "sample_mono_us": 12345678000, "clock_id": "hostA-boot1",
  "publisher_id": "joystick_gateway", "session_id": "joy-session-1",
  "source": "JOYSTICK", "control_epoch": "enable-42",
  "valid": true, "control": {"roll": 0.35, "pitch": -0.12}
}
```

arm.command 的完整头部遵循第05章；下例展示业务扩展字段，合并公共头部后发送。position 模式以固定 joint_names 顺序解释数组，左右臂长度必须匹配配置；指令位置还须通过速度、加速度、工作空间和可达性校验。

```json
{
  "control_epoch": "enable-42", "mode": "JOINT_POSITION",
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

## 07 FlightState 结构与聚合规则

FlightState 由 Core 以 50 Hz 发布，保留 system、arms、hands、vision 四个业务分组及公共头部。其值表示机器人反馈，不能与飞控自身姿态或飞机状态混淆。

| 字段组 | 必需内容 | 约束 |
| --- | --- | --- |
| system | state、current_error_code、last_error_code | 建议同时提供 control_source；错误码为十进制整数。 |
| arms.left / right | joint_position[]、joint_velocity[]、tcp_pose | 两组数组长度相同并匹配机械臂配置。 |
| tcp_pose | frame_id、position{x,y,z}、orientation{qx,qy,qz,qw} | 参考系与外参版本明确，四元数归一化。 |
| hands.left / right | joint_position[]、joint_velocity[] | 有效关节数量与顺序由对应硬件配置定义。 |
| vision | status、confidence、yoke | confidence∈[0,1]；与原始图像采样时间关联。 |
| vision.yoke | detected、roll、pitch | 归一化方向盘状态；无有效检测时位置为 null。 |
| freshness | arm、hand、camera 的 valid、age_ms、sequence | 建议纳入 v1 基线，避免顶层新时间掩盖旧子状态。 |

### 运行状态与错误码

运行状态保留 STANDBY（待机）、GRASPING（抓握中）、FOLLOWING（随动）、CONTROL（主动执行飞控目标）、ERROR（故障），建议补充 INIT、SAFE 和 EMERGENCY_STOP。FOLLOWING 表示经授权的柔顺或目标随动，不执行主动飞控操纵量，具体控制律由算法设计规定。

正常转换为 INIT → STANDBY → GRASPING → FOLLOWING → CONTROL；允许的反向转换须显式定义。进入 CONTROL 要求设备就绪、抓握验证通过、输入源授权有效、必要反馈新鲜、标定有效且无阻断故障。安全条件优先于操作请求，故障恢复不得直接返回 CONTROL。

current_error_code 表示当前最高优先级阻断错误，无当前错误为 0；last_error_code 保留最近一次错误，恢复后不自动清零。详细多故障列表放 system.diagnostic。错误域建议为 0x1xxx 系统、0x2xxx 左臂、0x3xxx 右臂、0x4xxx 左手、0x5xxx 右手、0x6xxx 视觉、0x7xxx 通信。

### 数据一致性

Core 在同一聚合周期读取各 Topic 快照并记录各自序号和年龄；50/100/30 Hz 数据并非同时采样。重复的视觉帧不伪装成新观测。任何子数据失效时保留最后值用于显示但将对应 valid=false，顶层 valid 按当前模式所需数据计算；零值不能代表未知。需要严格同步的算法应使用独立时间对齐模块及明确插值规则。

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
    "left": {"joint_position":[0.12,0.35,0.42,0.31,0.26,0.18],
             "joint_velocity":[0,0.01,0.02,0.01,0,0]},
    "right":{"joint_position":[0.12,0.35,0.42,0.31,0.26,0.18],
             "joint_velocity":[0,0.01,0.02,0.01,0,0]}
  },
  "vision":{"status":"TRACKING","confidence":0.96,
            "yoke":{"detected":true,"roll":0.32,"pitch":-0.15}},
  "freshness": {
    "arm":{"valid":true,"age_ms":4,"sequence":20470},
    "hand":{"valid":true,"age_ms":5,"sequence":20466},
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
| Hand Controller | 通信；设备控制；100 Hz 状态发送 | 设备更新频率服从硬件能力，不把发布 100 Hz 等同于硬件真实反馈 100 Hz。 |
| Camera Detector | 采集；推理；命令与结果通信 | 最新图像优先；推理有界队列；携带图像采集时间而非仅推理完成时间。 |
| Gateway 与工具 | 设备 I/O 或 SUB；编码或 UI 或磁盘工作线程 | 阻塞 I/O 与业务处理分离，队列有界，慢端不占用控制线程。 |

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

100 Hz 目标由本地插值器在伺服周期执行，不允许直接阶跃至远端目标。RT 独立检查本地命令年龄和上游 origin 年龄，即使通信线程卡住也能触发安全状态。若没有新有效目标，执行已验证的有界安全动作，不能无限保持旧速度或持续推进旧轨迹。

记录实际周期、执行耗时、唤醒延迟和 deadline miss。1 kHz 只是目标配置，是否满足最坏情况时间约束必须在目标硬件、驱动与最大干扰负载下验证。

## 10 Monitor Logger 与 Replay

### Monitor 与 Plotter

Monitor 默认只读，使用订阅线程持续收取消息，UI 按 10—20 Hz 刷新最新快照，显示控制源、运行状态、当前及历史错误码、双臂双手状态、视觉置信度与方向盘位置。通信面板同时显示接收频率、原始数据年龄、序号缺口、会话重启和无效消息计数。

连接正常不等于业务健康；显示过期数据时必须明确标记 STALE 和年龄。Plotter 使用有界环形历史，按时间戳对齐指令、接受目标与反馈，显示采样间隙并保留 min/max 降采样，不能用平滑曲线掩盖缺失数据。

### 数据记录节点 aviator_logger（C++ / MCAP）

aviator_logger 是独立部署的基础节点，统一使用 MCAP 作为机器人数据归档格式，不再以 JSONL 或自定义分块文件作为主记录格式。使用 MCAP 官方 C++ 库封装 Writer/Reader，供 Logger、Replay 和 Plotter 共用。[[7]](#ref-7) 本节中的采集路径、数据命名、队列和文件管理策略均为 AVIATOR 工程约定。

#### 全量记录范围

“全部机器人数据”指本次配置启用的全部设备与软件数据源，在采集源原始频率下记录每个样本，禁止默认降采样或 latest-value 覆盖。建立 recording_manifest，列明每个数据源的生产者、Schema、单位、采样率、时钟域、启用状态及预计带宽；新增数据源必须同步注册采集与验收项。未安装或硬件不提供的数据显式标记 unavailable，不能将 100 Hz 聚合反馈当作全部原始反馈。

| 数据类别 | 必须记录的内容 | 采集路径 |
| --- | --- | --- |
| 控制与状态 | 八个主 Topic、system.state、system.diagnostic、system.event 及全部已注册扩展 Topic | Logger 订阅本地总线全部 Topic，逐条保存收到的消息。 |
| 设备原始输入输出 | RS422 收发字节及校验结果、USB 原始报告、驱动实际提供的关节位置/速度/力矩/电流/温度/故障、传感器数据 | 设备适配层在解析或聚合前后提供带方向和样本标识的记录副本。 |
| 实时控制数据 | 每伺服周期的输入目标、插值目标、实际下发量、反馈、限幅/安全判定、周期耗时和 deadline miss | RT 写预分配有界 SPSC 环形队列，非实时采集线程取出并传输；不只记录 100 Hz 状态。 |
| 视觉与媒体 | 每次成功采集的原始图像、相机参数、帧号、曝光/采样时间、检测结果及其关联帧号；已配置的深度/点云 | 相机采集侧先提供记录副本，再进入 latest-frame 推理队列；媒体走独立记录通道。 |
| 服务与事件 | 可靠服务完整请求、响应、执行结果、授权变化、状态转换、故障与恢复 | 服务两端通过记录适配器采集；system.event 摘要不能替代完整服务记录。 |
| 运行上下文 | 构建哈希、依赖清单、协议 Schema、设备清单、配置、标定、坐标变换及其版本、时钟同步状态 | 会话开始保存快照，运行中变更保存新版本及生效时间；剔除密码、密钥等凭据。 |

原始媒体和高频伺服数据不经过控制 XSUB/XPUB，不受业务 JSON 的 64 KiB 上限约束，也不以 Base64 图像挤占控制总线。使用独立 IPC 记录入口（如 ipc:///run/aviator/record.ipc），由非实时适配器经有界队列发送版本化二进制记录信封；信封至少包含数据源、类型/版本、源会话、样本序号、采样时钟及负载长度。原始图像字节随信封进入 MCAP；共享内存仅作为传输优化时必须明确缓冲区所有权和释放确认，文件不能仅保存运行期共享内存句柄。该通道只承载记录副本，不接受执行器控制指令。

#### MCAP 数据映射

MCAP 提供 Schema、Channel、带时间戳的 Message、Metadata、Attachment 及索引结构；Message 的时间单位为纳秒，sequence 为 uint32。[[6]](#ref-6) 本项目规定以下映射：

| MCAP 对象 | AVIATOR 映射规定 |
| --- | --- |
| Schema | JSON 消息使用类型名与协议版本命名，encoding=jsonschema，内容为对应 JSON Schema；二进制记录信封使用冻结的 Protobuf Schema，并嵌入含依赖的描述符。 |
| Channel | 总线消息保留原 Topic；按 Topic、Schema 版本、publisher_id、session_id 区分 Channel。原始数据使用 record.raw.*、record.servo.*、record.camera.* 命名。 |
| Message.data | 合法 JSON 消息直接保存原始 Frame1 字节，message_encoding=json；媒体与原始设备数据使用 message_encoding=protobuf 的记录信封，内含原始字节及解码参数。 |
| Message.log_time | Logger 接收时的 Unix UTC 纳秒；另在 record.ingest 保存接收单调时间、clock_id、全局接收顺序及源标识，用于时钟跳变时复现。 |
| Message.publish_time | 有明确发布 UTC 时使用该时间；当前 timestamp 是快照生成时间，不能直接冒充发布时间，故缺失发布时刻时取 log_time，采样时间仍保留于原消息。微秒转纳秒使用检查溢出的 uint64 运算。 |
| Message.sequence | 使用源 sequence 的低 32 位；原始完整序号保留在负载或采集信封中。完整性核对使用完整序号与源会话，不能仅凭该字段判断。 |
| Metadata / Attachment | 元数据保存会话 ID、构建与配置哈希、分卷序号及完整性状态；附件保存配置、标定和数据清单快照。 |

json、protobuf 与 jsonschema 的编码名称遵循 MCAP 官方注册表。[[8]](#ref-8) 未识别 Topic、格式错误或不符合 Schema 的消息保存至 record.invalid 的二进制信封，保留 Topic、原始各帧及拒绝原因，不伪装成合法 JSON；超过记录入口安全上限时显式记录丢弃计数和原因。

#### 线程、文件与完整性

接收线程负责总线与原始数据接入、时间戳和有界入队，写盘线程独占 MCAP Writer；压缩、索引及磁盘 I/O 均处于非实时域。按数据类别划分字节配额，防止图像突发耗尽控制与事件的记录队列。Logger 不使用控制端的 latest-value mailbox，也不直接照搬控制端 HWM=8；记录队列深度依据第11章实测峰值设计。

文件使用 session_id/segment_000001.mcap 命名；每卷独立包含 Schema、Channel 和会话上下文。采用 Chunk、消息及 Chunk 索引、CRC，压缩初始选 Zstd；JPEG 等已压缩负载是否再压缩由测量决定。台架初始建议按 1 GiB 或 60 s 先到者分卷，Chunk 目标 4 MiB，均需随最大单条图像和目标磁盘性能调整。写入期间使用 .mcap.partial，成功关闭、持久化并校验后再原子更名，更新含哈希与计数的会话清单。flush 不等于断电持久化，持久化间隔及可接受尾部丢失窗口必须独立配置并测试。

正常关闭时先让生产者结束采集并提交结束序号，Logger 限时排空队列、写完索引和文件尾，再退出。崩溃遗留文件保留原件，恢复工具仅将可校验的完整记录导出为新 MCAP，重建索引并报告丢失区间；不保证恢复未落盘缓存或损坏 Chunk。

全量是采集范围与正常负载验收目标，MCAP 格式不使 ZMQ PUB/SUB 获得可靠传输。按数据源、源会话和完整序号核对生产、接收、写入计数；记录重复、缺口、未知起始区间、溢出和会话中断。生产者开始/结束清单用于发现首尾丢失，不能仅凭中间序号连续就宣称完整。Logger 输出 READY、RECORDING、DEGRADED、ERROR 状态，并发布队列字节数、最老等待时间、写入速率、剩余磁盘与缺口计数；自身状态本地记录并避免订阅回写形成递归。

慢盘或磁盘满时不阻塞控制线程或总线，超限丢弃必须将会话标记 incomplete，并经独立健康通道告警；恢复后补写故障信息。若试验要求故障期间仍可补齐，应增加生产者非实时持久化缓存与可靠补传，以源身份和完整序号去重；实时线程仍只进行有界入队，缓存耗尽须报告缺失。未部署补传能力时不能承诺无损记录。

### Replay 模式与隔离

| 模式 | 处理方式 | 边界 |
| --- | --- | --- |
| 查看回放 | 只读取日志，以原时间轴展示 | 不发布控制消息。 |
| 算法回归 | 仅重放选定输入 Topic，Core 输出另行记录比较 | 独立 IPC 命名空间、模拟执行器；不同时回放历史 Core 输出。 |
| 消息复现 | 复现接收顺序、缺口、延迟、倍速或单步 | 源时间保持为元数据，使用可注入虚拟时钟或重映射时间。 |
| 硬件在环 | 明确选择测试配置与允许的设备 | 显式测试授权、限幅和现场使能；禁止自动切入。 |

Replay 与 Plotter 通过共用 MCAP Reader 按 Topic、时间和源会话读取；原始图像依帧号与检测结果关联，源采样时间用于分析，全局接收顺序用于消息复现。Schema 不兼容或会话存在缺口时显式提示，不能静默补齐数据。

回放端点使用 ipc:///run/aviator-replay/...，默认拒绝生产端点。需要重发时分配新的 session_id 和 sequence，保留 original_header；不得沿用历史 control_epoch。倍速和单步回放必须统一算法时钟，不能一边暂停回放一边误用真实时钟触发全部超时。

## 11 资源开销与性能评估

以下为容量规划假设，单位 kB=1000 字节，不含 ZMQ 封装、内核缓存、JSON 对象和相机图像。实际大小随手部自由度、数值精度和扩展字段变化，应以抓取的紧凑 JSON 为准。

| Topic | 假设 kB/帧 | 频率 Hz | 输入流量 kB/s |
| --- | --- | --- | --- |
| flight.command | 0.6 | 50 | 30 |
| flight.state | 4 | 50 | 200 |
| arm.command | 1.5 | 100 | 150 |
| arm.state | 2 | 100 | 200 |
| hand.command | 1 | 100 | 100 |
| hand.state | 1.5 | 100 | 150 |
| camera.command | 0.5 | 30 | 15 |
| camera.detection | 1 | 30 | 30 |
| 合计 | — | 560 | 875 |

总线输入约 0.875 MB/s，即 7 Mbit/s；若每条消息平均有 4 个接收者，输出约 3.5 MB/s。Bus 入站与出站逻辑数据总量约 4.375 MB/s，不等于内存复制量或实际网卡流量。远端全量订阅约需 7 Mbit/s 纯业务带宽，应另留协议与突发裕量。

Logger 原始有效负载约 3.15 GB/小时、25.2 GB/8小时；按 1.5 倍包装与索引预算为 4.73 GB/小时、37.8 GB/8小时。压缩收益需实测，不预先抵扣容量。上述数字仅覆盖八个主 Topic，不代表全量记录容量。相机原图、点云和伺服数据通过独立记录通道写入 MCAP。

全量原始速率按 R_total = R_topics + Σ(采样率 × 样本字节数) + R_events 估算。例如单路 1920×1080、RGB8、30 Hz 原图约 186.624 MB/s，即 671.85 GB/小时；若伺服每周期样本 2 kB、1 kHz，再增加 2 MB/s、7.2 GB/小时。相机数量、像素格式、实际驱动采样率和信封开销必须纳入预算。默认保留原始像素或无损编码，有损视频不作为原图的等价替代。磁盘持续写入初始按实测峰值至少 1.5 倍预留，容量按试验时长加分卷/索引与保留余量计算，压缩率不得预先假定。

### 内存与 CPU

队列有效负载可按 Σ(HWM × 平均消息大小 × peer 数量)估算。例如 12 条队列、深度 64、平均 2 kB，约 1.54 MB，仅为消息体下限。还需计入双端队列、对象分配、I/O 缓冲和日志队列；64 MiB 仅可作为低速 Topic 队列的台架起点；上述单路原图速率下仅能缓冲约 0.36 s。媒体与伺服队列分别按“峰值字节率 × 允许写盘阻塞时间 + 最大单条记录”计算，并计入复制与 Chunk 工作区。

CPU 占用不能仅凭频率给出百分比。应分别测量 encode/decode、转发、算法、视觉和 UI 耗时；计算 Σ(每秒调用数 × 单次耗时)得到核秒预算，再测突发与长尾。当前通信量不构成改用二进制协议的充分理由，但也不保证指定硬件必然满足时序。

### RS422 必须独立核算

总线能力不代表外部串口能力。按 8N1 假设，4 kB FlightState × 50 Hz × 10 bit/byte 约需 2 Mbit/s 单方向有效串行速率，尚未包含外部帧头与校验。115200 bit/s 在此假设下每 20 ms 仅能承载约 230 字节。必须确认波特率、帧格式及飞控接收能力；必要时网关使用独立二进制 ICD、字段分级或协商频率，不能擅自删减已承诺状态。

## 12 故障检测与安全机制

| 故障场景 | 检测路径 | 规定响应 |
| --- | --- | --- |
| 总线退出或卡死 | 消息年龄、收帧间隔；外部健康检查 | Core 与控制器本地 watchdog 生效；Bus 重启不解除安全锁存。 |
| Core 退出或卡住 | 执行器检测命令与 origin 过期 | 本地停止推进新目标，执行经验证的安全动作。 |
| RS422 断流或摇杆拔出 | Gateway 检测设备样本年龄 | 发布 invalid 或停止有效输出；禁止以重复旧值维持“健康”。 |
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

恢复顺序为故障原因解除、设备自检通过、当前状态与物理位置核对、缓存清理、新会话及新 control_epoch 生效、连续新鲜数据验证、显式重新使能。重连、systemd 重启、错误码清零均不等于允许运动。上次错误保留并记录恢复事件，便于追溯。

## 13 启动流程与部署方案

### 运行配置

提供 flight、joystick、simulation、replay 四种配置。flight 默认只启动 Flight Gateway；joystick 只启动 Joystick Gateway，使用测试限值；simulation 使用虚拟设备；replay 使用独立端点、日志时钟和模拟执行器。配置文件包含端点、源白名单、频率、超时、HWM、设备映射、标定版本和日志路径。

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

程序安装到 /opt/aviator/bin，配置位于 /etc/aviator，IPC 位于 /run/aviator，日志位于 /var/log/aviator。使用专用非 root 服务账户和设备访问组；为串口、USB、EtherCAT 设备分配最小权限。IPC 路径必须由受控目录管理，限制非授权发布者写入。[[4]](#ref-4)

Bus 持有单实例文件锁，并在确认旧实例不存在后处理残留端点；不能只假设 IPC bind 一定会拒绝重复实例。[[4]](#ref-4) 多个业务服务不要各自删除公共运行目录。生产与回放目录、服务目标和配置物理分离。

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

## 15 建议工程目录与模块接口

```
aviator/
  CMakeLists.txt
  CMakePresets.json       # shared configure/build/test presets
  cmake/                 # dependency adapters and toolchains
  apps/
    aviator_bus/          # XSUB XPUB proxy
    flight_gateway/      # RS422 adapter
    joystick_gateway/    # USB input adapter
    aviator_core/        # orchestration and control
    arm_controller/      # dual arm process
    hand_controller/     # dual hand process
    camera_detector/     # vision process
    aviator_logger/      # C++ MCAP recording node
  libs/
    protocol/            # typed DTOs and JSON codecs
    transport/           # PUB SUB and service wrappers
    runtime/             # clock, mailbox, scheduling
    control/             # mapping, planning, state machine
    safety/              # validation, authorization, watchdog
    drivers/             # hardware-specific adapters
    recording/           # MCAP reader/writer, capture adapters, manifest
  tools/
    monitor/
    plotter/
    replay/
  schemas/               # one JSON schema per msg_type/version
  config/
    flight/  joystick/  simulation/  replay/
    robots/              # joints, limits, transforms
  deploy/
    systemd/  udev/  packaging/
  tests/
    protocol/  integration/  fault_injection/  timing/  recording/
  docs/
    architecture/  icd/  safety/  operations/
```

### C++ 与 CMake 工程维护规定

统一采用 C++17、RAII 和强类型接口；实时路径避免异常传播及不可预测分配，错误显式返回。业务算法不依赖 MCAP、ZMQ 或 UI 类型。依赖分层为 protocol/runtime/control/safety/drivers、transport 和 recording；recording 提供采集接口及 MCAP 存储适配，RT 仅依赖预分配采样接口，不调用 Writer。

每个库和进程建立独立 CMake target，使用 target_link_libraries、target_include_directories 和 target_compile_features 表达依赖、头文件路径与 cxx_std_17，不使用全局目录堆叠。MCAP 官方 C++ 实现由项目适配 target 封装并链接 Zstd/LZ4 等实际启用依赖；具体 target 名称与集成方式以锁定源码版本为准。ZMQ/cppzmq、JSON、MCAP、Protobuf、压缩库和硬件 SDK 统一锁定版本与校验值，禁止构建时追踪浮动分支；支持受控依赖缓存与离线构建。

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

apps 负责组装；control 只依赖强类型协议对象、时钟接口与安全接口，不依赖 JSON、ZMQ 或设备 SDK。drivers 不反向调用业务状态机。transport 负责完整消息接收与有界发送接口，protocol 负责序列化、Schema 与单位约定。

### 公共接口约定

Clock 提供 UTC 与 monotonic 时间，Replay 可注入虚拟时间；LatestMailbox<T> 提供有界、线程安全快照；CommandValidator 统一来源、授权、范围与时效检查；StateAggregator 输出 FlightState；SafetySupervisor 负责故障锁存、状态转换和恢复条件。

错误码、枚举、Topic 与 Schema 建立单一来源，生成或校验 C++ 定义与离线分析工具的一致性。配置启动时校验，运行中不静默更改关节顺序、单位、坐标系或安全阈值。测试夹具覆盖正确帧、错误帧、重启、乱序和旧数据。

## 16 实施阶段与验收计划

| 阶段 | 交付内容 | 退出条件 |
| --- | --- | --- |
| P0 接口冻结 | ICD、状态机、配置、硬件映射和安全策略 | 确认 RS422 容量、关节数、坐标系、控制方向与超时预算。 |
| P1 总线与仿真 | Bus、协议库、模拟生产者、Monitor、Logger | C++/CMake 干净构建通过；八 Topic 写入 MCAP 并可读回；启动丢帧、重连、队列限制符合预期。 |
| P2 摇杆台架 | Joystick、Core、双臂双手接入与本地 watchdog | 拔出、源冲突、旧数据和 Core 停止均进入规定安全状态。 |
| P3 真飞控与视觉 | RS422 双向映射、Camera、FlightState 聚合 | 50/100/30 Hz 目标完成测量；原始设备、全频伺服与原图记录覆盖清单完成核对。 |
| P4 回放与部署 | Replay、systemd、发布包和运维手册 | MCAP 隔离回放、分卷、索引、崩溃恢复、慢盘、磁盘满与远程干扰测试通过。 |
| P5 可靠服务与优化 | ACK/REQ-REP 或 ROUTER/DEALER、按实测优化 | 重试去重、结果未知对账及重新授权验证完成。 |

### 建议台架验收基线

对八个 Topic 同时运行至少 8 小时，接入全量 Logger、Monitor、Plotter 和一个远程观测端；记录发布与接收计数、P50/P95/P99 与最大消息年龄、进程 CPU/RSS、磁盘吞吐、控制周期抖动和所有丢弃原因。不同 clock_id 的时间差不得直接当作单向延迟。

建议初始指标：10 s 窗口发布平均频率偏差不超过 ±2%；同机从发布到接收校验完成的 P99 延迟小于 5 ms；正常负载下不得有未解释的有效命令超时。该指标不包含控制算法与机械响应，最终值由系统时序预算评审确定。

硬性功能判据：所有过期、重复、未授权与范围非法命令均不能进入执行；Bus/Core 被终止后，控制器在配置期限内检测并于下一可用伺服周期启动安全处置；重启不自动使能；回放默认不能向生产端点发布。安全动作完成时长另按物理系统预算验收。

实时验收单独统计伺服 deadline miss 和最坏执行时间。在指定测试窗口内零 deadline miss 是必要测试条件，但不能替代最坏情况分析。满 CPU、磁盘抖动、相机高负载、重连风暴及调度延迟都要纳入干扰测试。

### MCAP 全量记录验收

在目标硬件以全部已启用数据源的最高配置速率运行至少 8 小时，包含原始图像与完整伺服采样；按 recording_manifest 对账源端开始/结束序号、发送计数与各卷写入计数，正常负载零缺口、零未说明降采样。逐卷验证 Schema 可解析、CRC、索引检索、原始负载字节一致性、采样时间及图像关联；多卷边界无遗漏、无重复。

注入磁盘降速/满盘、Logger 被终止、Bus 重启、UTC 跳变、生产者重启及强制截断文件，确认缺口和 incomplete 状态可见、完整记录可恢复、队列与内存有界，且控制与伺服时序仍满足预算。单独验证 sequence 超过 uint32 后的完整序号对账，以及启动/退出边界的缺失检测；已配置补传时验证重传去重和缓存耗尽处理。

### 必须冻结的开放参数

RS422 波特率与线协议；双臂和双手自由度及反馈能力；坐标与 roll/pitch 正方向；允许操纵范围及变化率；视觉依赖条件；各模式安全动作与接管方式；最终 watchdog 阈值；可靠服务上线范围；原始数据覆盖清单、相机格式与数量、伺服记录样本结构、峰值记录带宽、持久化间隔、补传需求、日志保留量与远程接入策略。

## 17 关键设计原则与参考资料

### 必须保持的架构约束

| 原则 | 实现约束 |
| --- | --- |
| C++ 与 CMake | 交付节点统一 C++17；target 化依赖、版本锁定、Preset、测试与安装统一维护。 |
| MCAP 全量记录 | 全部启用数据源按源频率记录，逐源对账；异常缺口显式标记，原始大数据不占控制总线。 |
| 统一总线 | 连续业务消息全部经过 aviator_bus；不在模块间添加隐式控制旁路。 |
| 控制权唯一 | 部署互斥与 Core 仲裁双重约束；切换需要退出控制并重新授权。 |
| 最新且有效 | latest-value 按 Topic 管理；新到达不等于新采样，年龄和源会话必须检查。 |
| 可靠性分层 | 周期目标可以覆盖；离散动作有确认、去重、期限和执行结果。 |
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
