# AVIATOR ZMQ 协议格式说明

当前实现（2026-10-02）：节点 `session_id` / `client_session_id` / `server_session_id` 为普通文本启动标记，不再生成会话 UUID，也不用于授权、回包关联或来源固定。旧配置的会话绑定参数兼容但忽略；下文旧版方案中涉及会话匹配的要求以此为准。标记仍用于日志追踪和重启后的序号重置。请求 `request_id` 和 `control_epoch` 继续使用 UUID，发布者、序号、时钟、时效和控制权限检查保留。

文档编号：AVIATOR ICD ZMQ 001

文档版本：0.2（接口设计草案）

日期：2026年10月2日

依据：[AVIATOR 机器人驾驶飞机控制系统软件架构设计方案 v1.3](AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md)

## 1. 范围与约定级别

本文描述 AVIATOR 内部通过 ZeroMQ 传输的消息，包括八个主要业务 Topic、三个观测扩展 Topic、离散可靠服务、独立数据记录通道及回放约定。RS422 帧、USB 报告、机器人驱动总线和相机 SDK 的外部协议不在本文件定义范围内。 RS422 双路冗余外部接口的新增评审稿见 [RS422 通信协议规范](AVIATOR_RS422通信协议规范.md)，该文档不表示串口运行功能已实现。

本文按以下级别区分接口来源：

| 标记 | 含义 |
| --- | --- |
| 架构基线 | 架构方案已经明确的传输方式、字段、语义或约束，本文沿用。 |
| 补充拟定 | 为使协议可以实现而补充的具体字段、枚举、格式和边界规则，需在 ICD 评审后冻结。本文中的“必填”对采用本草案的实现生效，不表示原架构已冻结该字段。 |
| 待冻结 | 依赖硬件、标定、控制算法或部署参数，本文不假设实际设备数值。 |

当前 `common/` 已提供公共编解码和传输实现，部分节点已实现本文协议能力；尚未实现或仍待冻结的能力在对应章节明确标注。JSON 示例中的 `version:"1.0"` 是当前线上消息版本，与本文文档版本不同。

第 2—4 节以架构基线为主；第 5—12 节逐项说明主 Topic；第 13—16 节为扩展、服务、记录和回放；第 17—18 节给出接收检查与冻结清单。未在架构中明确的具体字段结构均属于补充拟定。

### 1.1 节点名称统一

以架构第 15 章的最终目录与进程划分为准：

- `flight_gateway` 根据配置接入 RS422 或 USB 摇杆；`source` 区分输入来源，不另设 `joystick_gateway` 进程。
- `manipulator` 发布 `arm.state` 并消费 `arm.command`，负责双臂设备接入与本地执行；不发布 `hand.state`，也不订阅 `hand.command`。
- `aviator_hand` 消费 `hand.command`、发布 `hand.state`，通过 SocketCAN 独立接入双手；默认 `publisher_id=inspire_hand`。
- `camera` 负责图像采集与检测。
- `aviator_core` 是连续设备目标的唯一业务生产者和控制源仲裁者。

示例使用上述进程名作为 `publisher_id`。多实例场景必须配置稳定且唯一的实例标识，并注册白名单。

## 2. 通道与线格式

### 2.1 端点

| 通道 | 默认端点 | bind 方 / socket | connect 方 / socket | 负载 |
| --- | --- | --- | --- | --- |
| 生产发布入口 | `tcp://127.0.0.1:5555` | aviator_bus / XSUB | 业务节点 / PUB | Topic + JSON |
| 生产订阅出口 | `tcp://127.0.0.1:5556` | aviator_bus / XPUB | 业务节点、工具 / SUB | Topic + JSON |
| 独立记录入口 | `tcp://127.0.0.1:5557` | aviator_logger / PULL（补充拟定） | 非实时记录适配器 / PUSH（补充拟定） | 单帧 Protobuf 信封，见第 15 节 |
| 回放发布入口 | `tcp://127.0.0.1:6555` | 隔离 aviator_bus / XSUB | aviator_replay、测试节点 / PUB | Topic + JSON |
| 回放订阅出口 | `tcp://127.0.0.1:6556` | 隔离 aviator_bus / XPUB | 仿真节点、工具 / SUB | Topic + JSON |
| 离散可靠服务 | 配置指定，无架构默认端口 | 服务端 / REP 或 ROUTER | 客户端 / REQ 或 DEALER | 见第 14 节 |
| 飞控高层操作（待实现） | `tcp://127.0.0.1:5559`（建议） | aviator_core / ROUTER | flight_gateway / DEALER | ServiceRequest / ServiceReply，见第 14.5 节 |
| 服务记录确认补传（待实现） | `tcp://127.0.0.1:5560`（建议） | aviator_logger / ROUTER | 非实时记录适配器 / DEALER | ServiceAuditRecord / RecordAck，见第 15.5 节 |

前三个生产端口和回放端口来自架构基线；记录通道的 PUSH/PULL 选择为补充拟定。地址均由配置统一维护。远程观测通过只读出口，不向远程客户端开放控制发布入口。

### 2.2 业务总线：恰好两帧

```text
ZMQ Multipart Message
┌──────────────────────────────────────────────────────────┐
│ Frame0：ASCII Topic，例如 arm.command                    │
│ Frame1：UTF-8 JSON 对象，例如 {"msg_type":"ArmCommand",…} │
└──────────────────────────────────────────────────────────┘
```

| 项目 | 规定 |
| --- | --- |
| Frame0 | 区分大小写；无引号、空格、换行、BOM 或尾随 NUL；与注册 Topic 字节精确相同。 |
| Frame1 | 单个 JSON object，UTF-8，无 BOM、尾随 NUL 或额外 JSON 文本；线上紧凑序列化，本文为阅读而换行。 |
| 帧边界 | 发送 Frame0 时设置 `sndmore`，发送 Frame1 时结束消息；接收完整消息后检查恰好两帧。 |
| 订阅 | ZMQ 使用前缀匹配，业务层必须二次精确匹配。例如订阅 `arm.state` 不能接纳 `arm.state.debug`。 |
| 大小 | JSON 负载上限初始采用 65,536 字节，嵌套深度上限 16，均来自架构建议；Topic 长度上限补充拟定为 128 字节。 |
| 禁止内容 | 重复 JSON 键、NaN、Infinity、无穷数值转换、注释和尾逗号；数组及字符串按 Schema/配置设置界限。 |
| 自定义帧头 | 业务消息不增加长度头、CRC、请求 ID 帧、空分隔帧或二进制图像帧。长度与消息边界由 ZMQ 提供。 |

收到了第三帧时，必须有界地排空或丢弃整条非法 Multipart，不能将剩余帧误当下一条消息。传输层还须限制异常消息资源占用，不能等巨大负载全部分配后才检查。

发送示意（C++17 / cppzmq，省略错误处理）：

```cpp
// pub 已 connect 到 XSUB 入口，且由当前线程独占。
pub.send(zmq::buffer(topic), zmq::send_flags::sndmore);
pub.send(zmq::buffer(json_payload), zmq::send_flags::none);
```

### 2.3 交付语义

PUB/SUB 无持久历史，也不提供业务执行确认。连续目标周期性覆盖，接收端按 `(Topic, 授权生产者)` 保存最新有效快照；各 Topic 的槽独立。不得启用 `ZMQ_CONFLATE` 处理两帧消息。HWM 只限制排队量，不能保证只保留最新值或最新值必达。

控制消费端只处理最新值；Logger 应逐条记录所有收到的消息，不使用 latest-value 覆盖。消息到达、通过校验、被控制器接纳、产生物理动作是不同阶段。

## 3. Topic 总表

| Topic / msg_type | 生产者 | 主消费者 | 名义频率 | 告警 / 超时初始值 |
| --- | --- | --- | --- | --- |
| `flight.command` / `FlightCommand` | flight_gateway | aviator_core | 50 Hz / 20 ms | 60 / 100 ms |
| `flight.state` / `FlightState` | aviator_core | flight_gateway | 50 Hz / 20 ms | 60 / 100 ms |
| `arm.command` / `ArmCommand` | aviator_core | manipulator | 100 Hz / 10 ms | 30 / 50 ms |
| `arm.state` / `ArmState` | manipulator | aviator_core | 100 Hz / 10 ms | 30 / 50 ms |
| `hand.command` / `HandCommand` | aviator_core | aviator_hand | 50 Hz / 20 ms | 命令 watchdog 100 ms |
| `hand.state` / `HandState` | aviator_hand | aviator_core | 10 Hz / 100 ms | 设备反馈有效期 300 ms |
| `camera.command` / `CameraCommand` | aviator_core | camera | 30 Hz / 约 33.3 ms | 架构未定义；部署配置冻结 |
| `camera.detection` / `CameraDetection` | camera | aviator_core | 30 Hz / 约 33.3 ms | 100 / 200 ms |
| `system.state` / `SystemState` | 各节点，仅报告自身 | 观测工具 | 1—10 Hz | 按节点健康策略配置 |
| `system.diagnostic` / `SystemDiagnostic` | 各节点，仅报告自身 | Core、观测工具 | 1—10 Hz | 按诊断策略配置 |
| `system.event` / `SystemEvent` | 事件所属节点 | 观测工具、Logger | 事件触发 | 无送达保证 |

八个主 Topic 的名称、类型与频率来自架构。后三个 Topic 名称及频率为架构建议，其具体消息模型由本文补充。Monitor、Logger、Plotter 可按权限订阅业务 Topic。上述时限是台架初始配置，不是硬实时或飞行参数承诺；频率不等于硬件实际采样频率。

## 4. 公共模型

### 4.1 类型表示与字段必填性

`uint53` 表示 JSON 整数 `0..9007199254740991`，序号等字段可进一步限制最小值为 1。`number` 必须能解码为有限数值；`string` 区分大小写；`T|null` 表示显式允许空值。下文字段表使用 `[]` 表示数组，点号表示嵌套路径。

公共头部位于根对象，不存在额外 `header` 或 `payload` 包装。各 Topic 的业务字段与公共字段并列。除标为“可选”或“条件必填”的字段外，字段均必填。不得把数值、布尔值或数组编码成字符串。

### 4.2 公共头部（八个主 Topic 与 system.* 共用）

| 字段 | 类型 / 约束 | 含义 |
| --- | --- | --- |
| `msg_type` | string | 与 Topic 总表一一对应的 PascalCase 类型。 |
| `version` | string | `major.minor`；本草案示例为 `1.0`。 |
| `sequence` | uint53，≥1 | 在 `(publisher_id, session_id, Topic)` 内严格递增，从 1 开始；允许缺口。 |
| `timestamp` | uint53，µs | 本条快照生成时刻，Unix UTC 微秒；不是必然的实际发送时刻。 |
| `sample_mono_us` | uint53，µs | 原始采样或输入接纳时刻，Linux CLOCK_MONOTONIC 微秒。 |
| `clock_id` | string，非空 | 主机与启动标识，同一主机本次启动的进程共享同一时钟域标识。 |
| `publisher_id` | string，非空 | 配置登记的稳定生产者标识。JSON 声明不是身份认证凭据。 |
| `session_id` | string，非空 | 每次进程启动生成普通文本标记（时间/进程号）；用于记录与序号重置，不校验 UUID 格式，不作为会话授权条件。 |
| `valid` | boolean | 业务数据是否有效；false 不能作为有效控制输入，也不刷新有效数据 watchdog。 |
| `source` | string，FlightCommand 必填 | 见第 5 节；其他消息不得依赖此字段判断飞控来源。 |
| `config_id` | string，可选，补充拟定 | 设备映射、关节顺序、标定和坐标配置的版本或哈希；存在时须与已加载配置一致。 |
| `original_header` | object，可选 | 仅回放使用，见第 16 节。 |

UUID 拟统一采用小写带连字符文本。一般标识字符串拟限制为 1—128 个 UTF-8 字节，版本最长 16 字节；自然语言说明最长 1024 字节。专用路径和图像参数限制由对应 Schema 规定。

整数达到上界前，应停止该序列并创建新会话，禁止回绕。新会话必须重新校验授权，不能通过换会话跳过恢复流程。

### 4.3 时间、有效性与序号

```text
原始数据年龄（同 clock_id）= now_mono_us - sample_mono_us
输入来源年龄（同 clock_id）= now_mono_us - origin.sample_mono_us
有效消息中断时长 = now_mono_us - last_accepted_receive_mono_us
```

计算前先检查未来时间和范围，避免无符号下溢。未来容差、告警与超时阈值从统一配置读取；超时边界补充拟定为 `age >= timeout`。跨 `clock_id` 不能直接相减，须使用已经验证的时钟映射及误差预算；未配置映射的控制输入拒绝用于执行。UTC 时间用于关联日志，不用于 watchdog。

网关重发同一设备样本、Manipulator 重发旧反馈、Camera 重发同一检测时，允许增加消息 `sequence`，但必须保留原始 `sample_mono_us`。Core 的设备命令头部记录此次目标生成/接纳时刻，同时保留 `origin`；FlightState 头部表示本次聚合时刻，同时保留各分组 `freshness`。当前 `aviator_hand` 是明确例外：`hand.state` 顶层 `sample_mono_us` 是状态生成时刻，每侧 `hands.{side}.sample_mono_us` 才是反馈快照最早读请求时间，消费者必须使用每侧时间或 `feedback_age_ms` 判断反馈年龄。其他发送线程重复发送旧计算结果不能改写其采样时刻。

同一已接纳会话内，`sequence <= 已处理序号` 的消息按重复/乱序丢弃。应区分“结构和身份合法的已处理序号”与“有效控制数据”：`valid=false` 可推进已处理序号并立即标记输入失效，但不覆盖最后有效数值、不刷新有效期限。格式错误、未授权等消息不能推进授权序列状态。不要在多个会话之间按最大序号竞争；旧会话退役后不重新接纳。

### 4.4 origin：派生目标的输入溯源

`arm.command`、`hand.command` 携带架构规定的 `origin`：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `origin.publisher_id` | string | Core 采用的输入消息生产者。 |
| `origin.session_id` | string | 输入启动标记，仅追踪，不参与授权匹配。 |
| `origin.sequence` | uint53，≥1 | 输入消息序号。 |
| `origin.sample_mono_us` | uint53 | 原始输入采样时刻，µs。 |
| `origin.clock_id` | string | 输入时钟域。 |
| `origin.topic` | string，可选，补充拟定 | 默认 `flight.command`；其他来源须单独注册其授权和时效策略。 |

持续依据同一输入计算目标时，`origin` 不随输出序号刷新。本文完整设备命令示例面向飞控连续目标；抓握准备、标定运动等其他目标源的溯源策略仍需专项冻结，不得伪造新鲜 FlightCommand。

### 4.5 单位、数组与空值

| 数据 | 单位 / 表示 |
| --- | --- |
| 机械臂及可标定手关节位置 / 速度 | rad / rad/s；顺序由版本化配置中的 `joint_names` 决定。 |
| TCP 位置 | m；`position:{x,y,z}`。 |
| TCP 姿态 | 单位四元数 `orientation:{qx,qy,qz,qw}`；范数容差由配置定义。 |
| 坐标系 | `tcp_pose.frame_id` 必填；例如 `robot_base`，变换与标定版本须可追溯。 |
| 飞控 / 视觉 roll、pitch | 归一化量 `[-1,1]`，不是角度；正方向与行程映射待标定冻结。 |
| 置信度 | `[0,1]`；执行阈值由视觉依赖策略指定，不因值非零就视为可靠。 |
| 图像 ROI | 像素，左上角为原点，x 向右、y 向下；见第 11 节。 |
| 错误码 | uint53，线上十进制；0 表示无错误。 |

本文件示例采用双 7 关节臂、双 6 关节手，仅展示形状。左右设备可有不同自由度，分别匹配配置；单侧的位置与速度数组长度相同。完整快照不是差量更新，缺字段或少发一侧不能解释为“维持原值”。

状态类消息：失效后可保留最后测量值供显示，并将对应 `valid=false`；从未获得测量时，本文补充允许该组测量数组或 pose 整体为 `null`，但不允许有效状态出现 `null`，也不允许数组中混入 null。设备子状态、`freshness` 或父级 valid 明确限定可用性。视觉检测无效时 roll/pitch 必须为 null。

命令类消息：`valid=true` 时所选模式的所有目标必填且非空。一般 `valid=false` 消息仍保持业务结构完整并绝不可执行；当前 `hand.command` 的已认证失效通知允许省略 `hands`，见第 8 节。无初始目标时可以停止发布，由未就绪状态和 watchdog 阻止执行，不构造伪造零目标。

## 5. flight.command — FlightCommand

### 5.1 字段

公共头部之外的结构沿用架构：

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `source` | enum string | 本文拟定 `FLIGHT`（真实飞控）、`JOYSTICK`（USB 摇杆）；仿真也须使用测试配置授权的来源。 |
| `control` | object | 完整连续目标。 |
| `control.roll` | number | `[-1,1]`。 |
| `control.pitch` | number | `[-1,1]`。 |

Core 同时校验部署模式、publisher/session 白名单和 source。到达顺序不能切换控制权。摇杆失联或外部样本过期后发布 invalid 或停止有效输出；禁止用新时间重发旧值延长有效期。切换来源通过可靠服务或受控本地流程完成。

当前 JOYSTICK 位置保持扩展（需消费者显式启用，默认拒绝）：

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `input_state.mode` | string | 固定 POSITION_HOLD，仅 JOYSTICK flight.command 可用。 |
| `input_state.device_connected` | bool | 内核设备接口检查状态；false 时头部 valid 必须为 false。 |
| `input_state.checked_mono_us` | uint53 | 最近一次成功设备状态检查时间，同一 clock_id；不能由重发消息刷新。 |

原始 sample_mono_us 始终保留。valid=true 时必须已经采集过完整事件帧，且 checked_mono_us >= sample_mono_us > 0。位置保持允许原始轴事件较旧，独立检查设备检查年龄和接收中断时长（默认均 100 ms），拒绝未来或倒退时间、重复序号和其他会话。有效位失效立即撤销可用性，即使设备检查时间已旧；新序号不能使旧设备检查重新有效。普通消息保留原始采样时效规则。设备查询只能证明内核接口可用，不能证明设备固件持续产出新报告；不是硬件心跳。该扩展明确允许健康设备的静止位置保持，不允许改写原始采样时间伪装新事件。


### 5.2 完整 Frame1 示例

Frame0：`flight.command`。

```json
{
  "msg_type": "FlightCommand",
  "version": "1.0",
  "sequence": 182736,
  "timestamp": 1790121600000000,
  "sample_mono_us": 12345678000,
  "clock_id": "hostA-boot1",
  "publisher_id": "flight_gateway",
  "session_id": "11111111-1111-4111-8111-111111111111",
  "valid": true,
  "source": "JOYSTICK",
  "control": {"roll": 0.35, "pitch": -0.12}
}
```

## 6. arm.command — ArmCommand

### 6.1 字段

| 字段 | 类型 | 规则 / 来源 |
| --- | --- | --- |
| `mode` | enum string | `JOINT_POSITION` 为架构示例；`JOINT_TRAJECTORY` 为补充拟定的可选能力。 |
| `origin` | object | 第 4.4 节定义，架构基线。 |
| `control_epoch` | UUID string | 第 4.5 节定义，补充拟定。 |
| `arms` | object | 同时包含 `left`、`right`。 |
| `arms.{side}.joint_position` | number[] | JOINT_POSITION 必填；该侧所有关节位置，rad。 |
| `arms.{side}.points` | object[] | 仅 JOINT_TRAJECTORY 模式使用。 |

有效命令必须通过关节限位、目标变化率、速度、加速度、工作空间、可达性、源年龄与授权检查。双臂命令整体接纳，任一侧非法时整条拒绝，禁止只执行一侧。命令不隐含上电、使能、模式状态机切换或故障复位。

### 6.2 完整位置命令示例

Frame0：`arm.command`。

```json
{
  "msg_type": "ArmCommand",
  "version": "1.0",
  "sequence": 20470,
  "timestamp": 1790121600001000,
  "sample_mono_us": 12345679000,
  "clock_id": "hostA-boot1",
  "publisher_id": "aviator_core",
  "session_id": "22222222-2222-4222-8222-222222222222",
  "valid": true,
  "control_epoch": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
  "mode": "JOINT_POSITION",
  "origin": {
    "publisher_id": "flight_gateway",
    "session_id": "11111111-1111-4111-8111-111111111111",
    "sequence": 182736,
    "sample_mono_us": 12345678000,
    "clock_id": "hostA-boot1"
  },
  "arms": {
    "left": {"joint_position": [0.1, 0.2, 0.0, -0.3, 0.0, 0.2, 0.0]},
    "right": {"joint_position": [-0.1, 0.2, 0.0, -0.3, 0.0, 0.2, 0.0]}
  }
}
```

### 6.3 可选轨迹段格式（补充拟定，未启用时拒绝）

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `trajectory_start_mono_us` | uint53 | 根字段；轨迹段起点，与消息 `clock_id` 同域。 |
| `arms.{side}.points[].time_from_start_us` | uint53 | 相对起点的微秒数；第一点为 0，后续严格递增。 |
| `arms.{side}.points[].joint_position` | number[] | 每点完整关节位置，rad。 |
| `arms.{side}.points[].joint_velocity` | number[] | 每点完整关节速度，rad/s。 |

每侧点数拟限制为 2—32，左右点数与时间网格相同；不得同时带 `joint_position` 单点字段。最大时间跨度、允许前瞻、接纳时起点容差及插值算法须在能力配置中冻结。轨迹替换整个未执行段，不是追加队列；接纳时检查位置和速度连续性。播放中的源数据和本地命令仍须满足 watchdog，轨迹结束时间不能延长授权。未完成这些约定前仅实现 JOINT_POSITION。

**当前 Core/Manipulator 实现约定（覆盖上段拟定方案）：** `JOINT_TRAJECTORY` 使用 `execution=SYNCHRONIZED_TICKS`、`trajectory_id`、`first_tick`、`total_ticks`、`trajectory_start_mono_us` 和 `wheel_reference`。普通离线轨迹为 2 ms 网格、每臂最多 32 点，执行端补出中间 1 ms 点。Servo 带 `streaming=true` 和 `finished`，每臂最多 51 个原始 1 ms 点，每点额外包含七轴 `joint_acceleration`（rad/s²）；速度为规划导数（rad/s），不以差分值替代。左右臂点数和时基必须一致，整个流的初始及最终静止点导数为零，窗口/规划块边界继承导数。连续追加保持同一轨迹 ID，`total_ticks` 只增不减，已提交区间的 q/dq/ddq 不允许改写；替换网络窗口时检查重叠样本。`finished=true` 表示已经提交至静止终点，未结束的流发生缓冲耗尽则报错。Core 本地队列不超过 250 ms，全部窗口仍受相同的 origin/command watchdog 约束。升级此扩展须同时重新编译 Core 和 Manipulator。

## 7. arm.state — ArmState

### 7.1 字段（测量字段沿用架构，设备状态结构为补充拟定）

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `arms.left`、`arms.right` | object | 双臂完整反馈。 |
| `arms.{side}.valid` | boolean | 本侧测量是否有效。 |
| `arms.{side}.status` | enum string | 拟定 `OFFLINE / INITIALIZING / READY / ACTIVE / SAFE / ERROR / EMERGENCY_STOP`。 |
| `arms.{side}.enabled` | boolean | 驱动执行使能状态，不表示可以忽略 Core 授权。 |
| `arms.{side}.error_code` | uint53 | 当前本侧最高优先级错误，0 表示无错误。 |
| `arms.{side}.joint_position` | number[] 或 null | rad，空值规则见第 4.6 节。 |
| `arms.{side}.joint_velocity` | number[] 或 null | rad/s。 |
| `arms.{side}.tcp_pose` | object 或 null | `frame_id`、`position`、`orientation`，结构见示例。 |
| `arms.{side}.sample_mono_us` | uint53 或 null | 补充拟定；每侧原始反馈时刻，未采集过时为 null。 |
| `accepted_command` | object 或 null，可选 | 最近接纳命令引用，结构见下文。 |

顶层 `valid` 拟定义为两侧测量均有效；设备故障与测量有效性分别表达，因此 valid=true 不能代替 status/enabled 判断。头部 `sample_mono_us` 为双侧已获取样本中较早的时刻，valid=true 时两侧均须存在样本；从未获取任何样本时使用此次状态生成时刻并标 valid=false。每侧年龄还须独立检查，不能因另一侧更新而刷新旧侧数据。

`accepted_command` 拟包含 `publisher_id`、`session_id`、`sequence`、`control_epoch`，表示最近被接纳进执行缓存的目标，不保证已完成或已到位。无已接纳命令时为 null；如需到位判断，由测量和控制状态判定。

### 7.2 完整 Frame1 示例

Frame0：`arm.state`。

```json
{
  "msg_type": "ArmState", "version": "1.0", "sequence": 20470,
  "timestamp": 1790121600002000, "sample_mono_us": 12345676000,
  "clock_id": "hostA-boot1", "publisher_id": "manipulator",
  "session_id": "33333333-3333-4333-8333-333333333333", "valid": true,
  "arms": {
    "left": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "sample_mono_us": 12345676000,
      "joint_position": [0.12, -0.32, 0.54, 0.22, -0.11, 0.67, 0.31],
      "joint_velocity": [0.01, 0.02, -0.01, 0, 0.01, 0.03, -0.01],
      "tcp_pose": {
        "frame_id": "robot_base", "position": {"x": 0.423, "y": 0.182, "z": 0.631},
        "orientation": {"qx": 0, "qy": 0, "qz": 0.70710678, "qw": 0.70710678}
      }
    },
    "right": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "sample_mono_us": 12345676000,
      "joint_position": [-0.12, 0.32, -0.54, -0.22, 0.11, -0.67, -0.31],
      "joint_velocity": [0.01, 0.02, -0.01, 0, 0.01, 0.03, -0.01],
      "tcp_pose": {
        "frame_id": "robot_base", "position": {"x": 0.423, "y": -0.182, "z": 0.631},
        "orientation": {"qx": 0, "qy": 0, "qz": -0.70710678, "qw": 0.70710678}
      }
    }
  }
}
```

## 8. hand.command — HandCommand

### 8.1 当前字段与模式

当前 `aviator_hand` 只接受配置声明支持的 `NORMALIZED_POSITION` 和 `GRASP_SETPOINT`。六路顺序固定为 `[拇指旋转, 拇指, 食指, 中指, 无名指, 小指]`；驱动刻度 1000 表示张开、0 表示闭合，不是 rad。

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `mode` | enum string | 当前为 `NORMALIZED_POSITION / GRASP_SETPOINT`。 |
| `origin` | object | 第 4.4 节；其时钟域必须与手节点一致。 |
| `control_epoch` | UUID string | 当前授权代次。 |
| `hands.left`、`hands.right` | object | `valid=true` 时双手完整目标必填，任一侧非法时整条拒绝。 |
| `hands.{side}.drive_position_normalized` | number[6] | NORMALIZED_POSITION 必填，每项 `[0,1]`；`raw[i]=round(value×1000)`。 |
| `hands.{side}.grasp.closure` | number | GRASP_SETPOINT 必填，`[0,1]`；`raw[i]=round((1-closure)×1000)`。 |

两种目标互斥且均无插值。当前 GRASP_SETPOINT 不携带 `profile_id`，同一 closure 映射到该侧全部六路。有效命令必须先完整校验双手，再进行任何 CAN 写入；CAN 写入不是双手原子事务。首条成功下发的命令绑定 publisher、session、epoch 及 origin 身份，后续身份变化、非递增序号、时钟不匹配或超过默认 100 ms 的消息/origin 均拒绝。

新鲜且身份匹配的 `valid=false` 命令用于立即请求 `safe_pose`，仍须带公共头、`control_epoch`、`mode` 和 `origin`，但允许省略 `hands`。停止发布时同一 100 ms watchdog 触发安全姿态；安全写入成功只表示已提交到 SocketCAN，不证明实体手已到位。

### 8.2 完整有效 Frame1 示例

Frame0：`hand.command`。

```json
{
  "msg_type": "HandCommand", "version": "1.0", "sequence": 20466,
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
    "left": {"drive_position_normalized": [1, 1, 1, 1, 1, 1]},
    "right": {"drive_position_normalized": [1, 1, 0.8, 1, 1, 1]}
  }
}
```

## 9. hand.state — HandState

### 9.1 顶层字段

`aviator_hand` 默认以 10 Hz 发布状态，反馈采集目标频率也是每侧完整六路 10 Hz。顶层公共 `sample_mono_us` 是本次状态生成时刻，不是设备反馈采样时刻。Manipulator 仍以 100 Hz 发布旧形状兼容占位：顶层始终 `valid=false`，每侧只有 OFFLINE、null 关节量等基础字段；Core 未启用真实手链路时可将其用于显示未知状态，但不得将其解释为本节定义的有效实际反馈。

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `valid` | boolean | 左右手反馈均在默认 300 ms 有效期内；与命令是否有效无关。 |
| `command_valid` | boolean | 最近命令仍满足接收、消息采样和 origin 三项 100 ms watchdog。 |
| `feedback_only` | boolean | true 表示只读模式，节点忽略命令且不发送速度、力、位置或退出安全姿态写入。 |
| `hands.left`、`hands.right` | object | 双手完整反馈与诊断状态。 |
| `accepted_command` | object | 最近通过校验并提交的命令引用；启动后尚无命令时各字符串为空、sequence/sample 为 0。 |
| `accepted_command.publisher_id/session_id` | string | 最近命令发布者和会话。 |
| `accepted_command.sequence/sample_mono_us` | uint53 | 最近命令序号和命令头部采样时间；用于接收确认，不表示物理到位。 |

### 9.2 每侧字段

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `valid` / `feedback_available` | boolean | 该侧已形成完整六路快照、没有当前 I/O 失败且反馈年龄小于 300 ms。 |
| `status` | enum string | `OFFLINE` 从未获得完整反馈；`STALE` 反馈过期或 I/O 失效；反馈新鲜时按命令状态为 `READY` 或 `ACTIVE`。 |
| `enabled` | boolean | 当前等于软件 `command_valid`，不是设备硬件使能反馈。 |
| `error_code` | uint53 | 当前固定为 0；尚未采集硬件错误寄存器，不能据此断言硬件无故障。 |
| `position_source` | string | 当前固定为 `angle_act_register`。 |
| `sample_mono_us` | uint53 或 null | 本次完整快照最早 CAN 读请求的主机单调时间；从未收到完整快照时为 null。 |
| `sample_time_basis` | string | 当前固定为 `host_read_request`，不是设备硬件采样时间。 |
| `feedback_age_ms` | number 或 null | 状态生成时刻减去每侧快照时间；从未有快照或时间不可比较时为 null。 |
| `drive_position_raw` | integer[6] 或 null | ANGLE_ACT 最近完整快照，逐项 `[0,1000]`；过期时保留最后值，从未收到时为 null。 |
| `drive_position_normalized` | number[6] 或空数组 | raw 除以 1000；从未收到完整快照时为 `[]`。 |
| `commanded_drive_position_normalized` | number[] | 上次成功下发目标的归一化回显；无目标时为空数组。 |
| `joint_position/joint_velocity` | null | 当前没有驱动刻度到 rad/rad/s 的标定，不用零值或驱动刻度冒充关节量。 |
| `grasp_verified` | boolean | 当前固定为 false；命令接纳或 closure 到达不能替代独立抓握验证。 |
| `feedback_samples` | uint53 | 本会话完成的六路快照数。 |
| `feedback_timeouts` | uint53 | 单寄存器响应超时计数。 |
| `feedback_io_errors` | uint53 | CAN 读请求或接收异常计数。 |
| `feedback_last_error` | string | 最近 I/O 错误文本；成功形成新快照后清空。 |

每侧最多保留一个未完成读请求，六路全部收齐后才原子替换快照，不混合不同轮次。CAN 没有事务序号，无法完全区分跨轮次延迟的同寄存器响应。Core 判断 hand 状态时同时检查顶层状态接收年龄、每侧 `sample_mono_us`、`valid` 和确认字段。

### 9.3 RH56FTP Modbus TCP 后端差异

`rh56ftp_hand` 使用相同的 `HandState` 信封。顶层 `valid` 是所有**已配置**手的每侧
`valid` 的逻辑与：反馈新鲜、最近读取无 I/O 错误且六路位置在 `[0,1000]` 内。
未配置侧仍发布 `valid=false`，但不参与顶层汇总；反馈恢复后顶层自动恢复 true。
`hold_control_error` 为独立的故障锁存字符串（无故障时为空），不再影响测量 `valid`；
Core 单独检查该字段，非空时禁止以该反馈确认动作完成。锁存保护仍需重连/重启解除，
有效反馈不表示运动保护已解除。设备故障码非零时每侧 `status=ERROR`。

每侧仅发布 `drive_position_raw`、`drive_position_normalized`、`force`、`current`、
`current_register_raw`、`error_codes`、`status_codes`、`temperature` 这些测量数组，
均按 AVIATOR 六路顺序。`current` 为有符号电流，`current_register_raw` 为原始无符号字；
`temperature` 单位为 ℃。`error_code` 保留为六路故障码最大值供 Monitor 汇总，
具体故障位须查看 `error_codes`，不将最大值解释为按位合并结果。
未配置或过期时测量数组为 `[]`。每侧采样基准为 `host_modbus_read`；
尚未接纳命令时 `accepted_command=null`，接纳后另含 `control_epoch`。

已删除重复字段 `angle`、`angle_raw`、`err`、`error`、`status_code`、`status_values`、`temp`，
以及重复顶层 `command_valid` 的 `enabled` 和始终为 null 的 `joint_position/joint_velocity`。
外部消费者应迁移到保留字段；`simulation` 的手状态同步删除上述别名及 `enabled`，
仍保留其能够计算的仿真关节量。CAN 后端 `aviator_hand` 的字段不变。
握持、故障保护与反馈计数等诊断字段见 [RH56FTP 节点说明](../nodes/rh56ftp_hand/README.md)。

### 9.4 完整 Frame1 示例（CAN 后端）

Frame0：`hand.state`。

```json
{
  "msg_type": "HandState", "version": "1.0", "sequence": 20466,
  "timestamp": 1790121600002000, "sample_mono_us": 12345680000,
  "clock_id": "hostA-boot1", "publisher_id": "inspire_hand",
  "session_id": "33333333-3333-4333-8333-333333333333", "valid": true,
  "command_valid": true, "feedback_only": false,
  "hands": {
    "left": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "feedback_available": true, "position_source": "angle_act_register",
      "sample_mono_us": 12345675000, "sample_time_basis": "host_read_request",
      "feedback_age_ms": 5,
      "drive_position_raw": [1000, 1000, 1000, 1000, 1000, 1000],
      "drive_position_normalized": [1, 1, 1, 1, 1, 1],
      "commanded_drive_position_normalized": [1, 1, 1, 1, 1, 1],
      "joint_position": null, "joint_velocity": null, "grasp_verified": false,
      "feedback_samples": 100, "feedback_timeouts": 0,
      "feedback_io_errors": 0, "feedback_last_error": ""
    },
    "right": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "feedback_available": true, "position_source": "angle_act_register",
      "sample_mono_us": 12345675000, "sample_time_basis": "host_read_request",
      "feedback_age_ms": 5,
      "drive_position_raw": [1000, 1000, 800, 1000, 1000, 1000],
      "drive_position_normalized": [1, 1, 0.8, 1, 1, 1],
      "commanded_drive_position_normalized": [1, 1, 0.8, 1, 1, 1],
      "joint_position": null, "joint_velocity": null, "grasp_verified": false,
      "feedback_samples": 100, "feedback_timeouts": 0,
      "feedback_io_errors": 0, "feedback_last_error": ""
    }
  },
  "accepted_command": {
    "publisher_id": "aviator_core",
    "session_id": "22222222-2222-4222-8222-222222222222",
    "sequence": 20466, "sample_mono_us": 12345679000
  }
}
```

## 10. flight.state — FlightState

### 10.1 字段与聚合语义

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `system.state` | enum string | `INIT / STANDBY / GRASPING / FOLLOWING / CONTROL / SAFE / ERROR / EMERGENCY_STOP`；其中 INIT、SAFE、EMERGENCY_STOP 来自架构建议。 |
| `system.control_source` | enum string | `FLIGHT / JOYSTICK / NONE`；NONE 为本文补充，表示当前无授权输入源。 |
| `system.current_error_code` | uint53 | 当前最高优先级阻断错误，无错误为 0。 |
| `system.last_error_code` | uint53 | 最近一次错误，恢复后不自动清零，无历史错误为 0。 |
| `arms.{side}.joint_position/joint_velocity/tcp_pose` | 同 ArmState 测量字段 | 必须保留双臂；设备状态字段可选附带。 |
| `hands.{side}` | 同 HandState 每侧字段 | 当前直接保留 `aviator_hand` 的归一化驱动反馈、null 关节量、状态和诊断字段。 |
| `hand_control` | object，可选 | Core 启用真实手连接时附带 `enabled/error/target_version`，表示 Core 手控制链路状态，不替代设备反馈。 |
| `vision.status` | enum string | 同 CameraDetection。 |
| `vision.confidence` | number | `[0,1]`。 |
| `vision.yoke` | object | detected、roll、pitch，同 CameraDetection。 |
| `freshness.arm/hand/camera` | object | 本文采纳架构建议，作为本草案必填项。 |
| `freshness.{group}.valid` | boolean | 聚合时该组的业务有效性及年龄是否满足当前要求。 |
| `freshness.{group}.age_ms` | number 或 null | 聚合时原始样本年龄，ms，可带小数；无样本或无法映射时钟时为 null 且 valid=false。 |
| `freshness.{group}.sequence` | uint53 或 null | 最近采纳的源消息序号；从未收到时为 null。 |
| `freshness.{group}.publisher_id/session_id/clock_id` | string，可选 | 补充拟定；建议附带以消除跨会话歧义。 |
| `freshness.{group}.sample_mono_us` | uint53，可选 | 补充拟定；直接复核该组年龄。 |

顶层时间是聚合时间；顶层 valid 按当前模式所必需的反馈、输入和条件计算。视觉可选的模式允许 `freshness.camera.valid=false` 而顶层仍有效；依赖视觉的模式不允许。臂组年龄按较旧侧样本保守计算。启用真实手链路时，手组 `freshness.age_ms` 使用 hand.state 顶层生成时间计算，同时以每侧 `valid/sample_mono_us` 检查实际反馈年龄；因此组 age_ms 不能替代每侧 `feedback_age_ms`。Core 还以独立配置的状态接收期限检查整个 hand 链路，当前默认 500 ms；它不能延长设备侧 300 ms 有效反馈。未启用真实手链路时，Core 可聚合 Manipulator 的 invalid 占位，`freshness.hand` 保持无效。子组过期时保留最后值用于显示，但其 freshness.valid=false。

消费者收到 FlightState 后必须考虑传输和本地驻留时间，不能永久使用发送时的 `age_ms`。同机可用 `age_ms + (now_mono_us - 头部 sample_mono_us)/1000` 估计当前子状态年龄，或直接用子组采样时间；跨时钟域须有映射。

状态含义沿用架构：FOLLOWING 为经授权的柔顺或目标随动，CONTROL 为主动执行飞控目标。错误域建议为系统 `0x1xxx`、左/右臂 `0x2xxx/0x3xxx`、左/右手 `0x4xxx/0x5xxx`、视觉 `0x6xxx`、通信 `0x7xxx`；线上发送十进制整数。具体错误码表待冻结。

### 10.2 完整 Frame1 示例

Frame0：`flight.state`。在聚合时刻 `12345680000` µs，臂、手、视觉年龄分别为 4、5、21 ms。

```json
{
  "msg_type": "FlightState", "version": "1.0", "sequence": 10235,
  "timestamp": 1790121600002000, "sample_mono_us": 12345680000,
  "clock_id": "hostA-boot1", "publisher_id": "aviator_core",
  "session_id": "22222222-2222-4222-8222-222222222222", "valid": true,
  "system": {
    "state": "CONTROL", "control_source": "JOYSTICK",
    "current_error_code": 0, "last_error_code": 8194
  },
  "arms": {
    "left": {
      "joint_position": [0.12, -0.32, 0.54, 0.22, -0.11, 0.67, 0.31],
      "joint_velocity": [0.01, 0.02, -0.01, 0, 0.01, 0.03, -0.01],
      "tcp_pose": {
        "frame_id": "robot_base", "position": {"x": 0.423, "y": 0.182, "z": 0.631},
        "orientation": {"qx": 0, "qy": 0, "qz": 0.70710678, "qw": 0.70710678}
      }
    },
    "right": {
      "joint_position": [-0.12, 0.32, -0.54, -0.22, 0.11, -0.67, -0.31],
      "joint_velocity": [0.01, 0.02, -0.01, 0, 0.01, 0.03, -0.01],
      "tcp_pose": {
        "frame_id": "robot_base", "position": {"x": 0.423, "y": -0.182, "z": 0.631},
        "orientation": {"qx": 0, "qy": 0, "qz": -0.70710678, "qw": 0.70710678}
      }
    }
  },
  "hands": {
    "left": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "feedback_available": true, "position_source": "angle_act_register",
      "sample_mono_us": 12345675000, "sample_time_basis": "host_read_request",
      "feedback_age_ms": 5, "drive_position_raw": [1000, 1000, 1000, 1000, 1000, 1000],
      "drive_position_normalized": [1, 1, 1, 1, 1, 1],
      "commanded_drive_position_normalized": [1, 1, 1, 1, 1, 1],
      "joint_position": null, "joint_velocity": null, "grasp_verified": false,
      "feedback_samples": 100, "feedback_timeouts": 0,
      "feedback_io_errors": 0, "feedback_last_error": ""
    },
    "right": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "feedback_available": true, "position_source": "angle_act_register",
      "sample_mono_us": 12345675000, "sample_time_basis": "host_read_request",
      "feedback_age_ms": 5, "drive_position_raw": [1000, 1000, 800, 1000, 1000, 1000],
      "drive_position_normalized": [1, 1, 0.8, 1, 1, 1],
      "commanded_drive_position_normalized": [1, 1, 0.8, 1, 1, 1],
      "joint_position": null, "joint_velocity": null, "grasp_verified": false,
      "feedback_samples": 100, "feedback_timeouts": 0,
      "feedback_io_errors": 0, "feedback_last_error": ""
    }
  },
  "hand_control": {"enabled": true, "error": "", "target_version": 3},
  "vision": {
    "status": "TRACKING", "confidence": 0.96,
    "yoke": {"detected": true, "roll": 0.32, "pitch": -0.15}
  },
  "freshness": {
    "arm": {"valid": true, "age_ms": 4, "sequence": 20470},
    "hand": {"valid": true, "age_ms": 0, "sequence": 20466},
    "camera": {"valid": true, "age_ms": 21, "sequence": 6141}
  }
}
```

## 11. camera.command — CameraCommand

### 11.1 字段（补充拟定）

相机命令仅描述可反复覆盖的检测期望，不承载“拍一次”“重置跟踪器”“标定一次”等边沿动作。

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `camera_id` | string | 已配置相机标识；本草案每个 camera.command 对应一个授权相机流。 |
| `target` | enum string | 本草案仅 `YOKE`，表示飞机操纵方向盘检测。 |
| `tracking_enabled` | boolean | 持续跟踪期望；false 为持续关闭检测期望，不是进程停止或硬件断电。 |
| `roi` | object 或 null | null 表示全图；非空时包含下述整数。 |
| `roi.x/roi.y` | uint53 | 原始采集图像上的左上角像素坐标。 |
| `roi.width/roi.height` | uint53，≥1 | 像素宽高。覆盖范围为 `[x,x+width) × [y,y+height)`，不能越过图像边界。 |
| `min_confidence` | number | `[0,1]`，当前检测输出的最小接受阈值，仍须满足配置的安全下限。 |

检测算法内部缩放、裁剪或畸变校正不能改变线上 ROI 的原始像素坐标定义。多相机独立控制需扩展注册 Topic 或明确按 camera_id 分槽，不能在当前单槽协议中交错覆盖。该能力待冻结。

相机命令超时阈值须显式配置；本草案拟在命令过期时停止将结果标为可用于控制，报告诊断。诊断采集是否继续由配置决定，不能默认永久延续过期跟踪期望。

### 11.2 完整 Frame1 示例

Frame0：`camera.command`。

```json
{
  "msg_type": "CameraCommand", "version": "1.0", "sequence": 6000,
  "timestamp": 1790121599970000, "sample_mono_us": 12345648000,
  "clock_id": "hostA-boot1", "publisher_id": "aviator_core",
  "session_id": "22222222-2222-4222-8222-222222222222", "valid": true,
  "camera_id": "cockpit_camera", "target": "YOKE", "tracking_enabled": true,
  "roi": {"x": 320, "y": 180, "width": 1280, "height": 720},
  "min_confidence": 0.8
}
```

## 12. camera.detection — CameraDetection

### 12.1 字段

`status`、`confidence`、`yoke` 沿用架构对视觉状态的定义，根级布局、图像关联字段和命令引用为补充拟定。

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `camera_id` | string | 对应配置中的相机流。 |
| `frame_id` | uint53 或 null | 在相机生产者 session 内的采集帧号，从 1 开始；无任何图像时为 null。与 tcp_pose.frame_id 的坐标系含义不同。 |
| `image_width/image_height` | uint53，≥1 | 原始图像宽高，像素；未采到图像时使用已校验的配置尺寸。 |
| `status` | enum string | `OFFLINE / INITIALIZING / SEARCHING / TRACKING / LOST / ERROR`。 |
| `confidence` | number | `[0,1]`，无候选时为 0；有候选但阈值不足时可保留实际分数。 |
| `yoke.detected` | boolean | 是否存在通过当前接受条件的有效方向盘观测。 |
| `yoke.roll/yoke.pitch` | number 或 null | 有效检测时为 `[-1,1]`；其他情况下均为 null。 |
| `command_ref` | object 或 null，可选 | 使用的相机命令的 `publisher_id/session_id/sequence`，用于追踪 ROI 与检测条件。 |

头部 `sample_mono_us` 使用主机接收 frameset 的单调时刻，timestamp 使用检测快照生成时刻。当前图像 MCAP 使用 Logger 的 `aviator.record.v1.CameraPacket`，以 `(publisher_id, session_id, camera_id, frame_id)` 与检测消息关联；推理队列可以跳帧，不要求检测 frame_id 连续。重复发布同一帧时不得改变 frame_id 或采样时间。

当前 Python 相机节点按 `config/camera.yaml` 的 `detector.type` 单选 `charuco` 或 `apriltag`，在自由 JSON body 中补充 `detector`；AprilTag 识别到配置 ID 时另附 `tag_id` 和原始 `decision_margin`。两种模式均复用相同的公共头部、`status/valid` 与 `pose`，`confidence` 为各自的启发式质量指示，不应解释为概率。

Python 相机节点同时补充 `steering_wheel` 对象，读取 `config/steering_wheel_calibration.yaml`
的零位与轴，使用 `T_current @ inverse(T_zero)` 计算运动；原始 `pose` 不变。
此物理单位扩展与上述归一化 `yoke.roll/pitch` 分别定义：

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `steering_wheel.valid` | boolean | 当前已加载有效标定、来源匹配且码位姿可计算；独立于顶层检测 valid。 |
| `steering_wheel.reason` | string | 有效时为空；否则为 disabled、calibration_unavailable、target_not_tracking、calibration_source_mismatch 或 invalid_target_pose。 |
| `steering_wheel.theta_rad` | number 或 null | 相对零位、绕存储轴的带符号角度，rad，范围 `[-pi,pi]`；无效时 null。 |
| `steering_wheel.translation_along_axis_m` | number 或 null | 相对零位沿存储轴的带符号平移，m；无效时 null。 |
| `steering_wheel.translation_vector_m` | array[3] 或 null | 相对变换的相机坐标系平移项，m；无效时 null。 |
| `steering_wheel.axis_direction/axis_frame` | array[3]/string 或 null | 使用的单位轴向量及轴坐标系。 |
| `steering_wheel.axis_error_rad/axis_match` | number/boolean 或 null | 旋转至少 3° 时独立估计轴并以 5° 夹角判断；轴不匹配时角度不能视为真实转角。 |
| `steering_wheel.calibration_id` | string 或 null | 使用的标定内容摘要；未加载标定时 null。 |

派生量与原始位姿共用同一帧身份和采样时间，丢码时不复用历史数值。
标定文件约每秒检查并按变更重载；派生量无效不阻断原始码位姿发布与重新标定。

控制可用时须同时满足顶层 valid=true、status=TRACKING、detected=true、置信度达标以及年龄合格。SEARCHING/LOST 等状态可正常上报，但 valid=false、detected=false、roll/pitch=null。未获取任何图像的状态报告使用生成时刻并标 valid=false。

### 12.2 完整有效检测示例

Frame0：`camera.detection`。

```json
{
  "msg_type": "CameraDetection", "version": "1.0", "sequence": 6141,
  "timestamp": 1790121599999000, "sample_mono_us": 12345659000,
  "clock_id": "hostA-boot1", "publisher_id": "camera",
  "session_id": "44444444-4444-4444-8444-444444444444", "valid": true,
  "camera_id": "cockpit_camera", "frame_id": 9001,
  "image_width": 1920, "image_height": 1080,
  "status": "TRACKING", "confidence": 0.96,
  "yoke": {"detected": true, "roll": 0.32, "pitch": -0.15},
  "command_ref": {
    "publisher_id": "aviator_core",
    "session_id": "22222222-2222-4222-8222-222222222222", "sequence": 6000
  }
}
```

### 12.3 完整失效检测示例

```json
{
  "msg_type": "CameraDetection", "version": "1.0", "sequence": 6142,
  "timestamp": 1790121600032000, "sample_mono_us": 12345692000,
  "clock_id": "hostA-boot1", "publisher_id": "camera",
  "session_id": "44444444-4444-4444-8444-444444444444", "valid": false,
  "camera_id": "cockpit_camera", "frame_id": 9002,
  "image_width": 1920, "image_height": 1080,
  "status": "LOST", "confidence": 0.2,
  "yoke": {"detected": false, "roll": null, "pitch": null}
}
```

## 13. system.* 观测扩展协议（补充拟定）

三个 Topic 共用第 4.2 节头部，生产者仅声明自身状态。消费者按 `(Topic, publisher_id)` 分槽，否则多个节点的诊断会互相覆盖；system.event 为逐条事件，不采用 latest-value。此处示例为业务字段片段，发送时必须合并公共头部。

### 13.1 system.state / SystemState

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `node` | string | 与 publisher_id 相同。 |
| `lifecycle` | enum string | `STARTING / READY / RUNNING / DEGRADED / STOPPING / ERROR`。 |
| `ready` | boolean | 本节点已具备其业务职责需要的条件。 |
| `uptime_ms` | uint53 | 本会话运行时长。 |
| `recording` | object，Logger 条件必填 | Logger 的记录状态与健康指标。 |
| `recording.state` | enum string | 架构规定的 `READY / RECORDING / DEGRADED / ERROR`。 |
| `recording.queue_bytes/oldest_wait_ms/write_bytes_per_sec/free_disk_bytes/gap_count` | uint53 / number | 队列字节、最老等待毫秒、写入字节每秒、剩余空间字节、缺口计数；均非负。 |
| `recording.incomplete` | boolean | 当前记录会话已出现无法确认完整的缺失。 |

```json
{
  "node": "aviator_logger", "lifecycle": "RUNNING", "ready": true, "uptime_ms": 60000,
  "recording": {
    "state": "RECORDING", "queue_bytes": 4096, "oldest_wait_ms": 2,
    "write_bytes_per_sec": 200000000, "free_disk_bytes": 900000000000,
    "gap_count": 0, "incomplete": false
  }
}
```

system.state 的 valid 表示状态快照本身可信；lifecycle=ERROR 时仍可 valid=true。是否健康由业务状态决定。Bus 不在转发循环解析业务，其健康状态可由独立健康线程或监护逻辑报告。

### 13.2 system.diagnostic / SystemDiagnostic

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `diagnostics` | object[] | 当前诊断快照，最多 128 项（拟定）；空数组表示当前无诊断项。 |
| `diagnostics[].code` | uint53 | 冻结错误码表中的错误/告警代码。 |
| `diagnostics[].severity` | enum string | `INFO / WARN / ERROR / FATAL`。 |
| `diagnostics[].component` | string | 本进程内组件标识，例如 `arms.left`。 |
| `diagnostics[].active/latched` | boolean | 当前是否存在 / 是否锁存。 |
| `diagnostics[].message` | string | 人可读说明，不参与控制分支判断。 |
| `diagnostics[].count` | uint53 | 本会话内该诊断累计发生次数。 |
| `diagnostics[].first_timestamp/last_timestamp` | uint53 | 首次/最近发生 UTC 微秒。 |
| `truncated` | boolean | 诊断条目超限时为 true，并保留最高优先级项。 |

```json
{
  "diagnostics": [{
    "code": 28673, "severity": "ERROR", "component": "arm.command",
    "active": true, "latched": true, "message": "命令数据过期",
    "count": 1, "first_timestamp": 1790121600100000, "last_timestamp": 1790121600100000
  }],
  "truncated": false
}
```

示例 28673（0x7001）仅演示通信错误域，具体含义须由错误码表冻结。低频诊断不能替代控制 watchdog。

### 13.3 system.event / SystemEvent

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `event_id` | UUID string | 一次事件的唯一标识。 |
| `event_type` | enum string | 拟定 `STATE_CHANGED / AUTH_CHANGED / FAULT_RAISED / FAULT_CLEARED / SERVICE_UPDATE / RECORDING_GAP`。 |
| `severity` | enum string | 同 diagnostic。 |
| `request_id` | UUID string 或 null | 与服务相关时填写请求标识，其他情况为 null。 |
| `details` | object | 按 event_type 解释，不能反向作为执行命令。 |

拟定 details 字段：STATE_CHANGED 使用 `from/to/reason`；AUTH_CHANGED 使用 `source/control_epoch/action`（action 为 GRANTED/REVOKED）；FAULT_* 使用 `component/error_code`；SERVICE_UPDATE 使用 `operation/target/status/error_code`；RECORDING_GAP 使用 `source_id/session_id/first_missing_sequence/last_missing_sequence`，未知边界用 null。各事件 Schema 单独约束，不接受无限制任意对象。

```json
{
  "event_id": "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
  "event_type": "STATE_CHANGED", "severity": "INFO", "request_id": null,
  "details": {"from": "INIT", "to": "STANDBY", "reason": "SELF_CHECK_PASSED"}
}
```

事件可能丢失或重复，订阅者不能因未收到故障事件而认定无故障。关键服务完整请求与结果同时走记录适配器；事件摘要不能替代可靠响应或完整审计记录。

## 14. 离散可靠服务协议（补充拟定）

### 14.1 边界与帧结构

架构列举 enable、reset_fault、set_source、calibrate，明确后续使用独立服务端点。本节为通用服务信封设计；第 19 节设备服务已有实现，第 14.5 节飞控高层服务待实现。简单同步部署选择 REQ/REP；需要异步进度与并发时选择 DEALER/ROUTER，不在同一端点混用两套帧规则。

| 模式 | 客户端应用发送/接收 | 服务端应用接收/发送 |
| --- | --- | --- |
| REQ/REP | 单帧 UTF-8 ServiceRequest / 单帧 ServiceReply | 单帧 ServiceRequest / 单帧 ServiceReply；底层信封由 socket 处理。 |
| DEALER/ROUTER | 单帧 ServiceRequest / 单帧 ServiceReply | ROUTER 接收 `[routing_id][JSON]`，发送 `[原 routing_id][JSON]`。 |

这里没有业务 Topic 帧，不经 XSUB/XPUB，也不复用业务两帧格式。ROUTER routing_id 是不透明路由字节，不是身份认证信息；本文 DEALER 协议不添加空分隔帧。服务 JSON 的编码、大小、整数范围与重复键规则沿用业务 JSON。

### 14.2 ServiceRequest

服务信封独立于业务公共头部，不强制带业务 `sequence/valid`；请求身份与期限使用以下字段：

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `msg_type/version` | string | `ServiceRequest` / `1.0`。 |
| `request_id` | UUID string | 一次逻辑请求 ID，重试保持不变。 |
| `client_id/client_session_id` | string / UUID string | 客户端稳定标识及启动会话。 |
| `timestamp` | uint53 | 首次构造请求的 UTC 微秒，重试保持不变。 |
| `issued_mono_us/clock_id` | uint53 / string | 首次请求的单调时刻及域，重试保持不变。 |
| `deadline_ms` | uint53，≥1 | 相对 issued_mono_us 的总有效期，不是每次重试后重新起算。上限由服务配置约束。 |
| `operation` | enum string | 下表注册操作。 |
| `target` | string | 目标节点或受控组件路径，必须来自服务注册表。 |
| `parameters` | object | 操作参数，不需要参数时为 `{}`。 |

| operation | target 示例 | parameters / 语义 |
| --- | --- | --- |
| `enable` | `aviator_core` | `{}`；请求协调显式使能，前置条件通过后产生新 epoch。不能绕过 Core 直接授权手臂。 |
| `reset_fault` | `manipulator/arms.left` | `{}`；复位仍需故障原因解除，不隐含使能。 |
| `set_impedance_profile` | `manipulator` | 已实现的 Core 设备服务：`config_id`、`profile`（`default` 或 `following`）；双臂静止保持时切换配置中的关节刚度，返回 `profile/stiffness/target`。Core 使用 5000 ms 期限，重复 request_id 不重复执行。 |
| `set_source` | `aviator_core` | `source` 为 FLIGHT 或 JOYSTICK；撤销旧授权并重新验证。 |
| `calibrate` | `camera` 或注册设备组件 | `profile_id` 指定已批准标定流程；未冻结的流程拒绝执行。 |
| `get_result` | 原服务节点 | `original_request_id/original_client_session_id`；查询同一 client_id 的原请求结果。查询本身使用新的 request_id。 |
| `cancel` | 原服务节点 | `original_request_id/original_client_session_id`；仅支持可取消阶段，取消不等于物理回滚。 |

后两种操作为本文补充的可靠结果查询与取消方案。不同 operation 的 parameters 必须按独立 Schema 校验；未知参数不能悄悄改变操作语义。

```json
{
  "msg_type": "ServiceRequest", "version": "1.0",
  "request_id": "cccccccc-cccc-4ccc-8ccc-cccccccccccc",
  "client_id": "maintenance_console",
  "client_session_id": "55555555-5555-4555-8555-555555555555",
  "timestamp": 1790121600000000, "issued_mono_us": 12345678000,
  "clock_id": "hostA-boot1", "deadline_ms": 1000,
  "operation": "reset_fault", "target": "manipulator/arms.left", "parameters": {}
}
```

设备阻抗切换的观测扩展：`arm.state.execution.impedance_switching` 表示正在重配置，`impedance_profile` 表示最近成功应用的配置（初始为 `default`）。暂停 RT 时消息 `valid=false`，关节样本时间戳不刷新；独立的 `status_mono_us` 表示设备服务仍在运行。Core 仅在本地发起的有界切换任务中允许使用该状态心跳维持资源就绪，不能将旧关节样本作为实时反馈。新轨迹必须等待切换完成，任一臂失败停止双臂。Managed `start_control` 返回 COMPLETED 并直接进入 CONTROL（表示状态转换完成），不创建额外的 CONTROL 状态机任务。默认刚度恢复期间 `settled=false`，暂不接收运动目标；恢复并确认反馈后放行原有 Servo 流程，只接受恢复后的新鲜输入。主线程仍处理输入、服务与保护，迟到的恢复结果不能解除 SAFE／ERROR／急停。阻抗切换复用连接和 RT 控制器，仅暂停与恢复运动循环，不重新初始化 RT。

### 14.3 ServiceReply

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `msg_type/version` | string | `ServiceReply` / `1.0`。 |
| `request_id/client_id/client_session_id` | 同请求 | 原样关联；ROUTER 路由标识不能替代它们。 |
| `server_id/server_session_id` | string / UUID string | 服务端身份与启动会话。 |
| `timestamp` | uint53 | 此响应生成 UTC 微秒。 |
| `status` | enum string | `ACCEPTED / RUNNING / COMPLETED / REJECTED / FAILED / CANCELLED / EXPIRED / UNKNOWN`。 |
| `error_code` | uint53 | 无错误为 0；错误码含义由注册表冻结。 |
| `message` | string，可选 | 人可读说明。 |
| `result` | object | 操作结果，无字段时 `{}`；enable 成功时可返回 control_epoch。 |

ACCEPTED 仅表示已受理，RUNNING 表示执行中，COMPLETED 才表示确认完成。REJECTED 表示未受理；FAILED 表示已确认失败；CANCELLED 表示取消已确认且必要安全收尾完成；EXPIRED 仅用于期限内未开始的请求。已开始的动作不能单因客户端超时就报告“没有执行”。UNKNOWN 表示无法判定既有请求的最终结果，客户端须对账。

```json
{
  "msg_type": "ServiceReply", "version": "1.0",
  "request_id": "cccccccc-cccc-4ccc-8ccc-cccccccccccc",
  "client_id": "maintenance_console",
  "client_session_id": "55555555-5555-4555-8555-555555555555",
  "server_id": "manipulator",
  "server_session_id": "33333333-3333-4333-8333-333333333333",
  "timestamp": 1790121600010000,
  "status": "COMPLETED", "error_code": 0, "result": {}
}
```

REQ/REP 一次请求只能有一次响应：短操作可直接返回终态；长操作先返回 ACCEPTED，后续通过独立 get_result 请求查询。get_result 外层响应对应查询自身，`result` 包含原请求的 `request_id/status/error_code/result`。不得在同一次 REP 事务发送 ACCEPTED 后再主动发送第二个 COMPLETED。DEALER/ROUTER 可按同一请求关联发送多个进度响应，客户端仍须能查询遗漏结果。

### 14.4 去重、期限与恢复

服务端按 `(client_id, client_session_id, request_id)` 去重，校验重试的 operation、target、参数、原始发起时间与期限完全一致；同 ID 内容不同则拒绝。重试返回已知状态，不重复执行。不同 clock_id 的期限计算须经过可靠映射，否则拒绝执行跨域请求。

客户端超时表示结果未知；重试使用原 ID，不创建新动作请求。REQ 超时后恢复其请求状态机或重建 socket。非幂等动作的执行身份和结果需持久化或通过设备状态对账；ACK 不保证恰好一次。服务端重启也不能把旧请求当新请求重新执行。缓存保留期限、最大重试次数、身份认证和可取消阶段必须在部署前冻结。

### 14.5 RS422 Gateway → Core 高层操作服务（新增设计，待实现）

配置 `core_operation_service=tcp://127.0.0.1:5559`，Core ROUTER bind，Gateway DEALER connect，使用第 14.1 节单帧 JSON/路由信封，不经 5555/5556。每个 socket 由固定非实时线程管理。此接口独立于第 19 节 Core → Manipulator 的 5558 REQ/REP 服务。以下操作注册到 target=`aviator_core`：

| RS422 operation | ServiceRequest.operation |
| --- | --- |
| 0x01 ENTER_STANDBY | `enter_standby` |
| 0x02 GRASP_WHEEL | `grasp_wheel` |
| 0x03 START_CONTROL | `start_control` |
| 0x04 EXIT_CONTROL | `exit_control` |
| 0x05 LEAVE_WHEEL | `leave_wheel` |
| 0x06 RESET_ERROR | `reset_error` |

ServiceRequest 使用第 14.2 节信封，client_id=`flight_gateway`；parameters 必含 `server_session_id`（预期 Core 启动会话）、`rs422_connection_id`（受控建立的本次串口运行连接 UUID）、`timestamp_ms`（uint32）和 `operation_code`（上述 u8）。操作名和操作码必须一致。Core 会话由启动就绪握手或受控配置确认，不能从不受信任请求中接受任意会话。连接 ID 仅为内部关联，不增加串口字段，也不使网关具备自动识别飞控重启的能力。

Gateway 按 `(rs422_connection_id, timestamp_ms)` 查找原请求，比较完整原始内容后分配或复用 UUID request_id；与 operation 不同的同时间戳请求按串口冲突规则拒绝，不创建第二项事务。Core 按 `(client_id, client_session_id, request_id)` 去重，并校验参数、原始期限及预期 server_session_id。Gateway 未取得结果时保留 pending 记录，重复副本不创建新的动作；允许以原请求身份进行有界内部重试。受理记录必须先于动作派发建立。

此接口每个逻辑动作固定首个决策为 ACCEPTED、COMPLETED 或 REJECTED，重发原请求返回相同首个决策；长动作的后续结果通过新的 `get_result` 查询及记录适配器获取，不以重试原动作更新首个决策。此约定不同于第 19 节设备服务返回当前进度的行为。查询响应不得转换为第二条串口 REPLY。Gateway 将首个决策映射为串口 result=1/3/4，原样回显 timestamp_ms 和 operation，并将 Core 错误映射到 RS422 第 8 节已定义错误码；未映射错误不得截断为 u16，须在联调冻结错误映射表。

同一时刻仅允许一个普通状态修改操作，安全退出可按状态机规则中止操控开始流程；ROUTER 接收循环不得被长动作占用。服务受理和串口发送共同满足建议 100 ms 应答预算，具体内部 deadline_ms 按实测分配且重试不刷新。Core 不可用或内部超时不伪造业务 REJECTED/ACCEPTED，外部按原协议重试并最终视为结果未知。连接/进程重启后停止自动重试，清理积压并核对实际状态。

### 14.6 当前摇杆测试入口（已实现客户端，Core 服务端待接入）

common/service.hpp 提供第 14.2/14.3 节信封的构造、校验、关联检查和单帧非阻塞收发；不依赖机器人库。flight_gateway 使用 5559 DEALER 发送六个外部状态操作，parameters 为 source=JOYSTICK、button=1..11，配置 Core 会话时增加 server_session_id；不携带第 14.5 节 RS422 专有连接/时间参数。默认前六个按钮对应六个操作，其余可通过 config/flight.yaml 的 buttons 列表配置，内部状态机事件不开放。

服务端仍须检查来源、同机时钟/期限、去重和状态守卫。当前客户端无自动重试，最多 11 项未决请求；发送失败标记 NOT_SENT，等待过期为 UNKNOWN。没有修改 aviator_core，也没有模拟本地状态切换成功。通用信封 helpers 不代表服务端执行语义已实现。

为满足当前最小测试记录需求，Gateway 将记录副本发到总线 record.service.request/reply，Logger 校验后按独立 ServiceRequest/ServiceReply Schema 写入数据 MCAP。请求副本附加可选 gateway_observation=QUEUED/NOT_SENT/TIMEOUT_UNKNOWN，其他字段与请求相同；响应保留收到的完整 JSON，含晚到/未匹配响应。该附加字段不发给 Core。超时不是伪造的 ServiceReply，也不能将 QUEUED 解释为 ACCEPTED。MCAP 按 request_id/client_session_id 关联，服务没有 sequence，不对其报告连续序号缺口。

这是有损可观测的测试路径，不能替代第 15.5 节待实现的独立记录、持久化确认补传及 Core 最终结果采集。Logger 未就绪时 PUB/SUB 可能丢失，当前没有严格记录模式的动作准入门控。使用方式见 flight_gateway 和 aviator_logger 的 README。

## 15. 独立记录通道与二进制信封

### 15.1 传输约定

当前相机节点通过独立 `5557` PUSH/PULL 入口发送 `aviator.record.v1.CameraPacket`，由 Logger 将业务 JSON 与图像写入同一个 MCAP。该相机专用接口见 `schemas/recording/camera_packet.proto` 和 Logger 实现说明。以下通用 `RecordEnvelope` 面向其他原始设备和伺服数据，仍为草案：

- 非实时采集适配器用 PUSH connect，Logger 用单一 PULL bind；每条消息恰好一帧，内容是序列化的 `RecordEnvelope`。
- 原始 bytes 嵌入信封，不使用 Base64，不把大图送入 `5555/5556`。该入口只接纳记录副本，不能传执行命令。
- PUSH/PULL 不提供持久化确认。发送采用有界队列和非阻塞/有界超时；发送失败必须计数并标记缺口，不阻塞伺服或控制线程。
- `max_record_bytes` 按最大图像/伺服批次配置，独立于控制 JSON 的 64 KiB；超限不得无界分配。每个原始采样使用一个信封；批量编码如需启用应另行冻结样本内时间与序号格式。
- 多生产者仅保证逐源身份可追踪，不依赖全局到达顺序代表采样顺序。可靠补传须单独设计确认、持久化和去重，不能把本通道宣称为无损交付。

### 15.2 拟定 Protobuf 定义

以下字段号是草案建议，冻结后不得复用删除的字段号。proto3 默认值不代表业务有效：接收方须验证必需字符串非空、sequence≥1、长度一致以及 oneof 元数据与 payload_type 匹配。二进制 uint64 不受 JSON uint53 限制；导出 JSON 时须采用明确的无损表示规则。

```proto
syntax = "proto3";
package aviator.record.v1;

message RecordEnvelope {
  uint32 envelope_major = 1;       // 必须为 1
  uint32 envelope_minor = 2;       // 初始为 0
  string source_id = 3;            // recording_manifest 登记的数据源
  string topic = 4;                // record.raw.* / record.servo.* / record.camera.* 等
  string payload_type = 5;         // 例如 RawImage、RawDeviceBytes、ServoSample
  string payload_version = 6;      // 例如 1.0
  string publisher_id = 7;
  string session_id = 8;           // UUID，源会话
  uint64 sequence = 9;             // 按 source_id + session_id 递增，从 1 开始
  uint64 timestamp_us = 10;        // 快照生成 UTC 微秒
  uint64 sample_mono_us = 11;      // 原始采样单调微秒
  string clock_id = 12;
  uint64 payload_length = 13;      // payload 的实际字节数，不含信封开销
  bytes payload = 14;              // 原始字节或注册 Schema 的序列化负载
  string payload_encoding = 15;    // raw / protobuf / json / png 等注册值
  string schema_id = 16;           // 解码 Schema/布局版本的注册 ID 或哈希
  string config_id = 17;           // 单位、设备映射、标定配置
  optional uint64 publish_time_utc_ns = 18; // 只有实际采集到发布时间才填
  oneof metadata {
    ImageMetadata image = 20;
    DeviceMetadata device = 21;
    ServoMetadata servo = 22;
    InvalidMessageMetadata invalid = 23;
  }
}

message ImageMetadata {
  string camera_id = 1;
  uint64 frame_id = 2;
  uint32 width = 3;
  uint32 height = 4;
  uint32 stride_bytes = 5;         // 原始行跨度；编码图像时为 0
  string pixel_format = 6;        // 例如 RGB8、MONO8；具体集合由配置冻结
  string calibration_id = 7;
  optional uint64 exposure_us = 8;
}

message DeviceMetadata {
  string device_id = 1;
  string direction = 2;           // RX 或 TX，以适配节点视角定义
  string interface_type = 3;      // RS422 / USB / 已注册驱动接口
  string validation = 4;          // PASS / FAIL / NOT_CHECKED / NOT_APPLICABLE
}

message ServoMetadata {
  string device_group = 1;
  uint64 cycle_id = 2;
  uint32 sample_count = 3;        // 本草案固定为 1
  string layout_id = 4;           // ServoSample Schema、关节顺序、单位的版本
}

message InvalidMessageMetadata {
  repeated bytes frames = 1;      // 原始帧，保留顺序，允许非 UTF-8
  string reject_reason = 2;
  uint64 receive_mono_us = 3;
  string receive_clock_id = 4;
}
```

### 15.3 数据类型与负载规则

| topic 示例 | payload_type / encoding | 负载及必需元数据 |
| --- | --- | --- |
| `record.raw.rs422.rx` | RawDeviceBytes / raw | 原始接收字节，device 元数据包含方向和校验结果。TX 为实际下发字节。 |
| `record.raw.usb.report` | RawDeviceBytes / raw | 原始 USB 报告，设备配置说明报告布局。 |
| `record.servo.manipulator` | ServoSample / protobuf | servo 元数据；每周期输入、插值目标、实际下发量、反馈、安全判断、耗时与 deadline miss。具体 ServoSample Schema 待硬件冻结。 |
| `record.camera.cockpit.image` | RawImage / raw | image 元数据；例如 RGB8 原图，长度=`stride_bytes × height`，stride≥width×3。 |
| `record.camera.cockpit.image` | RawImage / png | image 元数据；无损压缩 bytes，stride=0，长度为编码字节数，解码尺寸/格式必须与元数据一致。 |
| `record.service.request`、`record.service.reply` | ServiceRequest / json、ServiceReply / json | 完整服务 JSON UTF-8 原始字节，不只是 system.event 摘要。 |
| `record.invalid` | InvalidMultipart / raw | invalid 元数据保留各帧；payload 为空，payload_length=0，避免重复存储帧字节。 |

以上 `RecordEnvelope` 图像行属于通用信封草案；当前相机使用 Logger 的 `CameraPacket`。图像 frame_id 与 CameraDetection 关联，记录 sequence 与帧号不是同一计数器。图像副本在推理前入队，队列溢出时显式计数；当前实现尚不能保证每帧均落盘。Logger 的 raw 模式保留原始 RGB8/Z16；compressed 模式的 H.264/H.265 RGB 是有损编码，不能视为原始像素等价物。

payload_length 只核对 payload，不包含 protobuf 元数据。使用压缩图像时它是编码后长度。所有乘法、微秒转纳秒和缓冲区计算必须检查溢出。未知 payload_type/schema_id 不允许猜测解码；登记为不可解码数据并报告配置不一致。记录数据源的布局定义须随 MCAP 保存，否则只有 bytes 不足以形成可复现记录。

### 15.4 MCAP 映射与完整性

| MCAP 内容 | 映射规则（架构基线） |
| --- | --- |
| 业务 Channel | 保留原 Topic，按 Topic、Schema 版本、publisher_id、session_id 区分。 |
| 业务 Schema / Message | Schema encoding=`jsonschema`；Message encoding=`json`，data 保留原始 Frame1 bytes，不重排或重新序列化。 |
| 记录 Schema / Message | Schema 使用含依赖的 Protobuf 描述符；Message encoding=`protobuf`，data 为整个 RecordEnvelope。内嵌 payload 的 Schema 也须注册保存。 |
| `log_time` | Logger 接收 UTC 纳秒。 |
| `publish_time` | 有真实发布时间则使用；没有则取 log_time，不能把快照 timestamp 冒充发布时间。 |
| `sequence` | 完整源序号的低 32 位；完整性对账使用负载中的原序号。 |
| `record.ingest` | 保存接收单调时刻、接收 clock_id、全局接收顺序和完整源引用。其精确 Schema 待冻结。 |
| Metadata / Attachment | 保存记录会话、各卷编号、Schema、构建/配置/标定、清单和完整性状态。 |

源会话与 Logger 记录会话是不同概念，不能用 Logger 会话覆盖信封 session_id。开始/结束清单需记录各源首尾序号与发送计数，和接收、写入数量对账；缺口、未知起始区间、溢出、重复和中断显式可见。上述清单与补传握手的精确格式尚待冻结。

### 15.5 离散服务与 RS422 指令记录契约（新增设计，待实现）

本节独立于相机 CameraPacket；Logger 的总线订阅无法获取 DEALER/ROUTER 或 REQ/REP 服务内容。Gateway、Core 及参与设备服务的节点必须在非实时路径设置记录适配器。

| 记录主题 | 采集内容 |
| --- | --- |
| `record.service.request` / `record.service.reply` | 每次实际发送、接收的完整原始服务 JSON；标明 SEND/RECEIVE、对端和重试序号。 |
| `record.service.result` | Core 受理决策、最终结果、错误、完成依据与结果未知；长动作完成也记录，但不新增串口 REPLY。 |
| `record.rs422.mapping` | 串口运行连接、原 timestamp_ms/operation、内部请求身份的对应关系，以及 Core 派生设备请求的父子关系。 |
| `record.raw.rs422.rx` / `record.raw.rs422.tx` | A/B 端口、原始字节、校验/解析结果、本地时间；TX 区分尝试、驱动接纳字节数与失败，不宣称远端收到。 |
| `record.rs422.decision` | 新请求、pending 副本、缓存应答重发、冲突/旧请求拒绝、超时、切路和受控恢复。 |

服务记录使用 `ServiceAuditRecord/1.0` JSON Schema（待实现）：必含 `recording_session_id`、`producer_id`、`producer_session_id`、`record_sequence`、`topic`、`event`、`mono_us`、`clock_id`、`timestamp`（UTC 微秒或 null），以及 `correlation` 和 `payload`。record_sequence 从 1 连续递增，JSON 中以十进制字符串保存 uint64；每次采集事件分配新序号，补传沿用原序号。correlation 包含可用的 `rs422_connection_id/timestamp_ms/operation_code/client_id/client_session_id/request_id/server_session_id` 和 `parent_request_id`，尚不可解析的字段为 null，不猜测事务关联。服务原始 JSON 放入 payload 的 `raw_json` 字符串，解码字符串所得 UTF-8 字节须与原报文一致；其他事件 payload 按主题 Schema 冻结。原始串口二进制继续使用 RecordEnvelope，不转换成服务 JSON；其记录身份与关联信息须在冻结 Protobuf 时补齐。记录 ID 与业务请求 ID 是两套独立身份。

普通采集模式沿用第 15.1 节有界记录副本及缺口报告，不保证持久交付。需要故障后补齐的离散服务记录使用新增配置 `service_record_endpoint`，建议 `tcp://127.0.0.1:5560`：生产者 DEALER connect、Logger ROUTER bind，单帧 ServiceAuditRecord JSON；Logger 回复单帧 `RecordAck/1.0` JSON，包含原 `producer_id/producer_session_id/record_sequence` 及 `status=DURABLE`。ROUTER 侧仅增加 routing_id，无空分隔帧；此端点不承载控制指令，不复用相机 5557 或控制服务 5559。原始串口通用二进制记录暂沿用普通采集模式；若要求原始字节也可补传，须扩展并冻结对应二进制确认接口，不能据服务记录确认宣称原始帧无损。

确认补传模式的非实时适配器先将记录写入有界本地持久化 spool，再发送；Logger 仅在 MCAP 记录及用于重启恢复的记录身份索引完成规定持久化屏障后确认，入内存队列或仅调用 Writer.write 不得回 DURABLE。生产者收到匹配确认才删除 spool 项；断连或确认丢失时以原记录身份有界重传。Logger 按 `(producer_id, producer_session_id, record_sequence)` 幂等接纳；相同身份不同内容报冲突，重启先恢复已持久化索引再确认重复记录。传输提供至少一次交付，归档按记录身份去重，不承诺业务动作恰好一次。批次持久化周期、spool 字节上限和补传限速在部署配置冻结并实测，不允许无界缓存。

实时线程和安全停止不得等待 spool/Logger；因此采集事件入队到 spool 持久化之间仍有崩溃丢失窗口。队列/磁盘满、入队失败及缺失尾部必须标记 incomplete/unknown，并保存逐源起止清单和生产/持久化计数。严格记录模式下 Logger/spool 未就绪拒绝新的普通试验动作，安全退出仍执行并尽力记录。不能把记录 ACK 映射为业务 REPLY，也不能因记录失败改写已经成立的 Core 受理结果。

验收至少覆盖：一次长动作的一条串口逻辑应答与独立最终结果；A/B 副本和双层重试只执行一次且完整留痕；Logger 断连后补传；确认丢失后的记录去重；Logger/生产者重启恢复；慢盘、磁盘满及队列溢出不阻塞安全退出且显式报告缺口；按映射从原始 REQUEST 追到 Core 决策、设备子请求及 REPLY。只有启用源的起止序号和计数核对通过，才标记该源记录完整。


## 16. 回放消息

回放复用业务两帧协议，默认只连接 `6555/6556`，不向生产执行器发布。算法回归仅发布选定输入，不同时回放历史 Core 输出。真实硬件参与时必须另有测试授权。

重新发布时分配新 session_id 和递增 sequence，按测试白名单登记 publisher_id，并保留以下拟定元数据：

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `original_header` | object | 保存原 msg_type、version、publisher_id、session_id、sequence、timestamp、sample_mono_us、clock_id。 |
| `original_header.control_epoch` | string，可选 | 原消息存在时保留，仅用于追溯。 |
| `original_header.origin` | object，可选 | 原始派生输入引用，仅用于追溯。 |

原始完整消息继续保存在 MCAP，不以 original_header 代替原始存档。实时重映射模式下，将当前头部、origin、各侧采样时间、轨迹起点等全部相关单调时刻映射到同一测试时钟域；虚拟时钟模式下所有参与算法和 watchdog 使用同一注入时钟。暂停、倍速、单步不能混用真实时间和历史采样时间。

原 control_epoch 只可进入 original_header，不能作为当前授权；测试设备命令必须使用新的测试 epoch。消息引用的 session/sequence 同步映射到回放输入；无法映射的派生目标仅供查看，不标记为可执行。不得只改头部时间而保留过期 origin 掩盖输入失效。

## 17. 接收校验、拒绝与兼容

### 17.1 业务接收顺序

1. 有界接收完整消息，检查恰好两帧、Topic 字节与各帧长度。
2. 按注册表精确匹配 Topic，校验 UTF-8、单个 JSON 对象、重复键、深度及类型。
3. 校验公共必填字段、Topic/msg_type 对应关系、major/minor 和整数边界。
4. 校验部署模式、生产者、控制源、会话，以及设备目标的 control_epoch。
5. 校验序号、时钟域、未来时间、原始年龄和 origin 年龄。
6. 校验 mode、数组长度、双侧完整性、数值、关节/图像/坐标范围及当前状态机前置条件。
7. 将已校验对象转成强类型快照；仅有效且新鲜的数据更新控制 mailbox 和有效 watchdog。合法 invalid 报告只更新失效状态和诊断。

RT 执行侧再次检查本地命令年龄、origin、授权和本地安全约束；通信线程校验不能替代执行时复核。执行器在收到无效报告或超时后进入模式规定的有界安全动作，不把“失效”解释为目标归零。

### 17.2 建议拒绝原因

以下为诊断字符串，属于补充拟定，不是已经冻结的数值错误码：

| 原因 | 触发条件 |
| --- | --- |
| `BAD_FRAME_COUNT / PAYLOAD_TOO_LARGE` | 帧数或大小非法。 |
| `UNKNOWN_TOPIC / TYPE_MISMATCH` | 未注册 Topic 或 msg_type 不一致。 |
| `INVALID_UTF8 / INVALID_JSON / DUPLICATE_KEY` | 编码/语法/重复键异常。 |
| `SCHEMA_VIOLATION / UNSUPPORTED_VERSION` | 类型、必填字段或版本异常。 |
| `UNAUTHORIZED_SOURCE / STALE_SESSION / INVALID_EPOCH` | 来源、会话或授权代次不合法。 |
| `DUPLICATE_OR_OUT_OF_ORDER` | 序号不递增。 |
| `CLOCK_DOMAIN_MISMATCH / FUTURE_SAMPLE` | 无法比较时钟或采样时间异常未来。 |
| `STALE_SAMPLE / STALE_ORIGIN` | 原始数据或上游输入过期。 |
| `INVALID_MODE / OUT_OF_RANGE / CONFIG_MISMATCH` | 模式、数值、数组、坐标或配置不符。 |

错误消息不进入控制缓存；按原因计数并限速输出诊断。Logger 在自身入口上限内保留原始各帧到 record.invalid。合法但 valid=false 是业务失效报告，不等同于格式损坏。

### 17.3 演进规则

- minor 仅增加可忽略的可选字段；接收者可读取已知字段，不能依赖未知字段完成安全判断。
- 删除字段、将可选变必填、改变类型、单位、坐标语义、数组解释或 null 规则，均需升级 major。
- 新控制枚举或模式需要能力匹配；旧接收者拒绝未知模式，不回退默认模式。不能仅增加 minor 就要求旧执行器理解新动作。
- 同一冻结版本的字段、错误码和枚举应来自单一 Schema/注册表，C++ 类型、日志解码和离线工具共同校验。
- Protobuf 新字段使用新编号，旧编号不复用；改变既有字段语义需升级信封或 payload 主版本。

## 18. 接口冻结与联调清单

| 项目 | 冻结内容 |
| --- | --- |
| 硬件与数组 | 左右臂/手自由度、joint_names 顺序、角度或归一化反馈能力、位置/速度/加速度边界。 |
| 坐标与映射 | TCP 参考系、安装外参、四元数容差、roll/pitch 正方向、机械行程映射。 |
| 控制授权 | source 枚举、白名单、session 接纳/退役规则、control_epoch 安装与撤销流程。 |
| 时间预算 | 告警/超时、camera.command 期限、未来容差、跨主机映射和时钟误差上限。 |
| 模式能力 | JOINT_TRAJECTORY、归一化手驱动与抓握模式是否启用，插值/替换/变化率和安全动作。 |
| 相机 | camera_id、图像格式/尺寸、ROI 坐标、置信度阈值、帧号与原图记录关联。 |
| 可靠服务 | 端点、socket 模式、操作参数、权限、结果保存、查询/取消、重试与持久去重。 |
| 记录通道 | PUSH/PULL 与信封字段号、数据源清单、payload Schema、最大消息、队列、首尾清单和补传需求。 |
| 协议资产 | 八 Topic 与扩展 JSON Schema、错误码表、Protobuf 定义、正确/错误消息夹具。 |

联调至少覆盖：八个 Topic 的完整消息编解码；两帧/多帧异常与精确 Topic 匹配；缺字段、重复键、空值、未知版本/模式、数组长度错误；重复/乱序/旧会话；旧样本换新 sequence；Core 新目标携带过期 origin；UTC 跳变和 clock_id 不匹配；失效视觉不回中；双侧反馈不同步；Bus/Core 重启不自动使能；服务超时重试不重复执行；记录超限/缺口可见；回放端点、授权和时钟隔离。

公共编解码器及部分运行节点已经实现；尚未落地的 Schema 和能力仍需按清单实施。文档示例的语法检查不能替代协议实现、SocketCAN 通信或实体设备验证。

## 19. 双臂与手节点迁移采用的协议能力

本节记录 `nodes/aviator_core`、`nodes/manipulator` 和 `nodes/aviator_hand` 当前采用的具体能力。前文“尚未实现”的历史说明不能替代当前代码；common 已实现公共编解码、来源/会话/时效校验，完整系统的所有可选业务模式并未全部实现。

### 19.1 本地任务与配置

本次承接原示例的 Demo、Servo Demo 和交互操作，origin.topic 显式为 `local.task`，publisher_id 为 aviator_core，session_id 为当前 Core 会话。origin.sample_mono_us 是 Core 主线程最近一次任务监督采样时间；通信线程不能自行刷新它。此来源只在显式 authorize/enable 后生效，不伪造 flight.command。现有 InputGuard 默认仍只授权 flight.command，Manipulator 明确配置 local.task 策略。

system.yaml 统一配置总线与服务端点，开发默认服务 tcp://127.0.0.1:5558，三者仅接受本机回环地址。当前身份白名单是受控本机进程约束，JSON 身份字段不构成密码学认证。生产跨主机访问需要另行实现认证及可靠时钟映射。

### 19.2 JOINT_TRAJECTORY / SYNCHRONIZED_TICKS

本地执行采用明确能力字段 `execution:"SYNCHRONIZED_TICKS"`，其他模式拒绝。它补充第 6.3 节的执行约定：

- arm.command 以 100 Hz 更新窗口，双侧每条 2—32 个点，time_from_start_us 为 0、2000、4000…，均含完整 7 轴位置和速度。
- 新增 trajectory_id（当前 epoch 内递增）、first_tick、total_ticks，tick 单位 1 ms；first_tick/total_ticks 为偶数，total_ticks 不超过 3,600,000。每个窗口最多覆盖 62 ms。
- trajectory_start_mono_us 是本窗口的名义时间起点；实际进度由执行游标决定。发生调度延迟只延长执行，不按墙钟跳过轨迹点。该字段不延长授权或 watchdog。
- Manipulator 线性插值得到 1 ms 位置点，每个执行步双臂共同推进一次；SDK 回调之间只协调消费顺序，不提供硬件同步保证。
- 同一轨迹的新窗口须覆盖当前游标，并与该游标的已下发指令一致；替换尚未执行的窗口，不建立无界追加队列。新轨迹必须从 tick=0、上一段已完成的最后指令开始。
- wheel_reference 为每个线上点对应的 `[angle_rad, displacement_m]` 数组，长度与 points 相同；它是业务几何参考，真机没有轮盘测量反馈。
- 实际下发前再次检查时效、关节限位与相邻指令速度上限。Core 完整规划并检查 IK、碰撞和速度后再发布；Home 使用 Ruckig，传输采样与本地插值会产生离散近似，不能宣称 SDK 输出严格保留连续曲线的所有高阶导数。

arm.state.execution 包含 trajectory_id、tick、target[14]、stopping、fault、error。accepted_command 仍只表示接纳；执行游标表示指令进度，不能用作实际到位证明。software_lock 表示软件操作阶段，wheel_reference 表示当前指令参考。只有 MuJoCo 发布 wheel_measurement，供观测与独立仿真测试使用。

Managed 入口的 Rokae 待机就绪判据使用 arm.state 的 `status_mono_us`（同机单调微秒，设备状态发布时间）；未使能时 RT 采样可以无效。Managed flight.state.valid 表示整机状态发布依据及 Core 拥有线程更新有效，机械臂采样时效仍通过 freshness.arm.valid 单独报告；这允许 Gateway 在未使能时自动识别 Core，不表示机械臂数据有效。只有 UNINITIALIZED/INITIALIZED/DISABLED、无进行中轨迹和无停止任务时允许此判据。开始运动后仍检查原始采样时间及 valid，不使用 status_mono_us 给关节/TCP 采样续期。旧 Manipulator 不提供该字段，需与 Core 一起升级。

### 19.3 服务与生命周期

本次采用 REQ/REP 单帧服务。describe 只读返回 server_session、q、target、speed、backend 和 config_id；authorize 在未使能且无故障、无进行中操作时绑定 Core 会话，返回 control_epoch。其余操作为 enable、disable、stop、lock、unlock、reset_fault。

除 describe/get_result 外 parameters 必须恰好包含 server_session 与 config_id。请求身份、时钟、期限、目标、操作和参数均校验；deadline_ms 范围 1—10000 ms。长操作返回 ACCEPTED/RUNNING，客户端重发完全相同的请求查询状态，完成后返回原结果。它不重新执行原动作，也不刷新原始期限；另支持 get_result：parameters 增加 original_request_id/original_client_session_id，使用新的查询请求期限读取原结果，外层 COMPLETED 表示查询完成，原动作状态放在 result 内。未知记录返回 UNKNOWN，不重新执行。停止通过 stop 服务执行，当前未提供通用 cancel 操作。

去重按 `(client_session_id, request_id)`（固定 client_id=aviator_core）保存，最多 1024 项，容量耗尽拒绝新请求，不淘汰后重做既有动作。服务进程重启更换 server_session，旧请求因前置会话不匹配而拒绝；不跨重启自动重放动作。超时仍表示结果未知。stop/disable/enable/reset 建立命令时间屏障，屏障前积压目标不能重新进入执行。

初次 enable 后只保持实际起始位置，允许最多 1 s 接入目标流；第一条命令后执行配置的 50 ms 命令/100 ms 来源时效检查。正常停止本地减速；失效或故障停止并失能双臂，锁存错误。重新运行须显式处理故障、重新使能，不因总线重连自动恢复。

### 19.4 当前范围

保留原示例已废除的判据：不按 TCP 偏差、关节跟踪偏差、grasp.ready 或抓握丢失阻断执行。软件锁定不等于独立抓握验证。独立 `aviator_hand` 已提供双手归一化驱动位置反馈，但关节角、关节速度和抓握验证仍为 null/false；缺失视觉测量不阻止当前本地开环任务，也不能伪装成有效测量。

`aviator_core` 与 `aviator_core_servo` 为直接调用 Aviator 功能函数的本地任务入口，不经整机业务状态机；`aviator_core_sml` 只做离线状态机测试，不发布/订阅 ZMQ。`aviator_core_managed` 调用 Aviator 的六个状态机操作，导出真实整机状态并控制既有设备接口；它保留本地终端操作，并默认接入摇杆六操作 ZMQ 服务与 flight.command（`--console` 切换为终端目标测试）；RS422 路径仍未实现。Gateway 的 core_session 默认留空，从首个合法、新鲜、有效的 flight.state 自动绑定 Core 会话，请求自动携带 server_session_id；绑定后不自动跨会话切换，Core 重启需重启 Gateway 重新识别。此本地测试入口不依赖安全证据文件，守卫使用设备反馈、Gateway 时效和本地策略，软件急停仅在当前进程锁存。Managed 服务默认 ROUTER bind 5559，强制匹配 Gateway/Core 会话、同机时钟、1–10000 ms 请求期限及六操作白名单，拒绝记录副本；本次 Core 会话最多保存 1024 条原始应答去重，不淘汰执行身份。长动作即时返回 ACCEPTED，最终状态看 flight.state，尚无 get_result/异步最终应答。parameters 恰好为 source=JOYSTICK、button=1..11、server_session_id；不使用第 14.5 节尚未实现的 RS422 参数。默认绑定第一条有效输入的 Gateway 会话，也可显式指定 --gateway-session。仅 CONTROL 调用大写 ServoWheel，POSITION_HOLD 使用通过校验的设备检查时间放行目标，检查时间必须属于本次操控；输入丢失走整机 SAFE 保护，不自动恢复操控。`aviator_core_servo`：默认自动绑定第一条通过来源、时钟、有效性和时效检查的 flight_gateway 消息的 session，也可用 --gateway-session 手动指定。接收 source=JOYSTICK 的 flight.command，按 `roll * 0.87266` rad、`min(pitch, 0) * 0.170` m 映射并调用 ServoWheel(v=1)。入口显式启用 JOYSTICK POSITION_HOLD，按第 5.1 节检查设备检查时间、消息接收时效、有效位和序号；普通输入仍检查原始采样时效。失效后停止更新目标，由 Servo 超时减速，同一会话恢复有效数据后可恢复跟随。自动绑定只进行一次，网关重启后必须重启 Core 重新绑定，不在失效后自动切换会话。flight.state.control_source 在有效跟随时为 JOYSTICK，否则为 NONE。设备轨迹仍使用 Core 的 local.task 来源，尚未把网关原始 origin 贯穿到 Manipulator，不能将此入口解释为完整飞控授权链路。具体启动命令和超时边界见 Core README。

Core 已通过同机 hand.command / hand.state 接入真实手开合，软件 lock 不表示抓握已验证；尚未实现视觉闭环或跨主机控制。离散服务与短轨迹能力只对明确支持本节约定的节点开放，既有 simulation 节点不能直接消费此轨迹模式。

### Managed 状态补充：READY / HOMING

启动流程为 INIT → INITIALIZING（资源初始化、上使能、阻抗配置）→ READY，不自动回 home。外部 enter_standby 触发 READY → HOMING；实际到 home 且停稳后 STANDBY。READY 状态码 11、HOMING 状态码 12，原有状态码保留。六个操作名称不变。释放任务现在包含撤离后回 home，完成后为 STANDBY；初始化 30 s，回 home/抓握/释放各 180 s。部署时同步升级 Core、Gateway 和消费状态枚举的程序。
