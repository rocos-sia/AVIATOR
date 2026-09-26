# AVIATOR ZMQ 协议格式说明

文档编号：AVIATOR ICD ZMQ 001

文档版本：0.1（接口设计草案）

日期：2026年9月26日

依据：[AVIATOR 机器人驾驶飞机控制系统软件架构设计方案 v1.1](AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md)

## 1. 范围与约定级别

本文描述 AVIATOR 内部通过 ZeroMQ 传输的消息，包括八个主要业务 Topic、三个观测扩展 Topic、离散可靠服务、独立数据记录通道及回放约定。RS422 帧、USB 报告、机器人驱动总线和相机 SDK 的外部协议不在本文件定义范围内。

本文按以下级别区分接口来源：

| 标记 | 含义 |
| --- | --- |
| 架构基线 | 架构方案已经明确的传输方式、字段、语义或约束，本文沿用。 |
| 补充拟定 | 为使协议可以实现而补充的具体字段、枚举、格式和边界规则，需在 ICD 评审后冻结。本文中的“必填”对采用本草案的实现生效，不表示原架构已冻结该字段。 |
| 待冻结 | 依赖硬件、标定、控制算法或部署参数，本文不假设实际设备数值。 |

当前 `common/`、`schemas/` 和节点目录尚未提供协议实现；配置文件也仅为占位。本文是实现输入，不是已运行协议的描述。JSON 示例中的 `version:"1.0"` 表示拟定的首个线上协议版本，与本文文档版本不同；冻结前不能据此宣称 v1.0 已发布。

第 2—4 节以架构基线为主；第 5—12 节逐项说明主 Topic；第 13—16 节为扩展、服务、记录和回放；第 17—18 节给出接收检查与冻结清单。未在架构中明确的具体字段结构均属于补充拟定。

### 1.1 节点名称统一

以架构第 15 章的最终目录与进程划分为准：

- `flight_gateway` 根据配置接入 RS422 或 USB 摇杆；`source` 区分输入来源，不另设 `joystick_gateway` 进程。
- `manipulator` 同时发布 `arm.state`、`hand.state`，消费对应命令；Arm/Hand Controller 是其内部逻辑模块。
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
| `hand.command` / `HandCommand` | aviator_core | manipulator | 100 Hz / 10 ms | 30 / 50 ms |
| `hand.state` / `HandState` | manipulator | aviator_core | 100 Hz / 10 ms | 30 / 50 ms |
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
| `session_id` | string，UUID | 每次进程启动生成新 UUID；不得重启后复用旧会话。 |
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

网关重发同一设备样本、Manipulator 重发旧反馈、Camera 重发同一检测时，允许增加消息 `sequence`，但必须保留原始 `sample_mono_us`。Core 的设备命令头部记录此次目标生成/接纳时刻，同时保留 `origin`；FlightState 头部表示本次聚合时刻，同时保留各分组 `freshness`。发送线程重复发送旧计算结果不能改写其采样时刻。

同一已接纳会话内，`sequence <= 已处理序号` 的消息按重复/乱序丢弃。应区分“结构和身份合法的已处理序号”与“有效控制数据”：`valid=false` 可推进已处理序号并立即标记输入失效，但不覆盖最后有效数值、不刷新有效期限。格式错误、未授权等消息不能推进授权序列状态。不要在多个会话之间按最大序号竞争；旧会话退役后不重新接纳。

### 4.4 origin：派生目标的输入溯源

`arm.command`、`hand.command` 携带架构规定的 `origin`：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `origin.publisher_id` | string | Core 采用的输入消息生产者。 |
| `origin.session_id` | UUID string | 输入会话。 |
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

命令类消息：`valid=true` 时所选模式的所有目标必填且非空；`valid=false` 时业务结构仍完整，可保留最后目标，但绝不可执行。无初始目标时可以停止发布，由未就绪状态和 watchdog 阻止执行，不构造伪造零目标。

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

### 8.1 字段（补充拟定）

架构规定双手连续关节或抓握设定值，且不能将归一化驱动刻度标为 rad。本文用显式模式区分：

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `mode` | enum string | `JOINT_POSITION / NORMALIZED_POSITION / GRASP_SETPOINT`，仅允许配置声明支持的模式。 |
| `origin` | object | 第 4.4 节。 |
| `control_epoch` | UUID string | 第 4.5 节。 |
| `hands.left`、`hands.right` | object | 双手完整目标，任一侧非法整条拒绝。 |
| `hands.{side}.joint_position` | number[] | JOINT_POSITION 必填，rad，匹配关节配置。 |
| `hands.{side}.drive_position_normalized` | number[] | NORMALIZED_POSITION 必填，`[0,1]`，匹配驱动通道配置。 |
| `hands.{side}.grasp` | object | GRASP_SETPOINT 必填。 |
| `hands.{side}.grasp.profile_id` | string | 已加载的抓握映射配置；不得触发临时标定或加载任意动作脚本。 |
| `hands.{side}.grasp.closure` | number | `[0,1]`，0/1 分别对应配置中的张开/闭合端点；不是抓握力。 |

三类目标互斥；不能把缺失目标理解为“保持”。抓握设定值是可覆盖的连续期望，抓握验证、使能和整机 GRASPING 状态转换另行管理。双手异构需要混合模式时，另行冻结兼容设计；本草案一个消息使用一个 mode。

### 8.2 完整 Frame1 示例

Frame0：`hand.command`。

```json
{
  "msg_type": "HandCommand", "version": "1.0", "sequence": 20466,
  "timestamp": 1790121600001000, "sample_mono_us": 12345679000,
  "clock_id": "hostA-boot1", "publisher_id": "aviator_core",
  "session_id": "22222222-2222-4222-8222-222222222222", "valid": true,
  "control_epoch": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
  "mode": "JOINT_POSITION",
  "origin": {
    "publisher_id": "flight_gateway",
    "session_id": "11111111-1111-4111-8111-111111111111",
    "sequence": 182736, "sample_mono_us": 12345678000, "clock_id": "hostA-boot1"
  },
  "hands": {
    "left": {"joint_position": [0.12, 0.35, 0.42, 0.31, 0.26, 0.18]},
    "right": {"joint_position": [0.12, 0.35, 0.42, 0.31, 0.26, 0.18]}
  }
}
```

## 9. hand.state — HandState

### 9.1 字段

| 字段 | 类型 | 规则 |
| --- | --- | --- |
| `hands.left`、`hands.right` | object | 双手完整状态。 |
| `hands.{side}.valid/status/enabled/error_code` | 同 ArmState | 补充拟定；测量有效不等于具备执行条件。 |
| `hands.{side}.sample_mono_us` | uint53 或 null | 补充拟定；每侧原始采样时间，null 表示从未采集。 |
| `hands.{side}.joint_position` | number[] 或 null | 可提供角度反馈时为 rad；不具备角度能力时为 null。 |
| `hands.{side}.joint_velocity` | number[] 或 null | 可提供角速度反馈时为 rad/s；能力缺失时为 null。 |
| `hands.{side}.drive_position_normalized` | number[]，条件必填 | 仅有归一化驱动位置反馈时必填，`[0,1]`；补充拟定。 |
| `hands.{side}.grasp_verified` | boolean，可选 | 经独立判据验证抓握完成；不能用 closure 达到目标直接替代验证。 |
| `accepted_command` | object 或 null，可选 | 同 ArmState，用于手命令引用。 |

顶层 valid 和双侧采样时刻合成规则同 ArmState。角度或速度能力缺失必须在配置和记录清单登记，不得用零值填补；归一化反馈可以在设备声明的能力下有效，但需要角度/角速度的上层控制模式仍判定该反馈能力不足。FlightState 保留相同的能力与空值语义。

### 9.2 完整 Frame1 示例

Frame0：`hand.state`。

```json
{
  "msg_type": "HandState", "version": "1.0", "sequence": 20466,
  "timestamp": 1790121600002000, "sample_mono_us": 12345675000,
  "clock_id": "hostA-boot1", "publisher_id": "manipulator",
  "session_id": "33333333-3333-4333-8333-333333333333", "valid": true,
  "hands": {
    "left": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "sample_mono_us": 12345675000,
      "joint_position": [0.12, 0.35, 0.42, 0.31, 0.26, 0.18],
      "joint_velocity": [0, 0.01, 0.02, 0.01, 0, 0], "grasp_verified": true
    },
    "right": {
      "valid": true, "status": "ACTIVE", "enabled": true, "error_code": 0,
      "sample_mono_us": 12345675000,
      "joint_position": [0.12, 0.35, 0.42, 0.31, 0.26, 0.18],
      "joint_velocity": [0, 0.01, 0.02, 0.01, 0, 0], "grasp_verified": true
    }
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
| `hands.{side}.joint_position/joint_velocity` | 同 HandState 测量字段 | 必须保留双手；归一化反馈字段按能力条件附带。 |
| `vision.status` | enum string | 同 CameraDetection。 |
| `vision.confidence` | number | `[0,1]`。 |
| `vision.yoke` | object | detected、roll、pitch，同 CameraDetection。 |
| `freshness.arm/hand/camera` | object | 本文采纳架构建议，作为本草案必填项。 |
| `freshness.{group}.valid` | boolean | 聚合时该组的业务有效性及年龄是否满足当前要求。 |
| `freshness.{group}.age_ms` | number 或 null | 聚合时原始样本年龄，ms，可带小数；无样本或无法映射时钟时为 null 且 valid=false。 |
| `freshness.{group}.sequence` | uint53 或 null | 最近采纳的源消息序号；从未收到时为 null。 |
| `freshness.{group}.publisher_id/session_id/clock_id` | string，可选 | 补充拟定；建议附带以消除跨会话歧义。 |
| `freshness.{group}.sample_mono_us` | uint53，可选 | 补充拟定；直接复核该组年龄。 |

顶层时间是聚合时间；顶层 valid 按当前模式所必需的反馈、输入和条件计算。视觉可选的模式允许 `freshness.camera.valid=false` 而顶层仍有效；依赖视觉的模式不允许。臂手组年龄按较旧侧样本保守计算，同时检查两侧状态。子组过期时保留最后值用于显示，但其 freshness.valid=false。

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
      "joint_position": [0.12, 0.35, 0.42, 0.31, 0.26, 0.18],
      "joint_velocity": [0, 0.01, 0.02, 0.01, 0, 0]
    },
    "right": {
      "joint_position": [0.12, 0.35, 0.42, 0.31, 0.26, 0.18],
      "joint_velocity": [0, 0.01, 0.02, 0.01, 0, 0]
    }
  },
  "vision": {
    "status": "TRACKING", "confidence": 0.96,
    "yoke": {"detected": true, "roll": 0.32, "pitch": -0.15}
  },
  "freshness": {
    "arm": {"valid": true, "age_ms": 4, "sequence": 20470},
    "hand": {"valid": true, "age_ms": 5, "sequence": 20466},
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

头部 `sample_mono_us` 使用图像采集时刻，timestamp 使用检测快照生成时刻。原始图像用 `(publisher_id, session_id, camera_id, frame_id)` 与记录信封关联；推理队列可以跳帧，不要求检测 frame_id 连续。重复发布同一帧时不得改变 frame_id 或采样时间。

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

架构列举 enable、reset_fault、set_source、calibrate，明确后续使用独立服务端点。本文补充服务信封，不表示该服务已实现。简单同步部署选择 REQ/REP；需要异步进度与并发时选择 DEALER/ROUTER，不在同一端点混用两套帧规则。

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

## 15. 独立记录通道与二进制信封

### 15.1 传输约定

架构规定记录入口 `5557`、版本化二进制信封和 MCAP Protobuf 映射，但未冻结 socket 类型、帧数与 `.proto`。以下均为补充拟定：

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

图像 frame_id 与 CameraDetection 关联，记录 sequence 与帧号不是同一计数器。整条原始图像记录在推理前创建，不能因检测跳帧丢失原图。PNG 为补充拟定的无损编码示例；JPEG 等有损编码不得替代承诺保留的原始图像。

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

本文仅新增文档。Schema、编解码器、运行节点及上述联调检查需在实现阶段落地；本次文档示例的语法检查不能替代协议实现与设备验证。
