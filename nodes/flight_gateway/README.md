# flight_gateway

当前 RS422 帧长为下行 32B、上行 64B，上下行均无帧尾。`aviator-bench-v1` 配置名保留，但旧 9/29 字节程序不兼容；网关与模拟器必须配套更新。当前 UART 为 8E1，帧校验为排除帧头后的直接累加，版本为高 8 位主版本/低 8 位小版本；此前采用其他编码的 32/64B 程序也须更新。

同一程序支持本机手柄/键盘、机器人端双路 RS422、飞控模拟端双路 RS422。机器人端连接本机 ZMQ/Core；模拟端直接读取手柄并与另一台工控机串口通信，不需要 Bus、Core 或机器人 SDK 运行进程。

| role | source | 用途 |
| --- | --- | --- |
| robot（缺省） | joystick | 保留既有本机手柄/键盘，50 Hz 发布 JOYSTICK |
| robot | rs422 | 接收双路 32B，发布 FLIGHT，双路发送 232B |
| flight_simulator | joystick | 手柄/键盘发送双路 32B，接收/显示两路 232B |

串口按 [RS422 规范第 10 节](../../docs/AVIATOR_RS422通信协议规范.md) 的 `aviator-bench-v1` 台架约定运行。基础 ICD 的未定义字段使用显式台架哨兵，不伪造传感器反馈。下面“键盘操作”和“摇杆输入”章节的 50 Hz、5 分钟失焦保持描述适用于本机模式；模拟端例外见双机运行步骤。

## 双工控机运行

两台机器每台需要两个独立全双工 RS422 通道。A 路对 A 路、B 路对 B 路连接 TX/RX 差分线，具体引脚、参考地和终端配置按串口卡说明书。使用 `/dev/serial/by-id/...` 或稳定 udev 别名；没有该目录的板载串口可使用实际 tty 节点。程序检查两个路径不能指向同一个设备，并使用设备文件锁避免本程序重复打开。当前用户需有串口读写权限。

### 工控机 A：连接机器人

1. 编辑 `config/flight-rs422.yaml` 的 `channel_a`、`channel_b`。
2. 核对 `config/system-rs422.yaml` 中的设备配置；它设置 `flight_source: FLIGHT`。普通 `system.yaml` 仍缺省 JOYSTICK，不自动接受 FLIGHT。
3. 构建后启动：

```bash
cmake --build build --target flight_gateway aviator_core_managed --parallel 2
./scripts/start_aviator.sh --rs422
# 或使用既有仿真设备完成通信联调：
./scripts/start_aviator.sh --simulation --headless --rs422
```

若设备、Bus、Core 已按相同 FLIGHT 配置独立启动，只运行网关：

```bash
./build/bin/flight_gateway --config config/flight-rs422.yaml
```

`--rs422` 启动脚本使用专用 system/flight 配置，避免只改 Gateway 却仍让 Core 接受 JOYSTICK。`--simulation --rs422` 使用 `system-simulation-rs422.yaml` 保留仿真时效参数。

### 工控机 B：接手柄、模拟飞控

推荐仅复制独立项目 [standalone/flight_simulator](../../standalone/flight_simulator/README.md) 到 B，无需部署 AVIATOR 仓库。它自带 CMake、配置、测试和打包入口，仅依赖 SDL2、FreeType、yaml-cpp 和系统中文字体。

```bash
# 在独立项目目录中执行：
cmake -S . -B build -DBUILD_TESTING=OFF
cmake --build build --parallel 2
# 修改 config/config.yaml 的两路串口路径后：
./build/flight_simulator --config config/config.yaml
```

独立配置将 `profile` 放在根级，不含 Bus/Core 参数。已有 `flight_gateway --config config/flight-simulator.yaml` 和 `scripts/start_flight_simulator.sh` 入口继续兼容；其配置格式和独立项目不同，不应直接混用。

输入窗口显示原有按钮映射，标题显示两路 LIVE 状态对应的机器人运行状态（无新鲜上行为 STALE）；终端每秒显示两路帧计数、主备选择、通信位、指令回绕、实测百分比、机构状态、故障、版本和短码。STALE 行保留最后值供诊断，其值不可视为当前状态。

默认 `require_joystick: true`：没有手柄时不发送；拔出手柄停止两路下行。设为 false 可用聚焦键盘，无手柄且失焦立即停发。按键 1～6 默认待机、抓握、控制、退出控制、释放、复位；配置映射与本机模式相同。每次操作发送 10 拍、随后至少 10 拍空闲；期间新点击不缓存，终端提示 BUSY_OR_NEUTRAL 时需重新按键。回绕不是动作完成，必须观察上行状态。

### 切回原有用法

```bash
./scripts/start_aviator.sh
# 或：Core 使用 system.yaml；Gateway 使用默认 flight.yaml。
./build/bin/flight_gateway
```

不在运行中自动切换来源。只打开一个机器人端 Gateway；原有锁文件仍防止串口模式和本机手柄模式同时争用本机输入。

### 串口配置和测试注入

| 字段 | 含义 |
| --- | --- |
| rs422.profile | 必须显式为 aviator-bench-v1 |
| rs422.channel_a / channel_b | 两个不同的绝对设备路径；重连间隔 1 s |
| rs422.receive_timeout_ms | 默认 50，允许 25～100 |
| rs422.recovery_frames | 默认连续 10 帧后可选用通道 |
| rs422.switch_max_step | 默认 100，即切路位置差超过 10 个百分点进入保护 |
| rs422.release_root | 默认程序所在的安装/构建根目录；必须包含正在运行的程序 |
| rs422.allow_test_identity | 缺省 false；台架样例显式 true，仅所有发布元数据缺失时用既有测试短码 |
| simulator.require_joystick | 默认 true；false 允许聚焦键盘备用 |
| simulator.send_a / send_b | 默认 true；改为 false 可模拟单路停发，重启后生效 |
| simulator.corrupt_every | 默认 0；N>0 时每 N 周期破坏 A 路下行校验，B 路保持正常 |

串口固定 115200/8E1、80 Hz，无 LEN、事务 ID 或额外串口回复。双路同时发送，机器人默认主路优先，切备用后不自动抢回。串口/进程重连、计数中断后要求空闲序列，不能自动重放操作。两路同时失效发布 invalid；Core 已进入 ERROR 时链路恢复不会自动恢复 CONTROL。

### 上行反馈的实际边界

`flight.state.wheel_measurement` 仅在设备提供实际测量时上报；当前 MuJoCo 设备具备此字段，未提供测量的硬件节点返回未知位置 `00 80`。后退到位、手刹物理输入尚未接入，使用 `FF`。现有 Core 通用错误 28673 发送未知故障 `FFFF`，不伪装成某个 ICD 业务错误；已分配的 50 个故障码可正常透传。详见协议第 10 节。

启动身份验证调用 `scripts/aviator_release.py identity`，实际校验成功后才使用发布短码，并核对编译版本。台架测试身份在终端明确显示 bench-test，短码 `9CA1`。安装包内的配置也参与发布校验：修改交付配置后需重新生成合法发布清单，不能让校验失败退回测试身份。校验不在 80 Hz 主循环中重复执行。

已经记录本机服务 request/reply；当前串口诊断写入日志，尚未实现逐字节 RS422 MCAP 录制。

## 新增自动测试

```bash
cmake --build build --target aviator_rs422_test aviator_rs422_simulator_driver flight_gateway --parallel 2
ctest --test-dir build -R '^(rs422.*|gateway.*|managed_flight_process)$' --output-on-failure
```

`rs422` 验证独立字节向量、噪声重同步、计数回绕、双路去重、5/10 拍门控、无效字段及 Core 参数。`rs422_process` 使用两个 PTY 和独立 Python 对端验证真实网关进程；`rs422_simulator` 用 SDL 虚拟手柄运行生产模拟器代码，验证轴、10 拍按钮、两路相同发送、反馈解析和手柄掉线。`managed_flight_process` 使用 MuJoCo 验证 FLIGHT 来源的六操作和 ERROR 流程，不连接真机。实际串口卡、接线和机器人坐标方向仍需台架验证。

## 构建和运行

```bash
sudo apt-get install libsdl2-dev libfreetype6-dev fonts-noto-cjk
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel

# 先在另一个终端启动总线。
./build/communication/bin/aviator_bus

# 编辑 config/flight.yaml 后直接启动，无需配置参数。
./build/communication/bin/flight_gateway
```

所有运行配置来自 [config/flight.yaml](../../config/flight.yaml)，支持 `--config <路径>` 和 `--help` / `-h`；不接受原有的设备、端点、按钮等单独启动参数。启动时打印实际配置路径；修改文件后重启生效，不热加载。

构建目录中的程序默认读取编译时定位的源码 `config/flight.yaml`，不依赖当前工作目录。安装后的程序优先读取可执行文件旁的 `../share/aviator/config/flight.yaml`（随 CMake 安装）；该路径不存在时回退源码路径。配置文件必须存在且原有字段完整；新增 `keyboard` 块及其中字段可省略并使用默认值。未知/重复字段、非法按钮事件及超出范围的数值都会导致启动失败。

通过 SDL2 枚举摇杆，默认 `device: auto` 自动选择第一个具备所需轴的设备；无需 `/dev/input` 路径。自动选择及指定索引均跳过名称含 `Keyboard` / `Mouse`（不区分大小写）或“键盘” / “鼠标”的设备，并记录跳过原因，避免 CHERRY 等键盘的额外 HID 轴被当作飞行输入。此过滤也用于热插拔；不按 SDL 类型一律拒绝 `UNKNOWN`，以兼容普通 USB 摇杆。可填写从 0 开始的 SDL 设备索引（如 `device: 0`），但索引可能随插拔变化。启动及重新连接时打印实际名称、索引和实例 ID。支持热插拔；没有可用摇杆时，聚焦输入窗口仍可使用键盘。更换摇杆通常无需修改设备配置，但不同型号的轴/按钮排列仍需核对。

| YAML 字段 | 默认值 / 含义 |
| --- | --- |
| `source` | `joystick`（摇杆+键盘）或 `rs422`（机器人端双路串口，禁用本地输入）。 |
| `device` | `auto` 自动选择，或非负 SDL 摇杆索引；不影响键盘。 |
| `publish` / `subscribe` | `tcp://127.0.0.1:5555` / `tcp://127.0.0.1:5556`。 |
| `service` | `tcp://127.0.0.1:5559`；本机 Core 操作服务，与总线端点不同。 |
| `roll_axis` / `pitch_axis` | `0` / `1`；SDL 摇杆轴索引（0–255），必须不同；键盘不使用。 |
| `invert_roll` / `invert_pitch` | `false`；反转对应轴。 |
| `input_timeout_ms` | `100`，范围 1–100；设备检查结果有效期。 |
| `service_timeout_ms` | `100`，范围 1–10000；服务请求等待期限。 |
| `core_session` | 旧配置兼容字段，已忽略；接收合法 Core 发布者的新鲜反馈，不再固定会话。 |
| `lock_file` | 空字符串自动使用 `/tmp/flight_gateway-<uid>.lock`，否则填写绝对路径。 |
| `buttons` | 恰好 11 项，依次对应按钮 1–11；键盘对应 `1`–`9`、`0`、`-`；`none` 禁用该按钮。 |
| `keyboard.roll_speed` / `keyboard.pitch_speed` | 均为 `1.0`；每秒增加的归一化行程，必须为有限正数。 |
| `keyboard.roll_limit` / `keyboard.pitch_limit` | 均为 `1.0`；各轴对称绝对限位，范围 `(0, 1]`。 |

终端保留启动、状态变化、失焦和超时提示，不周期打印 roll/pitch。仅使用键盘时，连续失焦满 5 分钟后输入无效。

本机手柄模式的生产者固定为 `flight_gateway`，线上报文 source 固定为 `JOYSTICK`。键盘作为虚拟摇杆复用该身份（包括服务请求参数），兼容现有 Core 协议和授权；YAML 的 `source: joystick` 同时接收两类本地输入。启动打印本次 session 和 clock_id，Core 应通过自己的授权流程接纳该会话；打印 STARTED 仅说明节点已初始化，不代表总线已连通或控制已获授权。

## 键盘操作

设置 `source: joystick`，启动后聚焦标题为 `flight_gateway | Input guide` 的 SDL 窗口。键盘无需指定设备，系统连接的键盘均可向此窗口输入。

窗口显示中文操作指南、实际 `buttons` 配置对应的按键、当前键盘焦点/摇杆连接状态，以及 ERROR 恢复步骤。黄色提示说明：仅用键盘且处于 CONTROL 时，窗口失焦立即停止累加并保持当前目标，连续失焦满 5 分钟后输入才失效并触发 ERROR；失焦和超时都会输出提示。5 分钟内重新聚焦可继续操作；已经进入 ERROR 后重新聚焦不会自动恢复控制。故障清除后先复位到 READY，仍在握盘时松盘撤离到 STANDBY，再握盘到 FOLLOWING，最后开始控制。文字使用 FreeType 和系统 Noto CJK（也支持文泉驿微米黑）字体，在启动时预渲染，输入循环只在状态变化或窗口重绘时更新画面。

```yaml
source: joystick
device: auto
keyboard:
  roll_speed: 1.0
  pitch_speed: 1.0
  roll_limit: 1.0
  pitch_limit: 1.0
```

- 左/右方向键沿 roll 负/正方向增加目标，上/下沿 pitch 负/正方向增加；`invert_roll` / `invert_pitch` 同样生效。
- 按住按单调时钟经过时间积分，以 50 Hz 发布，达到各轴限位后保持；默认从零到满行程需 1 秒，不依赖系统按键重复。两轴可同时操作。
- 方向键按下时从当前输出接管对应轴，另一轴继续使用摇杆。松开、到达限位或同轴相反方向同时按下时保持当前目标；换向从当前值继续累加。松开后移动摇杆对应轴可重新接管。实际机械运动仍由 Core 控制。
- 主键盘或小键盘 `0` 将 roll/pitch 目标归零，并停止当前按住的方向键；需松开再按才能继续累加。这是输入目标原点，不是机械臂 Home 操作。
- 主键盘或小键盘 `1`–`9` 对应 `buttons` 第 1–9 项，`-` 对应第 11 项；`0` 专用于目标归零，不触发第 10 项。摇杆第 10 个按钮映射不变。小键盘数字键按物理键位识别，Num Lock 开关均可使用，不映射为方向键。默认 `1` 待命、`2` 握盘、`3` 开始控制、`4` 退出控制、`5` 松盘、`6` 复位错误，其余禁用。键盘和摇杆按钮均可触发，长按重复和松开不发请求；仍需 Core 的新鲜反馈及授权。
- 失去窗口焦点会清除键盘方向状态；重新聚焦后需重新按下。启动或获取焦点时已按住的按键须先松开再按。超过 100 ms 的旧按下事件不会触发控制或状态请求。

SDL 键盘事件依赖输入窗口焦点，不读取终端 stdin，也不全局监听其他窗口。需要可用的桌面显示环境；SSH 终端字符不能直接控制该窗口。无显示环境仅用摇杆时可通过 `SDL_VIDEODRIVER=dummy` 启动，此时没有可操作的键盘窗口。摇杆允许在窗口后台继续使用。

## 摇杆输入及发布语义

- SDL 摇杆轴值范围为 `[-32768, 32767]`，以零为中心分别归一化为 `[-1, 1]`；反转参数保持原有含义。轴和按钮编号采用 SDL 索引，不再采用 Linux ABS/KEY 代码。安装方向和物理行程仍需实际校准。
- 每轮在主线程泵送 SDL 事件，队列排空后读取 SDL 摇杆状态并积分键盘目标。静止摇杆可以立即建立有效位置，无需先移动。合并后的目标使用本次软件采样的 CLOCK_MONOTONIC 时间作为 `sample_mono_us`，不是硬件报告时间；输入切换不会使时间戳倒退。
- 按单调绝对期限以 50 Hz 发布，延迟时跳过错过的周期；每次发布递增 sequence。每轮最多处理 128 个 SDL 事件和 64 条总线消息，避免输入积压阻塞发布。队列未排空时不更新输入检查时间。
- 摇杆移除后停止使用旧摇杆值，自动重新选择符合 `device` 配置的设备；聚焦的键盘仍可用。无摇杆且窗口连续失焦满 5 分钟后 `device_connected=false`、`valid=false`；此前继续发布保持目标。重新插入摇杆或聚焦窗口后恢复本地输入，是否允许运动仍由 Core 状态机决定。
- 关闭 SDL 窗口或收到 SIGINT/SIGTERM 时退出，尽力发布一次 invalid；PUB/SUB 不保证送达，Core 保留 watchdog。

### 位置保持与设备检查

继续沿用已有 `JOYSTICK` / `POSITION_HOLD` 协议：

```json
"input_state": {
  "mode": "POSITION_HOLD",
  "device_connected": true,
  "checked_mono_us": 123456789
}
```

`checked_mono_us` 只在 SDL 事件队列排空、输入状态完成处理后更新；单纯重复发送消息不会续期。检查超过 `input_timeout_ms` 后 valid=false。Core 继续校验来源、会话、序号、有效位、检查时效及接收时效；通用 InputGuard 默认拒绝位置保持扩展。

SDL 的连接状态不能证明设备固件持续产生新硬件报告；键盘焦点也不等同于物理键盘在线检测。参见 [SDL 事件循环](https://wiki.libsdl.org/SDL2/SDL_PollEvent)、[摇杆索引与实例 ID](https://wiki.libsdl.org/SDL2/SDL_JoystickOpen) 和 [后台摇杆事件](https://wiki.libsdl.org/SDL2/SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS)。

## FlightState 反馈

Gateway 接收通过发布者、时钟、序号和时效检查的 Core 反馈；旧 `core_session` 配置已忽略。节点启动标记保留用于记录与序号重置，不要求 UUID 或人工配对。

节点启动时生成普通文本标记；Core 重启后可接收其新鲜反馈，无需同步会话 ID。请求仍需新鲜 Core 状态及 source_authorized 确认。

Managed Core 的 flight.state.valid 表示设备状态及 Core 拥有线程更新新鲜，业务故障看 system.state；机械臂实时数据是否可用看 freshness.arm.valid。Rokae 未使能时也能识别 Core，不伪造 RT 采样。此处只消费系统摘要，不宣称完成双臂、双手、视觉业务 Schema 校验；USB 摇杆暂不向设备回传或发送震动，RS422 回传遵循独立 ICD 及第 10 节台架约定。

## 测试与边界

```bash
ctest --test-dir build/communication --output-on-failure
```

`gateway_sdl` 使用 SDL dummy 视频驱动及虚拟摇杆，覆盖键鼠伪摇杆的自动/指定索引过滤、热插拔过滤、普通 UNKNOWN 摇杆兼容、无摇杆键盘输入、焦点丢失、轴归一化、双输入优先级、按键重复抑制、热插拔和 RS422 不初始化 SDL。原有解码、键盘积分、协议、配置和服务测试继续保留。可运行：

```bash
ctest --test-dir build/communication -R '^(gateway.*|service)$' --output-on-failure
```

真实 USB 权限、桌面焦点、按钮编号与持续操控仍需现场验证。

本次未连接实际键盘或摇杆硬件，设备权限、映射、发布抖动和持续操控需真机验证。本机手柄模式不记录原始输入；RS422 模式见前述双机运行章节。安装入口为 `${CMAKE_INSTALL_BINDIR}/flight_gateway`。

## 摇杆按钮状态请求

按钮编号 1–11 对应 SDL 摇杆按钮索引 0–10；编号不保证与外壳标注一致。默认按钮 1–6 依次为 `enter_standby`、`grasp_wheel`、`start_control`、`exit_control`、`leave_wheel`、`reset_error`，7–11 未分配。仅这六个外部状态事件可配置，Boot/Done/Fault/Emergency 不作为测试按钮接口。

直接编辑 flight.yaml 的 buttons 列表，例如把第 7 项改为 `exit_control`、第 11 项改为 `reset_error`；填 `none` 禁用对应按钮。所有配置均从 YAML 读取。


SDL 按下沿触发一次；松开、自动重复、打开设备时已按住和超过 100 ms 的旧按下事件不产生请求。连续轴值仍以 50 Hz 发布；反馈支持 INITIALIZING/READY/HOMING/RELEASING 状态；启动准备完成是 READY，按钮 1 回 home 后才是 STANDBY。

服务采用 DEALER → ROUTER，单帧 `ServiceRequest/1.0`，target=`aviator_core`、client_id=`flight_gateway`，parameters 为 `{"source":"JOYSTICK","button":1}`。不再发送或校验 server_session_id；回包按 request_id、client_id 和 server_id 关联。Managed Core 保留发布者白名单、请求期限、去重和状态机守卫。

最多保留 11 项未决请求，收发均非阻塞；无已连接服务时输出 NOT_SENT，不缓存到未来连接；已入 ZMQ 队列只代表 QUEUED，超时输出 UNKNOWN，不自动重试。每次新的物理点击是新请求；超时后需核对实际状态，不靠反复点击猜测是否执行。响应核对请求/客户端/目标身份，晚到及不匹配回包只记录，不当成本次成功。

Gateway 将请求副本发布至 `record.service.request`，完整响应发布至 `record.service.reply`；请求副本增加仅记录用的 `gateway_observation=QUEUED|NOT_SENT|TIMEOUT_UNKNOWN`，其余字段与原请求相同，超时记录沿用原 request_id。这些副本不用于执行。先启动 Bus 和 Logger 再测试；PUB/SUB 可能丢记录，没有持久化 ACK/补传。

Managed 真机入口及启动顺序见 [Core README](../aviator_core/README.md#managed-core摇杆按钮与连续目标驱动真机)。只有 CONTROL 状态使用连续轴值。Gateway/Core 可接收对方重启后的新鲜反馈，无需复制或固定会话；故障恢复仍受状态机和控制 epoch 约束。

可用 `ctest --test-dir build/communication -R '^service$' --output-on-failure` 验证模拟 ROUTER 应答、按钮事件和 MCAP 记录；`tests/managed_gateway_process_test.py` 用隔离 MuJoCo 验证真实 Managed 服务端和状态流程。未连接真实摇杆进行按钮标号验证。
