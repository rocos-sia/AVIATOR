# flight_gateway

当前实现 USB Joystick / Keyboard → `flight.command`，并订阅 `flight.state` 的系统状态摘要。采用一个主循环，直接复用 common 的 ZMQ、消息编解码和 InputGuard；没有新增通信框架或后台线程。`main.cpp` 负责设备与循环，`gateway.hpp/.cpp` 负责采样、归一化和消息构造。

RS422 运行入口尚未实现；配置 `source: rs422` 时明确拒绝启动。后续在本节点内添加串口适配。

## 构建和运行

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel

# 先在另一个终端启动总线。
./build/communication/bin/aviator_bus

# 编辑 config/flight.yaml 后直接启动，无需配置参数。
./build/communication/bin/flight_gateway
```

所有运行配置来自 [config/flight.yaml](../../config/flight.yaml)，只保留 `--help` / `-h` 查看用法，不接受原有的设备、端点、按钮等启动参数。启动时打印实际配置路径；修改文件后重启生效，不热加载。

构建目录中的程序默认读取编译时定位的源码 `config/flight.yaml`，不依赖当前工作目录。安装后的程序优先读取可执行文件旁的 `../share/aviator/config/flight.yaml`（随 CMake 安装）；该路径不存在时回退源码路径。配置文件必须存在且原有字段完整；新增 `keyboard` 块及其中字段可省略并使用默认值。未知/重复字段、非法按钮事件及超出范围的数值都会导致启动失败。

使用 Linux evdev `/dev/input/event*` 或稳定符号链接，**不是 `/dev/input/js*`**。设备路径不存在、无权限或不支持所选轴（摇杆）/方向键（键盘）/单调事件时钟时直接报错退出；修改 YAML 中的 device 后重启，不再在终端临时输入覆盖。程序不自动扫描其他设备。

| YAML 字段 | 默认值 / 含义 |
| --- | --- |
| `source` | `joystick` 或 `keyboard`；默认 `joystick`，`rs422` 尚未实现。 |
| `device` | `/dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick`；绝对 evdev 路径。 |
| `publish` / `subscribe` | `tcp://127.0.0.1:5555` / `tcp://127.0.0.1:5556`。 |
| `service` | `tcp://127.0.0.1:5559`；本机 Core 操作服务，与总线端点不同。 |
| `roll_axis` / `pitch_axis` | `0` / `1`；Linux ABS 事件代码，必须不同；键盘模式不使用。 |
| `invert_roll` / `invert_pitch` | `false`；反转对应轴。 |
| `input_timeout_ms` | `100`，范围 1–100；设备检查结果有效期。 |
| `service_timeout_ms` | `100`，范围 1–10000；服务请求等待期限。 |
| `core_session` | 旧配置兼容字段，已忽略；接收合法 Core 发布者的新鲜反馈，不再固定会话。 |
| `lock_file` | 空字符串自动使用 `/tmp/flight_gateway-<uid>.lock`，否则填写绝对路径。 |
| `buttons` | 恰好 11 项，依次对应按钮 1–11；键盘对应 `1`–`9`、`0`、`-`；`none` 禁用该按钮。 |
| `keyboard.roll_speed` / `keyboard.pitch_speed` | 均为 `1.0`；每秒增加的归一化行程，必须为有限正数。 |
| `keyboard.roll_limit` / `keyboard.pitch_limit` | 均为 `1.0`；各轴对称绝对限位，范围 `(0, 1]`。 |

启动后以最多 10 Hz 打印归一化控制值 `roll=... pitch=...`（范围 `[-1, 1]`，保留四位小数），终端中在同一行刷新，退出或输出其他日志前自动换行；重定向到文件或管道时逐行输出。不再打印 `input=VALID/INVALID`。首次成功设备查询或完整输入报告建立初始位置，失效后显示最后采样值；显示值不代表消息 valid=true。

生产者固定为 `flight_gateway`，线上报文 source 固定为 `JOYSTICK`。键盘作为虚拟摇杆复用该身份（包括服务请求参数），兼容现有 Core 协议和授权；YAML 的 `source` 选择本地输入设备类型，启动日志 `input=keyboard` 可区分实际输入。启动打印本次 session 和 clock_id，Core 应通过自己的授权流程接纳该会话；打印 STARTED 仅说明节点已初始化，不代表总线已连通或控制已获授权。

## 键盘操作

将 `config/flight.yaml` 中 `source` 改为 `keyboard`，`device` 改为实际键盘的绝对 evdev 路径，例如 `/dev/input/by-id/usb-<你的键盘>-event-kbd`。可用 `ls -l /dev/input/by-id/*-event-kbd` 查找设备；内置键盘也可使用 `/dev/input/by-path/*-event-kbd`。程序需要该设备读取权限，不扫描或自动选择其他设备。

`kbd` 后缀不是必需条件。部分复合设备的键盘接口可能被命名为 `event-joystick`：例如本机 `CHERRY MX 3.0S Dongle Keyboard` 对应 `/dev/input/event7`，其稳定链接为 `/dev/input/by-id/usb-CHERRY_MX_3.0S_Dongle-event-joystick`。没有 `*-event-kbd` 时，查看 `cat /proc/bus/input/devices`，按设备名称找到键盘，在同一段 `Handlers` 中取得 `eventN`，再通过 `ls -l /dev/input/by-id /dev/input/by-path` 查找指向它的链接。也可直接配置 `/dev/input/eventN`，但编号可能在重启或拔插后改变。网关按设备实际按键能力校验，不按路径后缀判断。

```yaml
source: keyboard
device: /dev/input/by-id/usb-<你的键盘>-event-kbd
keyboard:
  roll_speed: 1.0
  pitch_speed: 1.0
  roll_limit: 1.0
  pitch_limit: 1.0
```

- 左/右方向键分别沿 roll 负/正方向增加绝对位置目标，上/下分别沿 pitch 负/正方向增加；`invert_roll` / `invert_pitch` 同样生效。
- 按住后按单调时钟经过时间积分，以 50 Hz 发布，达到各轴限位后保持；默认从零到满行程需 1 秒，与操作系统的按键重复延迟/频率无关。roll 和 pitch 可同时操作。
- 松开后该轴目标在下一发布周期回零（正常调度最多约 20 ms），另一轴不受影响。同轴相反方向同时按下时回零；剩一个方向或切换方向后从零重新增长。这里的回零指发布的位置目标，实际机械运动仍由 Core 控制。
- 主键盘 `1`–`9`、`0`、`-` 依次对应 `buttons` 的第 1–11 项；不含数字小键盘。默认 `1` 待命、`2` 握盘、`3` 开始控制、`4` 退出控制、`5` 松盘、`6` 复位错误，其余禁用。按下沿在完整输入帧后请求一次，长按重复和松开不发请求；仍需 Core 的新鲜反馈及授权。
- 启动时已按住的方向键和数字键须先松开再按才生效。输入帧未完成或队列未排空时不续期；拔出、读/查询失败、丢帧、异常时间或超过 100 ms 的旧键盘事件锁存失效，需要重启。

使用本机 Linux evdev 读取真实按下/松开事件，不读取 stdin；SSH 终端字符不能控制远端键盘设备。设备监听不依赖窗口焦点，也不独占键盘，节点运行期间在其他窗口按同样的键仍会被识别。

键盘每 20 ms 用 `EVIOCGKEY` 检查设备，在队列排空且完整输入帧已提交后积分产生新的软件位置目标，同时更新 `sample_mono_us` 与 `checked_mono_us`。这与下述摇杆保持原始硬件采样时间的行为不同，两者都沿用 `POSITION_HOLD` 检查链路。

## 普通用户设备权限

出现 `Permission denied` 时，可使用根目录的 udev 配置脚本：

```bash
sudo ./scripts/setup_joystick_udev.sh --device /dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick
```

脚本自动读取 USB VID/PID，为当前 sudo 用户配置专用组的读取权限。执行后注销并重新登录，必要时拔插设备。支持 `--dry-run` 预览及 `--user` 指定用户，详见 [scripts 使用说明](../../scripts/README.md)。

## 摇杆输入及发布语义

- 设备事件通过 `EVIOCSCLOCKID` 指定为 CLOCK_MONOTONIC。EV_ABS 更新待提交轴值，SYN_REPORT 才提交完整快照，使用事件原始时刻而非读到事件的时刻。
- 有正负范围的轴以 0 为中心，分别按正负行程归一化；非负范围使用中点。输出范围为 [-1,1]，保留正负满行程。异常越界值拒绝，不静默截断。
- 启动后，事件队列已读完、无半帧且两轴 EVIOCGABS 查询成功时，可用查询值建立一次初始位置快照；sample_mono_us 为查询完成时间，不伪造 SYN_REPORT。摇杆静止也可发布有效输入供 Core 绑定，无需先晃动摇杆。后续位置变化仍通过完整事件报告更新。安装方向与物理行程仍需台架校准，内核范围不代替标定。
- 按单调绝对期限以 50 Hz 发布；调度延迟时跳过错过的周期，不补发一串旧命令。每次发布递增 sequence，更新快照 timestamp，**保持原始 sample_mono_us**。
- 每 20 ms 用 EVIOCGABS 检查两轴设备接口、范围和值是否正常，事件队列已读完且无待提交半帧时才更新 checked_mono_us；除首次建立位置快照外，查询结果不替代完整事件帧更新控制目标。设备检查过期后 valid=false。拔出、读取/查询错误、范围改变、SYN_DROPPED、倒退或未来事件时间及越界输入锁存失效，需要重启；不自动重连使能。
- 每轮最多读取 128 个设备事件和 64 条总线消息，避免积压输入独占发布循环。ZMQ PUB 成功不等于送达。
- SIGINT/SIGTERM 通过 signalfd 在主循环处理，退出前尽力发布一次 invalid；未保证送达，Core 必须保留 watchdog。

### 位置保持与设备检查

建立初始查询快照或收到首个完整报告后，摇杆静止时继续发布最后位置目标。原始 `sample_mono_us` 不变，另增加：

```json
"input_state": {
  "mode": "POSITION_HOLD",
  "device_connected": true,
  "checked_mono_us": 123456789
}
```

`checked_mono_us` 是设备查询成功且事件读取完成的 CLOCK_MONOTONIC 时间，不是硬件采样时间，也不能只因发送消息而刷新。尚未建立初始位置时仍 valid=false。存在事件积压或半帧时暂停续期；设备失效后继续发布 invalid，保留最后数值供显示。

Core 必须显式启用此策略（aviator_core_servo 已启用）：检查来源、会话、序号、有效位、设备检查时效及消息接收时效，允许轴事件时间较旧。通用 InputGuard 默认拒绝位置保持扩展；普通消息继续按原始事件时间过期。新网关和 Core 需一起更新并重启。

设备查询只能证明内核接口可用，不能证明摇杆固件仍在产生新硬件报告；设备固件卡死而接口可用需要额外硬件心跳才能识别。evdev 的事件变化与状态查询语义见 [Linux 内核输入事件文档](https://docs.kernel.org/input/event-codes.html)。

## FlightState 反馈

Gateway 接收通过发布者、时钟、序号和时效检查的 Core 反馈；旧 `core_session` 配置已忽略。节点启动标记保留用于记录与序号重置，不要求 UUID 或人工配对。

节点启动时生成普通文本标记；Core 重启后可接收其新鲜反馈，无需同步会话 ID。请求仍需新鲜 Core 状态及 source_authorized 确认。

Managed Core 的 flight.state.valid 表示设备状态及 Core 拥有线程更新新鲜，业务故障看 system.state；机械臂实时数据是否可用看 freshness.arm.valid。Rokae 未使能时也能识别 Core，不伪造 RT 采样。此处只消费系统摘要，不宣称完成双臂、双手、视觉业务 Schema 校验；USB 摇杆暂不向设备回传或发送震动，后续 RS422 回传使用独立 ICD。

## 测试与边界

```bash
ctest --test-dir build/communication --output-on-failure
```

测试覆盖键盘时间积分、双轴/反转/限位、松开回零、对向键、数字键映射和重复抑制、启动已按住、异常及配置校验，以及摇杆归一化、完整帧提交、静止位置保持、原始时间保留、设备检查停止/半帧/断开失效、异常时间、消息编解码与授权。机器人构建的 control_nodes_flight_hold 复用网关消息构造函数，通过真实 ZMQ 与无头 MuJoCo 验证长时间没有轴事件时仍能追到阶跃目标，并验证设备检查过期、网关静默、断开及恢复。真实 USB 查询与拔插仍需现场验证。

本次未连接实际键盘或摇杆硬件，设备权限、映射、发布抖动和持续操控需真机验证。当前不实现原始数据记录和 RS422 下行。安装入口为 `${CMAKE_INSTALL_BINDIR}/flight_gateway`。

## 摇杆按钮状态请求

按钮编号为设备声明的前 11 个 evdev 按键码（从 BTN_MISC 开始升序），启动时打印编号、实际 code 和事件映射；编号不保证与外壳标注一致。默认按钮 1–6 依次为 `enter_standby`、`grasp_wheel`、`start_control`、`exit_control`、`leave_wheel`、`reset_error`，7–11 未分配。仅这六个外部状态事件可配置，Boot/Done/Fault/SafetyLost/Emergency 不作为测试按钮接口。

直接编辑 flight.yaml 的 buttons 列表，例如把第 7 项改为 `exit_control`、第 11 项改为 `reset_error`；填 `none` 禁用对应按钮。所有配置均从 YAML 读取。


EV_KEY 按下沿在完整 SYN_REPORT 后触发一次；松开、自动重复、启动时已按住、超过 100 ms 的旧输入、SYN_DROPPED 不产生请求。连续轴值仍以 50 Hz 发布；反馈支持 INITIALIZING/READY/HOMING/RELEASING 状态；启动准备完成是 READY，按钮 1 回 home 后才是 STANDBY。

服务采用 DEALER → ROUTER，单帧 `ServiceRequest/1.0`，target=`aviator_core`、client_id=`flight_gateway`，parameters 为 `{"source":"JOYSTICK","button":1}`。不再发送或校验 server_session_id；回包按 request_id、client_id 和 server_id 关联。Managed Core 保留发布者白名单、请求期限、去重和状态机守卫。

最多保留 11 项未决请求，收发均非阻塞；无已连接服务时输出 NOT_SENT，不缓存到未来连接；已入 ZMQ 队列只代表 QUEUED，超时输出 UNKNOWN，不自动重试。每次新的物理点击是新请求；超时后需核对实际状态，不靠反复点击猜测是否执行。响应核对请求/客户端/目标身份，晚到及不匹配回包只记录，不当成本次成功。

Gateway 将请求副本发布至 `record.service.request`，完整响应发布至 `record.service.reply`；请求副本增加仅记录用的 `gateway_observation=QUEUED|NOT_SENT|TIMEOUT_UNKNOWN`，其余字段与原请求相同，超时记录沿用原 request_id。这些副本不用于执行。先启动 Bus 和 Logger 再测试；PUB/SUB 可能丢记录，没有持久化 ACK/补传。

Managed 真机入口及启动顺序见 [Core README](../aviator_core/README.md#managed-core摇杆按钮与连续目标驱动真机)。只有 CONTROL 状态使用连续轴值。Gateway/Core 可接收对方重启后的新鲜反馈，无需复制或固定会话；故障恢复仍受状态机和控制 epoch 约束。

可用 `ctest --test-dir build/communication -R '^service$' --output-on-failure` 验证模拟 ROUTER 应答、按钮事件和 MCAP 记录；`tests/managed_gateway_process_test.py` 用隔离 MuJoCo 验证真实 Managed 服务端和状态流程。未连接真实摇杆进行按钮标号验证。
