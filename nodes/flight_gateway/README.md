# flight_gateway

当前实现 USB Joystick → `flight.command`，并订阅 `flight.state` 的系统状态摘要。采用一个主循环，直接复用 common 的 ZMQ、消息编解码和 InputGuard；没有新增通信框架或后台线程。`main.cpp` 负责设备与循环，`gateway.hpp/.cpp` 负责采样、归一化和消息构造。

RS422 尚无 ICD，只保留 `--source rs422` 模式入口并明确拒绝启动；不猜测帧格式、波特率、校验或状态回传编码。协议冻结后，在这个节点内添加串口适配，不增加另一套网关进程。

## 构建和运行

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel

# 先在另一个终端启动总线。
./build/communication/bin/aviator_bus

# 使用默认 LiteStar PXN-F16 稳定路径，可省略 --device。
./build/communication/bin/flight_gateway \
  --device /dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick
```

使用 Linux evdev `/dev/input/event*` 或其稳定符号链接，**不是 `/dev/input/js*`**。省略 `--device` 时尝试 `/dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick`。默认或显式指定路径不存在、无权限、不是 evdev 设备或不支持所选轴时，会打印原因并提示输入新路径；可重复重试，空行或 EOF 退出。其他型号摇杆通过 `--device` 覆盖默认路径；非交互启动须确保设备路径有效。稳定链接可避免插拔后 event 编号变化。需要设备读取权限。默认轴为 ABS_X（0）、ABS_Y（1），启动时通过 ioctl 获取轴范围；不支持所选绝对轴或单调事件时钟的设备会提示重新输入路径。仅打开指定设备，不自动扫描或选择其他输入。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--source` | `joystick` | 当前仅此模式可用；`rs422` 明确返回未实现错误。 |
| `--device` | `/dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick` | evdev 设备路径；不可用时提示手动输入。 |
| `--publish` | `tcp://127.0.0.1:5555` | 连接 Bus XSUB 的入口。 |
| `--subscribe` | `tcp://127.0.0.1:5556` | 连接 Bus XPUB 的出口。 |
| `--roll-axis` / `--pitch-axis` | `0` / `1` | Linux ABS 事件代码，必须不同，不是 js 轴序号。 |
| `--invert-roll` / `--invert-pitch` | 关闭 | 反转对应归一化轴的方向。 |
| `--input-timeout-ms` | `100` | 原始输入超时，允许 1—100 ms；不超过架构初始 FlightCommand 超时。 |
| `--core-session` | 未配置 | 授权接纳的 aviator_core 会话 UUID；未指定时反馈显示 UNCONFIGURED。 |
| `--lock-file` | `/tmp/flight_gateway-<uid>.lock` | 单实例锁；退出释放锁但不删除文件。 |
| `--help` / `-h` | — | 查看用法。 |

启动后以最多 10 Hz 打印归一化控制值 `roll=... pitch=...`（范围 `[-1, 1]`，保留四位小数），终端中在同一行刷新，退出或输出其他日志前自动换行；重定向到文件或管道时逐行输出。不再打印 `input=VALID/INVALID`。首个完整输入报告前输出初始值 0，输入超时后显示最后采样值；控制消息的有效性与超时判定不变。

生产者固定为 `flight_gateway`，source 固定为 `JOYSTICK`。启动打印本次 session 和 clock_id，Core 应通过自己的授权流程接纳该会话；打印 STARTED 仅说明节点已初始化，不代表总线已连通或控制已获授权。

## 普通用户设备权限

出现 `Permission denied` 时，可使用根目录的 udev 配置脚本：

```bash
sudo ./scripts/setup_joystick_udev.sh --device /dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick
```

脚本自动读取 USB VID/PID，为当前 sudo 用户配置专用组的读取权限。执行后注销并重新登录，必要时拔插设备。支持 `--dry-run` 预览及 `--user` 指定用户，详见 [scripts 使用说明](../../scripts/README.md)。

## 输入及发布语义

- 设备事件通过 `EVIOCSCLOCKID` 指定为 CLOCK_MONOTONIC。EV_ABS 更新待提交轴值，SYN_REPORT 才提交完整快照，使用事件原始时刻而非读到事件的时刻。
- 有正负范围的轴以 0 为中心，分别按正负行程归一化；非负范围使用中点。输出范围为 [-1,1]，保留正负满行程。异常越界值拒绝，不静默截断。
- 启动时查询的轴值仅作为初始基线，在收到首个完整报告前 valid=false。安装方向与物理行程仍需台架校准，内核范围不代替标定。
- 按单调绝对期限以 50 Hz 发布；调度延迟时跳过错过的周期，不补发一串旧命令。每次发布递增 sequence，更新快照 timestamp，**保持原始 sample_mono_us**。
- 超时后保持最后 roll/pitch 供显示，但 valid=false。拔出、读取错误、事件丢失 SYN_DROPPED、倒退或未来时间及越界输入均锁存失效，继续发布 invalid，需要人工重启取得新会话；不自动重连使能。
- 每轮最多读取 128 个设备事件和 64 条总线消息，避免积压输入独占发布循环。ZMQ PUB 成功不等于送达。
- SIGINT/SIGTERM 通过 signalfd 在主循环处理，退出前尽力发布一次 invalid；未保证送达，Core 必须保留 watchdog。

### 静止输入的限制

Linux evdev 上报的是输入变化，设备静止时可能不产生新事件。本实现严格遵守架构“不用重发刷新旧样本”的要求，因此无新报告达到 100 ms 时也会失效，不能把打开的文件描述符或 ioctl 返回的驱动缓存视为新硬件样本。**当前路径适合事件采样和台架验证，尚不能保证静止持杆时持续有效。**

连续操控需要在实际设备上确认周期原始报告或独立链路存活机制，再明确如何将当前目标与源健康联合判定；不能仅延长网关超时绕过 Core 的原始数据年龄检查。事件分组和 SYN_DROPPED 含义依据 [Linux 内核输入事件文档](https://docs.kernel.org/input/event-codes.html)。

## FlightState 反馈

配置 `--core-session UUID` 后，仅接纳相同 clock_id、publisher_id=aviator_core、指定 session 的 `flight.state`；检查序号和 100 ms 时效。另检查所消费的 `system.state/current_error_code/last_error_code` 类型及范围。有效反馈的系统状态与 STALE 转换输出到 stdout，不用旧反馈冒充在线。

Core 重启后需显式更新授权会话并重启网关，不自动信任陌生 session。未配置 core-session 不影响 USB 命令发布，但反馈不标记有效。此处只消费系统摘要，不宣称完成双臂、双手、视觉业务 Schema 校验；USB 摇杆没有设计中定义的 FlightState 下行编码，暂不向设备回传或发送震动。后续 RS422 回传使用独立 ICD。

## 测试与边界

```bash
ctest --test-dir build/communication --output-on-failure
```

测试覆盖正负/无符号归一化、反向、完整事件帧提交、重发保留原时刻、超时/断开/丢帧失效、异常事件时间、命令编解码以及反馈会话与时效检查，并验证帮助入口和未实现 RS422 的拒绝行为。

本次未连接实际 USB 硬件，设备权限、映射、发布抖动和持续操控需真机验证。当前不读取占位 YAML、不实现原始数据记录和 RS422 下行。安装入口为 `${CMAKE_INSTALL_BINDIR}/flight_gateway`。
