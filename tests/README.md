# 通信测试

`rokae_impedance_hold` 使用真实 RokaeDataLink 和模拟 SDK，分别在暂停、设置刚度、恢复首帧前
注入位置漂移，验证暂停前旧目标已平滑更新为实测角度且 SDK 回调保持配置时长（默认 500 ms，另测 700 ms），
并拒绝非有限或非正驻留时间；
保持期间取消或反馈过期不能继续切换刚度；恢复时两臂首条目标等于实测位置、连续切换
不重发旧目标，恢复首帧速度为
0.03 rad/s 时仍同步实测保持目标，非有限速度反馈仍拒绝并停止双臂；测试不连接硬件。
`impedance_settling` 验证切换前的连续停稳条件。`impedance_lifecycle` 还验证实际保持目标原样返回
Core、无独立测量时不拟合轮盘零位、仿真轮盘测量不被缓存覆盖，以及相同刚度 profile
切换仍重建保持。`servo_planner` 验证正负非零位置接管后回到固定中位、逐毫秒轨迹
连续性、限 jerk 接管、取消期间不接受新目标，以及 2026-10-15 实机日志中约 4 cm
几何残差姿态的直接接管、共用 CONTROL 响应参数、首帧无跳变、途中换目标和最终非零目标 FK。`grasp_tools` 同时检查
仿真 handle 与 TCP 使用配置中的标定，避免 MJCF 旧抓握点被永久伺服偏移掩盖。

`tts_rules` 验证 `aviator_tts` 的状态去重、实际状态码、时效、初始告警、错误变化、
启动宽限、失联恢复、实例重启、来源冲突和队列过期。`tts_process` 用真实 loopback ZMQ、
ALSA null 设备与可控播放器验证缓存原子更新、文案失配/损坏拒绝、抢占、单实例与信号回收；
需要 Python/PyYAML 和 `aplay`，无需模型、pyzmq、机器人或扬声器。
运行 `ctest --test-dir build/communication -R '^tts_' --output-on-failure`。

`motion_protocol` 覆盖 81 点 Servo 双臂 q/dq/ddq 完整报文（超过旧 64 KiB 限制）、
128 KiB 报文预算、超出窗口上限的拒绝，以及普通轨迹原有 32 点/63 样本限制。
`control_nodes_servo_window` 使用独立端口和 MuJoCo，验证 42 ms 补窗间隔后的连续恢复、
显式 stop 后超过 1 s 的使能保位、从停止目标重启，以及持续断流仍触发 50 ms watchdog。
`managed_gateway_process_test.py` 验证运动期间输入失效后进入 ERROR、稳定保位、无自动恢复，
以及复位到 READY 后显式释放回到 STANDBY；预期故障注入之外的 ERROR 都会失败。`core_hand_process_test.py` 的 revoke
场景还检查没有运动任务时撤销授权也会请求本地 stop。

`grasp_tools`（完整机器人构建）验证不同左右工具变换、直接抓握目标、70/40 mm 独立触发与
平滑闭合终点、真实 Pinocchio FK/IK、旧共享配置兼容、缺失/混合配置拒绝、碰撞模型加载与
MuJoCo TCP；无需连接真机。`control_nodes_grasp` 运行真实总线和 MuJoCo，覆盖使能、从 home
连续接近、锁定、张开、失能和退出。
`grasp_tools` 还注入释放阶段 FK 失败，刻意让执行器先进入 FAULT，再运行状态机监督，
验证原始错误被保留并进入 ERROR，避免资源失效掩盖工作线程异常。

`camera_grasp_input` 使用独立进程内 ZMQ 发布者，验证抓握来源配置、按需订阅、拒绝请求前旧帧和
其他相机、重复读取、等待超时与取消；`camera_wheel_input` 验证已确认的坐标换算，并区分 Servo
限幅与抓握越界拒绝，并确认轴匹配诊断不拦截抓握。`grasp_tools` 覆盖 Direct/Managed 初始化不采集、抓取后采集、
重抓刷新、双臂 IK/碰撞/闭手共用相机位形、设备参考同步、后续 MoveWheel 起点以及失败不回退。

`core_hand` 验证接近期间的双手目标插值、超过普通完成时限的持续运动、终点 ACK 与取消。
`core_hand_process_test.py` 的 `synchronized` 场景使用真实 ZMQ 和模拟设备反馈，冻结机械臂
游标 300 ms，确认手指目标不按墙钟时间继续闭合，并验证恢复执行与最终锁定。

`robot_state_machine_hand_overcurrent` / `robot_state_machine_hand_overtemperature`
使用隔离 ZMQ 总线、MuJoCo 双臂及模拟 `rh56ftp_hand` 发布者，完成真实托管 Core 的
`FOLLOWING → RELEASING → STANDBY`。分别注入错误码 4（过流）、2（过温），顶层和左右手
`valid=false`；模拟驱动 800 ms 后恢复运动，反馈有效性继续保持无效至 2 s。
验证无效期间持续收到张开目标（间隔小于 100 ms）、左右手实际模拟位置张开，反馈恢复后
完成释放。测试把位置容差设为 0.03，保留临时目录内的 `hand-zmq.jsonl` 完整手部消息和
`hand-fault-result.json` 结果；不连接串口或部署总线。

`robot_state_machine_hand_latched-error` 在上述过程中永久保留 `hold_control_error`，
验证设备恢复后 Core 接受有效测量、完成张开，不进入 ERROR/ERROR；同时检查释放期间
`arm.command` 持续发布。该场景模拟驱动已经恢复执行目标，不能替代驱动故障锁存恢复测试。

`communication_test.cpp` 经 CTest 注册为 `communication`，不依赖机器人或相机硬件，使用实际 libzmq socket。

覆盖公共消息编解码、Topic 精确匹配、异常 JSON/重复键/UTF-8/大小/深度/整数边界、版本与 UUID、FlightCommand 范围、会话和 epoch 授权、旧样本/旧 origin/未来时间/序号及超时边界、无效报告、非实时 mailbox 并发读写、TCP 两帧收发与跨轮非法帧排空、XSUB/XPUB 转发及 context 退出。

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel
ctest --test-dir build/communication --output-on-failure
```

需要 libzmq 和 nlohmann/json 开发依赖。完整业务 Schema、设备限制、记录/回放及硬实时性能测试随相应实现补充；当前测试通过不代表具备真机控制条件。

`bus_test.cpp` 注册为 `bus_process`，启动真实 aviator_bus 子进程，验证 TCP 转发、端口冲突、绑定失败后的资源释放、单实例锁、信号退出和重启。`bus_help` 检查帮助入口。测试进程异常时会清理其子进程，使用独立临时锁文件。

`gateway_test.cpp` 检查 USB 事件快照、轴归一化、原始时间保留、过期/断开/丢帧失效以及 FlightState 系统摘要、会话和超时。另有网关帮助、缺少设备参数、RS422 尚未实现时拒绝启动的命令行测试；不需要实际 USB 硬件。

`monitor_state` 覆盖只读监控统计与有界缓存；`monitor_http`（Python3 标准库，仅测试依赖）启动真实 HTTP 服务与 C++ TCP 发布者，验证页面、接口、慢连接、过期、动态配置保存/来源切换、失败回滚、版本冲突、重启持久化和退出。`monitor_config` 覆盖 YAML 解析、类型往返与原子保存；`monitor_preview_receiver` 使用隔离 TCP 端口验证图像订阅启用、地址与身份切换、关闭和重新启用。配置页浏览器测试见 `monitor_config_browser_test.cjs`，需要 Node ≥18、playwright-core 和 Chrome。

## Logger 记录验证

通信构建包含 `recording`、`logger_help` 和 `logger_invalid_queue`；发现 Python 解释器时还运行 `logger_process`（仅使用标准库）。覆盖 MCAP 读回、全部已注册 Topic、多来源/版本映射、TCP 订阅和有界队列，以及信号关闭、输出冲突与写盘失败。测试使用独立临时目录和动态 TCP 端口，无需机器人硬件。

`camera_recording` 新增三模式、相机双流 TCP→MCAP→解码验证（软件 libx264/libx265、Zstd）、非法输入和图像队列溢出；不依赖相机/GPU。`camera_adapter` 使用 Python 标准库验证 Python/C++ Protobuf 信封互通。`logger_process` 同时验证 YAML 与 CLI 优先级。真实 D436、NVENC 和高分辨率持续带宽需另行实机验收。

`monitor_http` 包含超过 2 s 的大网格慢接收完整性检查。`monitor_remote_model_browser_test.cjs` 使用非 localhost HTTP 域名、限速和延迟验证完整模型加载与同源资源请求；依赖 Node ≥18、playwright-core 和 Chrome，运行方式见 Monitor README。

`rs422_process` 还验证真实双 PTY 接收字节完整附加到 `flight.command.rs422`（包含非零保留位）、选用通道与控制样本时间对应、A→B 切换及坏校验帧不会刷新原始报文；操作与空闲帧同批到达时仍保留操作，再次发送相同操作不继承旧请求 ID。
`monitor_state` 覆盖可选附加数据、原始接收时间超时、跨时钟及损坏附加字段的隔离处理，以及保留帧校验、精确请求关联、等待／拒绝结果与新操作清除旧应答。
`node tests/monitor_rs422_browser_test.cjs build/bin/aviator_monitor` 使用隔离 ZMQ 发布者与真实 Monitor，验证无 RS422 的空状态、原始 HEX／字段含义、操作帧保留与替换、实时刷新、Core 等待／拒绝结果、页面切换／刷新、暂停／复制、报文过期、HTTP 断流、未知时钟、异常附加数据和布局；需要 Node ≥18、playwright-core、Chrome 和 Python pyzmq，不需要串口硬件。
