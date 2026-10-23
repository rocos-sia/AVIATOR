# xCoreSDK_hand

独立 SDK RS485 手节点，与 `manipulator` 分进程。`policy.py` 复用 RH56FTP 的位置判据、
反馈和故障判断，并实现整手暂停和事件式动作。正式启动与握持测试 UI 使用同一策略。
支持 NORMALIZED_POSITION、GRASP_SETPOINT、完整非触觉反馈及执行 ACK。
同步规划仍由 Core 按 `hand_closing_distance` 完成。

```bash
./build/bin/xCoreSDK_hand --system-config config/system.yaml
./build/bin/xCoreSDK_hand --system-config config/system.yaml --fake-hand
# 可覆盖同一份手部配置，参数语义与 rh56ftp_hand 一致
./build/bin/xCoreSDK_hand --system-config config/system.yaml --config config/rh56ftp_hand.yaml
```

需要 Python 3、PyYAML、pyzmq；真实模式还需要构建的 `libaviator_xcore_hand_sdk.so`。
不需要 pymodbus，fake 模式不加载 SDK。启动器通过 exec 启动 Python，SDK 库加载在同一手进程中。
`rh56ftp_hand.yaml` 是唯一手部参数文件；`system.yaml.xcore_sdk_hand` 指定文件路径、启用及 fake 状态，
机器人地址来自 `robot.yaml.rokae.left_ip/right_ip`。手配置中的 `left_host/right_host` 仍只用于 Modbus。
发布者默认保留 `rh56ftp_hand`，禁止与另一同名手节点或 simulation 同时发布。

运动使用 `speed.grasp`、`force.grasp` 和模式 0。`hold` 参数为旧 Modbus 节点兼容保留，
SDK 整手暂停不写保持速度或力。`threshold`、`closing_hold_ms` 沿用共享配置。
`command_timeout_ms` 只校验新收到/排队待执行消息的年龄，不再要求上位机周期续发。
已接受动作由节点自主监测；静默不会自动张开。反馈超时仍按 `feedback_timeout_ms` 保护。

每个通道独立累计“位置偏差持续超阈值”或“完整窗口位置变化范围小于阈值”。
先满足的通道标记为 ready，继续保留运动目标；每只手六个通道全部满足后，向该手
**1634（0x0662）写单字节 1**。成功后标记 paused，保持期间只读反馈，不重复写暂停、
不发送 `-1` 或当前位置。左右手分别完成暂停。失败的暂停写入会重试，不提前报告成功。
暂停后的新动作直接写位置恢复；设备无需额外恢复寄存器操作。

`hand.command.action_id` 是可选的动作标识（非空字符串，最长 128 字符）。同一动作的
重试保持相同 ID；一次新的明确请求使用新 ID，因此相同位置也可重新运动。相同 ID 下
的目标变化可恢复已暂停的手；运动中的插值沿用原判据：目标变化重置稳定窗口，
同方向的持续位置偏差计时保留。无 ID 的旧
发布端可以改变目标恢复，但不能用相同目标重发来表达新的同目标动作。
`hand.state.command_delivery=event`，每侧新增 `paused`、`stop_criteria_met[6]`；
`closing_hold_active` 表示整手确实已暂停；`accepted_command.action_id` 回显动作身份。
ACK 序号/时间不随自主监测伪造刷新。反馈时间仍独立刷新。

Core 在 `xcore_sdk_hand.enabled=true` 时启用事件发送：新请求或目标变化后发送，
等待匹配发布者、会话、epoch、动作 ID 和序号的 ACK；确认后停止重复发送。
未确认命令按 50 Hz 上限重试，最长为 `core_hand.completion_timeout_ms`。
显式撤销发送 `valid=false`，也采用有界重试。手节点启动、正常退出、无故障的显式撤销
沿用安全张开；设备/反馈异常保护使用整手暂停（六通道独立故障确认后的安全张开规则保留）。
异常恢复不会自行重放故障前目标，需要新的动作。Core 进程意外消失与正常指令静默不再
通过动作频率区分；设备节点仍自主监测反馈。Core 等待返回未确认不代表抓稳。

`state_period_ms` 控制状态发布周期。`position_write_hz`、`position_read_hz`、
`telemetry_read_hz` 控制动作/保持检查、位置读取、其他遥测读取的频率上限，默认 20/10/5 Hz。
总线收发不等待 SDK I/O；同一端口串行，左右手并行。连续变化的待执行目标可合并，
撤销不能被合并丢弃。双手所需写入均成功才更新 ACK，入队不代表执行成功。
真、fake 模式使用相同事件策略和调度；独立 Modbus 与 CAN 节点保留原有流式协议。

手部 UI：`../../examples/xCoreSDK_RH56FTP_485/run_grip_demo.sh`，支持 `--fake-hand`、
`--side left|right|both`，只连接所选手；双手测试各用独立策略实例，单侧故障不阻断另一侧。

反馈时间标识为 host_rs485_read，使用位置与遥测中较旧的采样时间，避免旧遥测被伪装为新数据。
错误日志含控制器、寄存器和耗时。hand.state.scheduler 包含频率上限、接收/拒绝/合并计数。

手节点只调用 SDK 连接、RS485 超时设置和收发接口。没有上/下电、RT 模式、机械臂停止、
RS485 初始化或 disconnectFromRobot 调用。SDK 对象故意保留到进程结束，不运行其析构函数，
包括连接部分失败时；节点关闭仅停读写和执行原节点退出保护。操作系统回收进程内存和套接字。
这不保证控制器对网络断开没有自己的响应：双 SDK 客户端共存和独立进程退出的真机影响尚待验证。

一键启动创建 `hand.log`；退出先等待 Core 释放/停止，再等待手节点退出保护，最后释放机械臂进程。
SDK 使用独立的可写工作目录：一键启动放在本次日志目录的 `hand-sdk/xcore-hand-*` 下，
其中保留 SDK 自己的 `logs/`、`_rokae_log_/`；独立启动默认在临时目录创建。
可用 `--sdk-work-dir` 指定父目录，启动日志会打印实际路径。
输入配置和动态库路径先转换为绝对路径，再切换工作目录，因此原有相对路径命令仍可使用。
不与 sudo 机械臂进程共用仓库内日志；SDK 写日志失败会直接 abort，不能依赖 C++ 异常捕获。
脚本原有节点退出监测保留，不新增 Core ERROR 跳转。
离线验证：`ctest --test-dir build -R 'xcore_hand|rh56ftp|scripts|core_hand' --output-on-failure`。
