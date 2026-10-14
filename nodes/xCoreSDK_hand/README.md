# xCoreSDK_hand

独立 SDK RS485 手节点，与 `manipulator` 分进程。控制策略直接复用
`nodes/rh56ftp_hand/rh56ftp_node.py` 的抓握、保持与故障判断，替换底层 HandLink 和 I/O 调度，
不另加保护、逐笔写入期限或故障锁定。
支持 NORMALIZED_POSITION、GRASP_SETPOINT、完整非触觉反馈、ACK、逐通道保持、原节点的故障及超时处理。
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

速度/力的 grasp、hold 值、threshold、closing_hold_ms、command_timeout_ms、feedback_timeout_ms、
state_period_ms 在两个节点中共用；显式命令行参数覆盖 YAML。
`state_period_ms` 控制状态发布周期，独立于设备 I/O。`position_write_hz`、`position_read_hz`、
`telemetry_read_hz` 分别控制位置目标执行、位置读取、其余遥测读取的频率上限，默认 20/10/5 Hz。
实际频率受 SDK 耗时限制，延误后不突发补发。连续位置目标仅保留最新待执行值，撤销指令保留执行。
总线主线程负责接收和发布；策略线程负责原有抓握/保持规则；左右手并行写入、各自读取。
SDK 同一只手的一次请求/应答串行，跨进程没有机械臂锁。仍先配置双手参数，再并行下发位置。
接收时验证指令来源和时间，超时监测使用最新已验证指令；执行完成后的 ACK 单独记录，
只有双手所需写入均成功才更新，接收或入队不代表设备成功。持续收到指令时，慢写不会导致误判断流。
Core 持续以 50 Hz 发布最新规划目标，手部反馈/ACK/目标完成超时只用于诊断，不导致停发或冻结规划进度。
Core 等待返回未确认不代表物理到位或抓稳；停止、撤销及真正断流时的原有处理保留。
真、fake 模式使用相同调度；Modbus rh56ftp_hand 的主循环不变。
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
脚本原有节点退出监测保留，不新增 Core SAFE/ERROR 跳转。
离线验证：`ctest --test-dir build -R 'xcore_hand|rh56ftp|scripts|core_hand' --output-on-failure`。
