# 通信测试

`grasp_tools`（完整机器人构建）验证不同左右工具变换、直接抓握目标、70/40 mm 独立触发与
平滑闭合终点、真实 Pinocchio FK/IK、旧共享配置兼容、缺失/混合配置拒绝、碰撞模型加载与
MuJoCo TCP；无需连接真机。`control_nodes_grasp` 运行真实总线和 MuJoCo，覆盖使能、从 home
连续接近、锁定、张开、失能和退出。

`core_hand` 验证接近期间的双手目标插值、超过普通完成时限的持续运动、终点 ACK 与取消。
`core_hand_process_test.py` 的 `synchronized` 场景使用真实 ZMQ 和模拟设备反馈，冻结机械臂
游标 300 ms，确认手指目标不按墙钟时间继续闭合，并验证恢复执行与最终锁定。

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

`monitor_state` 覆盖只读监控统计与有界缓存；`monitor_http`（Python3 标准库，仅测试依赖）启动真实 HTTP 服务与 C++ TCP 发布者，验证页面、接口、慢连接、过期和退出。

## Logger 记录验证

通信构建包含 `recording`、`logger_help` 和 `logger_invalid_queue`；发现 Python 解释器时还运行 `logger_process`（仅使用标准库）。覆盖 MCAP 读回、全部已注册 Topic、多来源/版本映射、TCP 订阅和有界队列，以及信号关闭、输出冲突与写盘失败。测试使用独立临时目录和动态 TCP 端口，无需机器人硬件。

`camera_recording` 新增三模式、相机双流 TCP→MCAP→解码验证（软件 libx264/libx265、Zstd）、非法输入和图像队列溢出；不依赖相机/GPU。`camera_adapter` 使用 Python 标准库验证 Python/C++ Protobuf 信封互通。`logger_process` 同时验证 YAML 与 CLI 优先级。真实 D436、NVENC 和高分辨率持续带宽需另行实机验收。
