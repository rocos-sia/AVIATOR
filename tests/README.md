# 通信测试

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
