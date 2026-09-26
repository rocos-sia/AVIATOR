# 公共通信代码

当前实现三个小库，节点按需链接；直接使用 cppzmq socket，不增加连接管理器、消息路由框架、后台工作线程或业务节点基类。

| Target / 文件 | 已实现功能 |
| --- | --- |
| `aviator_protocol` / `protocol.hpp/.cpp` | 八个主 Topic 与三个 system Topic 的精确映射；强类型公共 Header/Origin；JSON 编解码、大小/深度/重复键/类型/数值/版本检查；FlightCommand 来源与 roll/pitch 范围检查；运动命令溯源字段检查。 |
| `aviator_transport` / `transport.hpp/.cpp` | socket 选项、订阅、两帧非阻塞收发；异常多帧有界排空；XSUB/XPUB 阻塞代理；生产与回放端点常量。 |
| `aviator_runtime` / `runtime.hpp/.cpp` | UTC/单调时钟、Linux 启动时钟域与 UUID；显式授权会话的 InputGuard；序号、原始年龄、origin 和 epoch 检查；非实时 LatestMailbox。 |

依赖为 libzmq、仓库 `cppzmq-4.11.0`、nlohmann/json ≥3.10、Threads。通信层与 YAML、机器人 SDK、运动学、日志存储无依赖。

## 使用方式

在节点 CMake 中按需添加：

```cmake
target_link_libraries(your_node PRIVATE aviator_transport aviator_runtime)
```

节点拥有 `zmq::context_t context{1}`，允许同进程线程共享 context；每个 socket 必须在一个固定线程创建、使用、关闭。先 `configure()`、`subscribe()`，再 bind/connect。配置默认 HWM=8、LINGER=0、非阻塞收发、CONFLATE=0；代理两侧 HWM=64。

发布示意：

```cpp
#include "protocol.hpp"
#include "runtime.hpp"
#include "transport.hpp"

zmq::context_t context{1};
zmq::socket_t pub{context, zmq::socket_type::pub};
aviator::configure(pub);
pub.connect(aviator::publish_endpoint);

// 以下身份在进程启动时生成一次，后续每条消息复用。
const auto session = aviator::new_session_id();
const auto clock = aviator::local_clock_id();
aviator::Message message;
message.topic = aviator::Topic::flight_command;
message.header = {"1.0", 1, aviator::utc_us(), aviator::monotonic_us(),
                  clock, "flight_gateway", session, true};
message.body = {{"source", "JOYSTICK"},
                {"control", {{"roll", 0.0}, {"pitch", 0.0}}}};
std::string payload, error;
if (!aviator::encode(message, payload, error)) {
    // 记录 error，本条消息不发送。
} else if (!aviator::send(pub, aviator::topic_name(message.topic), payload)) {
    // 发送暂不可用，计数后进入下一发布周期；不能当作送达确认。
}
```

后续 sequence 按 Topic 递增。示例使用当前时间模拟一次新采样，真实设备必须填原始采样时刻；重发旧值不能刷新 sample_mono_us。会话/时钟 ID 不在每次发送时生成。encode/decode 返回 false 时保留输出参数，错误写入 error；socket 错误通过异常交由节点生命周期处理。

订阅侧持有一个 `ReceiveState` 和 `WireMessage`，循环调用 `receive()`：

1. `empty`：本次无消息，用 `zmq::poll` 或调度器等待，避免忙循环。
2. `rejected`：格式非法或排空预算耗尽，记录拒绝并在下一接收轮继续；不要重置 ReceiveState。
3. `received`：调用 `decode(wire.topic, wire.payload, message, error)`；它会精确校验 Topic，不能仅信任 ZMQ 前缀订阅。
4. 节点检查业务必填字段、支持模式、数组形状、标定、硬件范围等，然后调用 `InputGuard::accept()`；成功才更新该 Topic 的缓存。
5. 无消息期间仍定期调用 `InputGuard::expired(now)`，不能只在收到新消息时检查超时。

每轮接收还须由节点设置消息数或时间预算。`receive()` 每次最多消费 32 帧，非法多帧尾部通过 ReceiveState 跨轮丢弃；输出只在 `received` 时改变。发送中途异常后重建 socket，不能继续沿用不确定的 Multipart 状态。maxmsgsize 是单帧传输限制，应用层排空预算不等同于 libzmq 内部的整条多帧内存配额，入口仍应仅允许受控发布者。

## 时效与控制授权

`InputGuard` 只保留一个授权来源的计数和时间，不维护无界会话表。构造时指定 Topic、publisher_id、session_id、clock_id、timeout_us；FlightCommand 还指定 source，臂手命令还指定 control_epoch 和上游来源/会话/超时。它在成功解码和业务校验之后运行，不能替代节点业务校验或网络身份认证。

同一序号、旧会话、跨时钟域、未来/过期样本、未授权 epoch 或过期 origin 均不刷新有效期限。合法的 valid=false 报告推进已处理序号并立即撤销可用性，但不更新最后有效数值。超时按 `age >= timeout` 判断。UTC 不参与 watchdog；当前只支持同一单调时钟域。

新会话和新 epoch 必须由节点的受控授权流程决定，清理 mailbox 并重建 InputGuard；不会根据新消息自动切换。InputGuard 由一个线程独占；要供算法线程消费，应在节点层发布带有效性和时间的强类型快照。

`LatestMailbox<T>` 是互斥锁保护的单值拷贝，仅用于非实时线程，一种 Topic/授权来源一个实例。它不是伺服域的无锁队列，不能把含 JSON 的 Message 或该 mailbox 直接用于硬实时循环。RT 交接和设备本地安全检查在 manipulator 中实现。

## 总线与退出

`run_bus(context)` 在调用线程内创建 XSUB/XPUB 并运行 `zmq::proxy()`，只转发帧与订阅，不解析业务。由控制线程调用 `context.shutdown()` 中断代理；函数将 ETERM 作为正常退出，其他 bind/转发错误向调用者抛出。shutdown 作用于整个 context，因此 Bus 进程使用自己的 context。控制线程再 join 代理线程，最后销毁 context。

`run_bus` 的可选 on_ready 回调在两个 bind 均成功后执行。独立入口见 [aviator_bus](../nodes/aviator_bus/README.md)，该节点负责命令行参数、单实例锁、stdout 就绪提示和信号退出；公共库本身不启动常驻进程。回放端点常量不代表已实现回放授权，生产/回放隔离由对应节点配置验证。

## 当前边界

Message 的 body 保留 JSON，完整业务 Schema 与各业务强类型模型尚未冻结。本次只实现通信公共校验以及已有明确规则的 FlightCommand 与 origin/epoch 必需字段；其他消息的业务字段、模式和自由度由节点验证，不能把 decode 成功理解为可直接执行。

暂未实现可靠服务、system.* 业务内容校验、MCAP/Protobuf 记录通道、跨主机时钟映射和 RT SPSC 队列；这些功能不以空壳 API 占位。接口依据详见 [ZMQ 协议格式说明](../docs/AVIATOR_ZMQ协议格式说明.md)。

通信独立构建及测试见 [构建系统说明](../docs/构建系统说明.md)。
