# aviator_logger

独立 C++ 数据记录节点：订阅总线（XPUB 出口 `tcp://127.0.0.1:5556`），把全部业务 JSON 消息按 SAD §10 的映射无损写入**单个** MCAP 文件。当前为最小实现（单文件、不分卷）；分卷、会话清单、状态机（READY/RECORDING/DEGRADED/ERROR）与原始记录通道（`tcp://127.0.0.1:5557`）是后续升级项，见 `examples/mcap_recording/README.md` 的「后续升级路径」。

## 构建与启动

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel
./build/communication/bin/aviator_bus                        # 先起总线
./build/communication/bin/aviator_logger                     # 默认按启动时间命名
```

参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--output` | `aviator_YYYY-MM-DD_HH-MM-SS.mcap`（启动本地时间） | 输出 MCAP 路径（先写 `.partial`，成功后原子 rename）。父目录须已存在。 |
| `--subscribe` | `tcp://127.0.0.1:5556` | XPUB 订阅端点。 |
| `--session` | 新 UUID | 会话标识，写入 MCAP metadata 与各 channel metadata。 |
| `--help` / `-h` | — | 打印用法并退出。 |

## MCAP 映射（SAD §10）

- **Schema**：每个消息类型一个，`encoding=jsonschema`；真实 Schema 尚未冻结（`schemas/` 为空），当前生成仅约束公共头部的占位 Schema。
- **Channel**：每个 Topic 一个，`message_encoding=json`，metadata 携带首条消息的 `publisher_id`/`session_id`/`clock_id`。
- **Message.data**：原始 Frame1 字节，不重编码（保证无损）；`log_time`=接收 UTC 纳秒；`publish_time`=源 `timestamp`(μs)×1000；`sequence`=源 sequence 低 32 位。

## 生命周期

接收线程独占 ZMQ SUB socket 与 MCAP Writer，主线程用 `sigtimedwait()` 等待 SIGINT/SIGTERM。收到信号后停止接收、关闭 Writer、把 `.partial` 原子改名成最终文件，并打印统计（消息数 / 无效数 / 被拒数 / 通道数）。Writer 打开或写入失败会抛异常并返回非零退出码，不静默丢数据。

解码失败（未知 Topic、非法 JSON 等）的消息当前仅计数跳过，不落盘；完整版将按 SAD §10 写入 `record.invalid` 二进制信封。Logger 订阅空前缀（全部 Topic），未识别的 Topic 由 `decode()` 拒绝并计入 invalid。

## 验证

无损写盘 + 读回校验的端到端示例见 `examples/mcap_recording`；本节点的写盘逻辑即由该示例升级而来。
