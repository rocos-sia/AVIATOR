# aviator_logger

独立 C++ 总线数据记录节点，从 XPUB（默认 `tcp://127.0.0.1:5556`）订阅全部 Topic，将通过公共协议校验的原始 JSON 字节写入单个 MCAP。存储封装位于 `common/recording.hpp/.cpp`，节点负责通信、队列和退出；不依赖机器人 SDK 或仿真库。

这是架构 P1 阶段的总线记录实现，**不等同于第 10 章完整原始数据归档系统**。PUB/SUB、总线 HWM、连接建立和关闭期间可能丢消息；记录始终标记 `completeness=unverified_pubsub`，不能凭文件可读或序号连续宣称端到端无损。

## 构建与启动

在仓库根目录执行：

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/communication --parallel 2
ctest --test-dir build/communication --output-on-failure

./build/communication/bin/aviator_bus
# 另一个终端；父目录须已存在。
./build/communication/bin/aviator_logger --output recording.mcap
# Ctrl+C 排空已入队消息并关闭文件。
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--output` | `aviator_DATE_TIME-UUID.mcap` | 输出路径；本地启动时间加随机 UUID。已有输出或 `.partial` 均拒绝覆盖。 |
| `--subscribe` | `tcp://127.0.0.1:5556` | XPUB TCP 订阅端点。 |
| `--session` | 新 UUID | Logger 记录会话标识，与消息内的源会话分开保存。 |
| `--queue-bytes` | `16777216` | 应用队列预算；按帧字节数加条目结构计费，不含分配器开销、ZMQ 队列和 MCAP Chunk。 |
| `--receive-hwm` | `4096` | SUB 接收 HWM，单位为消息。 |
| `--help` / `-h` | — | 打印用法。 |

接收线程独占 SUB socket，在收到两帧后记录 UTC 接收时刻并入队；写盘线程独占 MCAP Writer。队列满时丢弃新消息、累计 `queue_dropped`，不使用 latest-value 合并。接收和写盘都属于非实时域。

SIGINT/SIGTERM 停止接收，排空已入应用队列的数据，然后写索引与尾部、`fsync` 文件、读回检查索引和消息数、原子更名，最后 `fsync` 父目录。尚在生产者、网络或 SUB 内的数据不属于已入队范围，停止生产者与结束序号握手仍待实现。关闭读回耗时与文件大小相关；磁盘阻塞时没有硬性退出时限。

只在 Writer 成功打开后输出启动信息。创建、写入、同步、校验或更名失败返回非零，不打印成功统计；失败前的 `.partial` 保留，不自动修复或覆盖。若最后的父目录同步失败，最终文件可能已经存在，但程序仍报告失败。运行中没有周期性持久化承诺，断电可能丢失未关闭的 Chunk。

## MCAP 映射

- Schema：以 `类型/协议版本` 命名，`encoding=jsonschema`。目前仅约束公共头部，完整业务 Schema 尚未冻结；元数据明确标记 `schema_scope=common_header_only`。
- Channel：按 Topic、版本、`publisher_id`、源 `session_id`、`clock_id` 区分，Topic 保持原名；不会把后来来源错标为第一个发布者。
- Message：原始 Frame1 字节逐字节保留，包括空白、扩展字段与 `valid=false`。`sequence` 取源序号低 32 位，完整序号仍在 JSON 中。
- `log_time`：接收时的 Unix UTC 纳秒，不人为修正系统时钟回退。当前时钟接口精度为微秒。
- `publish_time`：按 SAD §10，在没有明确发布 UTC 时取 `log_time`；原始 `timestamp` 是快照时间，只保留于消息中。
- Chunk：4 MiB 目标大小，无压缩；启用 Chunk、Data、Summary CRC 和默认消息/Chunk 索引。关闭读回检查结构及计数，不是独立的全文件 CRC 审计。
- Metadata：保存 Logger 会话、消息/Channel 计数、接收时间范围、逐 Topic 统计、无效/拒绝/队列溢出计数和完整性标记。

按 Topic、publisher、源会话的完整序号高水位统计观察到的 `sequence_gaps` 和 `duplicate_or_reordered`，重复及乱序消息仍原样记录。缺口计数不因晚到消息回补，因此表示接收时观察到的前向跳号，而非最终丢失量；首尾区间未知，尚无生产者首尾清单对账。Schema/Channel 达到 MCAP 的 16 位 ID 上限时明确失败。

## 当前边界

尚未实现的架构升级项：`5557` 原始数据入口与冻结 Protobuf 信封、`record.invalid` 原始坏帧留存、`record.ingest` 单调接收时序、分卷、Zstd 压缩、带哈希的外部会话清单、配置/标定附件、周期性持久化、状态发布和崩溃恢复。

未知 Topic 或 JSON 校验失败目前计入 `invalid` 后跳过；非法 multipart 计入 `rejected`（超长 multipart 按每次受限排空拒绝计数）。超过 ZMQ 单帧安全上限（64 KiB）的消息可能在传输层被拒绝，不能承诺应用侧逐条计数。以上限制与统计一起用于判断本次记录是否适合分析。

## 验证

`recording` 测试覆盖已注册的 11 个 Topic、原始字节、来源与版本隔离、序号低 32 位、UTC 回退、无效数据、TCP 订阅、队列溢出、空文件以及已有文件/残留文件保护。`logger_process` 在 Python 可用时使用标准库验证 SIGINT/SIGTERM、创建失败和文件大小限制触发的写盘失败；Python 不参与运行节点。

MCAP 已在 `third_party/mcap-2.1.3/` 固定为 2.1.3，包含许可证；与 `examples/mcap_recording/third_party/mcap/` 副本一致。示例保留自包含构建，正式节点通过 `aviator_recording` 链接唯一实现翻译单元。
