# MCAP 全量记录 + 事后同步 示例

一个自包含的最小示例，演示文档 §10「MCAP 全量记录」的端到端流程，供你调整、测试，之后可升级为正式的 `nodes/aviator_logger`。

## 做什么

1. 造 3 个 topic 的合成消息（`flight.command`@50Hz / `arm.state`@100Hz / `camera.detection`@30Hz，共 2 秒）。
2. 按 §10 映射写入本地 MCAP：Schema=jsonschema、Channel=原 Topic、Message 带 `log_time`/`publish_time`/`sequence`。
3. 用 `.mcap.partial` → 原子 rename 落盘。
4. 读回并校验「无损」：消息总数、各 channel 计数、内容逐字节一致、每 channel 序号单调、Schema 齐全。
5. 打印 `PASS`/`FAIL`，失败返回非零退出码（可接 ctest）。

## 构建与运行

```bash
cmake -S . -B build
cmake --build build
./build/mcap_recording_example recording.mcap   # 不传参数默认输出 recording.mcap

# 或作为测试跑
ctest --test-dir build --output-on-failure
```

## 事后同步到远端

```bash
AVIATOR_REMOTE_HOST=user@host AVIATOR_REMOTE_DIR=/data/aviator \
    ./sync.sh recording.mcap
```

`sync.sh` 用 `rsync --checksum` 推送 + 远端 `sha256sum` 对账，传输被截断/丢字节会报 `MISMATCH` 并以非零码退出。

## 关于「远程传输」的选型结论

- **无损记录走本地 MCAP 落盘（权威）**，不押在会丢包的链路上。
- 远端**实时观测** → PUB/SUB 桥（丢帧可接受），见 SAD §03 telemetry_bridge。
- 远端**无损存档** → 本示例的「本地记录 + 事后同步」，或 ZMQ PUSH/PULL（链路断会反压/卡生产者）。
- 本示例采用最稳的「本地记录 + 事后同步」。

## 依赖

- 仅 C++17 标准库 + 头文件版 MCAP（vendor 在 `third_party/mcap/`，压缩 zstd/lz4 已用编译宏关闭）。

## 后续升级路径

- 数据源从合成消息换成订阅真实总线（连接 XPUB），或走独立记录通道 `tcp://127.0.0.1:5557`。
- Schema 换成 `schemas/` 里的真实 JSON Schema。
- 加入分卷（1 GiB / 60 s）、Chunk 索引、CRC、Zstd 压缩、`.partial` 原子更名 + 会话清单。
- 写入逻辑抽到 `common/recording.hpp/.cpp`，节点抽到 `nodes/aviator_logger/`。
