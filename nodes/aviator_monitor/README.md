# aviator_monitor（Web UI）

只读系统监控：C++17 节点订阅 ZMQ，总线消息保存在有界缓存；浏览器通过本地 HTTP 查看。无 Qt、Node.js 或外部前端依赖，HTML/CSS/JavaScript 在构建时嵌入可执行文件。

## 构建与启动

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel

# 在另一个终端运行总线和需要观测的节点。
./build/communication/bin/aviator_bus

# 启动监控，浏览器访问 http://127.0.0.1:8081
./build/communication/bin/aviator_monitor
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--port` | `8081` | 本地 Web 服务端口，1—65535。 |
| `--subscribe` | `tcp://127.0.0.1:5556` | Bus XPUB 订阅出口；仅支持 TCP。 |
| `--help` / `-h` | — | 查看用法。 |

回放观测示例：

```bash
./build/communication/bin/aviator_monitor --port 8082 --subscribe tcp://127.0.0.1:6556
```

HTTP 固定绑定 127.0.0.1，不开放控制发布端口。SIGINT/SIGTERM 正常退出。无需总线先在线，ZMQ 可以等待连接；网页“监控服务在线”仅代表 HTTP 可用，不等同于总线或设备健康。

## 页面内容

- 整机运行状态、控制来源、当前/历史错误码；仅在恰好一个新鲜 FlightState 来源时显示，否则标记无有效反馈或来源冲突。
- arm、hand、camera 聚合子状态的当前年龄与过期标记，避免顶层新时间掩盖旧设备反馈。
- 各 Topic、生产者、会话、接收频率、原始样本年龄、距最近接收时间、完整序号、序号缺口和重复/乱序计数。
- 点击数据行查看完整原始 JSON，包括关节、TCP、视觉和诊断字段。双臂双手数据当前通过 JSON 明细查看，不提供机器人三维显示或曲线。
- 显示最近接收异常、拒绝计数和缓存淘汰计数。system.event 保留每个来源的最后事件，不是事件历史或持久化日志。

页面每 100 ms 请求一次快照，同一浏览器不叠加未完成请求。断开 HTTP 后清除当前状态和数据表，已显示 JSON 明确标记为旧快照。消息文本通过 textContent 显示，不作为 HTML 执行。

## 时效与统计

| 标记 | 含义 |
| --- | --- |
| FRESH | 公共头部 valid=true，原始采样与接收间隔在监测阈值内。 |
| STALE | 原始采样或最后接收已超时。 |
| INVALID | 尚未超时，但生产者标记 valid=false。 |
| STALE_ORIGIN | 臂/手命令自身新鲜，但上游 origin 已超时。 |
| CLOCK_UNKNOWN | 采样/来源时钟域不同，不能直接计算单向年龄。 |
| FUTURE | 同时钟域采样时刻异常未来。 |
| EVENT | 最后一次 system.event，不赋予持续健康含义。 |

监测阈值：flight.* 为 100 ms，arm.*、hand.* 为 50 ms，camera.* 为 200 ms，system 状态/诊断为 2 s。camera.command 和 system.* 阈值只是本监控显示约定；不替代业务节点的运行配置。错误码及 valid 均保留生产者含义，FRESH 不代表硬件可用、动作成功或控制已授权，也不表示完整业务 Schema 已验证。

采样年龄只在本机 clock_id 匹配时计算。FlightState 子组当前年龄=发送时 age_ms+聚合快照已经过去的时间；跨时钟域或未知年龄不伪造为 0。

频率采用最近 2 秒接纳消息数除以 2，启动前两秒逐渐增长。这是监控端接收速率，不能据此证明源端精确发布率。重复/乱序计数不会刷新最新快照或有效期；缺口统计不包含首次订阅前丢失的数据。每个 `(Topic, publisher_id, session_id)` 独立统计，新会话新增一行，旧会话自然过期。

## 实现与接口

- `main.cpp`：独立 SUB 线程、单线程非阻塞 HTTP 服务、退出处理。
- `monitor.hpp/.cpp`：公共协议校验、状态缓存及统计，不发送控制数据。
- `index.html`：原生浏览器界面，无 npm 构建步骤。
- `GET /` 或 `/index.html`：页面。
- `GET /api/state`：摘要列表及拒绝/淘汰统计。
- `GET /api/message?id=N`：指定数据流的最新原始 JSON；ID 来自摘要，不是 Topic ID。

最多缓存 64 条数据流，满时淘汰最久未更新者；每流只留一个消息及最多 512 个接收时刻。接收端每轮最多 128 条消息或 5 ms。HTTP 最多 8 个同时连接、请求头最多 4096 字节、整个请求最长 2 s；慢浏览器不阻塞 SUB 接收。HTTP 不提供修改接口、长连接、账户系统或历史查询。

## 测试

```bash
ctest --test-dir build/communication --output-on-failure
```

`monitor_state` 验证时效、跨时钟域、缺口、重复及缓存界限；`monitor_http` 使用真实 TCP 发布者和 HTTP 子进程验证数据、原始消息接口、慢连接、断流和退出（仅测试阶段需要 Python3 标准库）；`monitor_help` 验证入口。另已使用无头 Chrome 实际渲染实时状态页面。运行节点没有 Python 依赖。

启动成功后，终端会提示 `请在浏览器打开：http://127.0.0.1:8081/`；使用 `--port` 时显示实际配置端口。
