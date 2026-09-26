# aviator_bus

独立 ZMQ 总线进程，直接调用 `common/transport` 的 `run_bus()`，通过 XSUB → proxy → XPUB 转发消息和订阅，不解析 JSON、不仲裁控制。

## 构建与启动

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel
./build/communication/bin/aviator_bus
```

开发依赖见 [构建系统说明](../../docs/构建系统说明.md)。默认 PUB 发布者连接 `tcp://127.0.0.1:5555`，SUB 订阅者连接 `tcp://127.0.0.1:5556`。

参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--input` | `tcp://127.0.0.1:5555` | XSUB bind 地址。 |
| `--output` | `tcp://127.0.0.1:5556` | XPUB bind 地址，必须与 input 不同。 |
| `--lock-file` | `/tmp/aviator_bus-<uid>.lock` | 当前用户持有的单实例文件锁。父目录须已存在。 |
| `--help` / `-h` | — | 打印用法并退出。 |

仅接受 TCP 端点；地址格式和端口占用由 libzmq 在 bind 时检查。默认回环地址；指定其他网卡地址时由部署层配置访问限制。

隔离回放示例：

```bash
./build/communication/bin/aviator_bus \
  --input tcp://127.0.0.1:6555 \
  --output tcp://127.0.0.1:6556 \
  --lock-file /tmp/aviator_bus-replay.lock
```

生产与回放使用不同端点及锁文件。锁文件不随退出删除，内核在文件描述符关闭或进程退出时释放锁；残留文件不代表仍在运行，不应在运行期间删除锁文件。

## 生命周期

一个 context 使用 1 个 I/O 线程。代理线程创建、使用、关闭两个 socket，主线程用 `sigtimedwait()` 等待 SIGINT/SIGTERM，收到后调用 `context.shutdown()` 并等待代理退出。信号在创建线程前阻塞，异步 signal handler 内不调用 ZMQ。

两个端点均 bind 成功后，标准输出打印并刷新：

```text
READY input=tcp://127.0.0.1:5555 output=tcp://127.0.0.1:5556
```

READY 只表示本地端口绑定成功，不代表订阅传播完成或设备可以执行；生产者仍须周期发送，消费者仍须检查新鲜数据。任一 bind 失败、锁冲突或参数错误会输出 stderr 并返回 1；SIGINT/SIGTERM 正常退出返回 0。HWM=64、LINGER=0、CONFLATE=0 等选项沿用 common，不增加重复配置。

当前使用命令行参数，不读取尚未冻结的 YAML 配置，不实现 `sd_notify`。使用 systemd 时应采用 `Type=exec`，不要将 stdout READY 当作 `Type=notify` 通知。节点安装到 `${CMAKE_INSTALL_BINDIR}/aviator_bus`。

## 验证

```bash
ctest --test-dir build/communication --output-on-failure
```

`bus_process` 使用真实子进程和本机 TCP 验证转发、第二个端口绑定失败时不报告 READY、失败后释放资源、单实例冲突、SIGTERM/SIGINT 退出和重新启动。`bus_help` 验证命令行入口，`communication` 保留 common 的协议与传输测试。
