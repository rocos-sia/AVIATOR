# aviator_photoelectric

通过 RS485/RS232 串口按 Modbus RTU 协议读取数字量输入模块的 X1 光电开关，
持续向 AVIATOR ZMQ 总线发布 `photoelectric.state`。输入不变化时仍周期采样和发布。

## 构建与运行

整机真机启动使用 `scripts/start_aviator.sh` 时，本节点已自动纳入启动、退出监控和清理。
启动顺序为 Bus → 光电 → 设备 → Gateway → Managed Core；`--simulation` 不启动本节点。
默认加载 `config/photoelectric.yaml`，可用环境变量 `PHOTOELECTRIC_CONFIG` 覆盖。
日志写入脚本输出目录的 `photoelectric.log`，此时不会显示终端动态刷新行。
使用一键启动前先停止手动运行的采集进程，避免串口占用。以下命令用于单独调试。

在仓库根目录执行：

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --target aviator_photoelectric aviator_bus --parallel 2
./build/communication/bin/aviator_photoelectric --check-config
```

先启动总线（已有总线运行时直接使用）：

```bash
./build/communication/bin/aviator_bus
```

另一终端启动采集节点：

```bash
./build/communication/bin/aviator_photoelectric --config config/photoelectric.yaml
```

直接在终端运行时，会按发布频率在同一行实时刷新并立即 flush 输出：

```text
X1=1  valid=true  status=ok
```

状态不变时也刷新；通信异常时显示 `valid=false` 和原因。退出时恢复换行。
标准输出重定向到文件或管道时关闭这一行动态显示，保留普通日志。

Ctrl+C 或 SIGTERM 退出并释放串口。`--check-config` 只检查配置，不打开串口或连接总线。
省略 `--config` 时优先使用可执行文件相对路径下的安装配置
`../share/aviator/config/photoelectric.yaml`，其次使用编译时的源码配置路径。
节点支持 Linux，使用现有的 yaml-cpp、ZMQ 和公共消息库，无需 Python 串口依赖。

## 配置

配置文件：[config/photoelectric.yaml](../../config/photoelectric.yaml)。修改后重启节点。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `serial.device` | `/dev/ttyUSB0` | 串口绝对路径；也可使用稳定的 `/dev/serial/by-id/...` 路径 |
| `serial.baud_rate` | `38400` | 支持 4800、9600、19200、38400、57600、115200 |
| `serial.data_bits` / `parity` / `stop_bits` | `8` / `none` / `1` | 8 位数据；校验可选 none/even/odd；停止位可选 1/2 |
| `modbus.slave_address` | `1` | 模块站号，按手册支持 1～255 |
| `modbus.input_address` | `0` | 零起始输入地址；0 对应 X1，允许 0～63（须有对应硬件通道） |
| `modbus.response_timeout_ms` | `100` | 一次请求的发送和响应总超时 |
| `modbus.reconnect_interval_ms` | `1000` | 打开失败、通信失败后重新打开串口的间隔；不得小于响应超时 |
| `poll_hz` / `publish_hz` | `20` / `20` | 采集和发布频率，独立计时，范围 1～100 Hz；采集频率还受串口带宽限制 |
| `stale_timeout_ms` | `150` | 成功采样的有效期；不得小于采集周期加响应超时 |
| `invert` | `false` | 是否对模块输入状态取反 |
| `publisher_id` | `aviator_photoelectric` | 消息发布者标识 |
| `publish_endpoint` | `tcp://127.0.0.1:5555` | PUB 连接的总线入口；订阅者连接总线出口 5556 |

仅使用功能码 `0x02` 读取一路离散输入，不修改模块参数或控制输出。
按照目录内手册第 2.3 节，X1 的请求为 `01 02 00 00 00 01 B9 CA`。
响应包含站号、功能码、字节数、输入状态以及低字节在前的 CRC16。
未触发响应为 `01 02 01 00 A1 88`；已触发响应为 `01 02 01 01 60 48`。
节点校验完整 Modbus RTU 帧后才更新信号值。

默认 `value=1` 表示模块检测到输入触发，`value=0` 表示未触发。
常开/常闭光电开关对“检测到物体”的电气输出可能相反，可用 `invert` 调整。
RS485 转换器需支持自动收发方向切换；串口不应同时由调试软件占用。
节点使用文件锁避免遵守该锁约定的进程同时访问同一串口。
模块应保持默认的“不主动上传”模式，以免主动上传数据干扰请求响应。

## 发布消息

使用项目标准两帧消息：第一帧为 `photoelectric.state`，第二帧为 JSON。
主题已注册到 `common/protocol`，消息类型为 `PhotoelectricState`。

```json
{
  "msg_type": "PhotoelectricState",
  "version": "1.0",
  "sequence": 25,
  "timestamp": 1792368000000000,
  "sample_mono_us": 123456789,
  "clock_id": "host-boot-id",
  "publisher_id": "aviator_photoelectric",
  "session_id": "run-instance",
  "valid": true,
  "channel": "X1",
  "value": 1,
  "status": "ok"
}
```

`value` 始终为整数 0 或 1，`valid=true` 时才代表有效信号。
启动尚未读到数据时为 `value=0, valid=false, sample_mono_us=0`。
CRC、站号、功能码、长度异常、设备异常响应、超时或断开后，节点继续发布，
保留最后的数值和采样时间，设置 `valid=false`，并在 `status` 中说明原因。
恢复成功采样后自动恢复 `valid=true`。串口无响应期间发布频率不受响应等待影响。

`timestamp` 是发布时 UTC 微秒；`sample_mono_us` 是成功读取响应时的单调时钟微秒，
仅成功采样时更新。`sequence` 每次发布递增。订阅者需同时检查 `valid` 和消息新鲜度；
ZMQ PUB/SUB 不保证每条消息到达，进程异常退出也可能无法发出无效通知。
普通轮询会漏掉短于采样间隔的脉冲，本节点用于持续状态采集。

在已安装 `pyzmq` 的环境观察信号：

```bash
python3 - <<'PY'
import json
import zmq
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect('tcp://127.0.0.1:5556')
sub.setsockopt_string(zmq.SUBSCRIBE, 'photoelectric.state')
while True:
    topic, payload = sub.recv_multipart()
    if topic != b'photoelectric.state':
        continue
    msg = json.loads(payload)
    print(msg['value'], 'valid=', msg['valid'], 'status=', msg['status'], flush=True)
PY
```

## 验证

```bash
ctest --test-dir build/communication -R '^photoelectric_' --output-on-failure
```

进程测试需要 Python 的 `yaml`、`zmq`，通过伪串口和独立 ZMQ 端口验证真实节点，
覆盖不变信号持续发布、0/1 切换、分段帧、CRC/地址/功能码/长度异常、Modbus 异常、
超时、设备消失与重连、反相、配置校验及退出；不访问实际硬件。
