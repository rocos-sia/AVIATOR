# inspire_hand_can — 因时手 SocketCAN ZMQ 节点

把 Inspire_Robot（因时手）驱动从 ECanVci USB-CAN 移植到 Linux SocketCAN，并做成
AVIATOR 总线上的 `hand.command` 订阅节点：收到命令 → 校验 → 通过 `can0` 驱动双手，
同时回发 `hand.state`。

## 文件

| 文件 | 作用 |
| --- | --- |
| `can_writer.hpp` | SocketCAN 裸写器（RAII，扩展帧、2 字节小端） |
| `inspire_hand.hpp` | 完整移植 `Inspire` + `InspireAction`：位置/速度/力寄存器、五指动作、动作序列/手势 |
| `hand_node.cpp` | ZMQ SUB 节点：订阅 `hand.command` → 校验/守卫 → 驱动双手 → 回发 `hand.state` |
| `config/inspire_hand.yaml` | 配置（can 接口、双手 id、速度/力、安全姿态、端点、支持模式、超时） |
| `CMakeLists.txt` | 独立构建（libzmq + cppzmq + nlohmann/json + yaml-cpp + pthread） |

## 前置：把 CAN 口拉起来

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

> 因时手波特率 500k。若你的接口名不是 `can0`，改 `config/inspire_hand.yaml` 的
> `can.interface`。需要 `can-utils` 可用 `sudo apt install can-utils`（`candump can0`
> 可用来观察总线帧）。

## 构建

```bash
cd examples/inspire_hand_can
cmake -B build
cmake --build build
```

## 运行

```bash
./build/inspire_hand_node --config config/inspire_hand.yaml
# 或省略 --config，默认找可执行文件旁的 config/inspire_hand.yaml
```

## 消息格式（遵循 AVIATOR `hand.command` 草案协议）

订阅 `hand.command`，Frame0 = `hand.command`，Frame1 = JSON。两条消息只差 `mode` 与
`hands` 字段：

```json
{
  "msg_type": "HandCommand", "version": "1.0", "sequence": 1,
  "timestamp": 1790121600001000, "sample_mono_us": 12345679000,
  "clock_id": "hostA-boot1", "publisher_id": "aviator_core",
  "session_id": "22222222-2222-4222-8222-222222222222", "valid": true,
  "control_epoch": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
  "mode": "NORMALIZED_POSITION",
  "origin": {
    "publisher_id": "flight_gateway",
    "session_id": "11111111-1111-4111-8111-111111111111",
    "sequence": 182736, "sample_mono_us": 12345678000, "clock_id": "hostA-boot1"
  },
  "hands": {
    "left":  {"drive_position_normalized": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
    "right": {"drive_position_normalized": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}
  }
}
```

### 模式

| mode | 输入 | 直通映射 |
| --- | --- | --- |
| `NORMALIZED_POSITION` | `hands.{side}.drive_position_normalized[6]` ∈[0,1] | `raw[i] = round(norm[i] * 1000)` 逐通道直发 |
| `GRASP_SETPOINT` | `hands.{side}.grasp.closure` ∈[0,1] | `raw[i] = round(closure * 1000)` 直发到 6 路 |

> 直通转发，无插值：发什么值就按比例到对应驱动位置。六指顺序恒为
> `[拇指旋转, 拇指, 食指, 中指, 无名指, 小指]`；1000 = 张开，0 = 闭合。

## 安全与授权

- 首个合法命令会**锁定** `session_id` + `control_epoch`，之后不同会话/epoch 的命令被拒绝；
  序号必须严格递增（去重/乱序丢弃）。要换会话/epoch 需重启节点。
- `valid=false` 或超过 `node.timeout_ms` 未收到命令 → 双手回落 `hand.safe_pose`，
  `hand.state` 报 `valid=false`。
- Ctrl-C / SIGTERM 退出前也会先回落到安全姿态再关闭 CAN。

## 测试

用一个最小发布脚本发一条 `NORMALIZED_POSITION` 命令即可观察手动作（需先起 aviator_bus，
或把 `subscribe_endpoint` 临时改成直连的 PUB 地址）。参考 `examples/aruco_camera` 的
`pub.connect/publish` 写法。

```bash
ctest --test-dir build          # 只跑 --help 冒烟测试
```
