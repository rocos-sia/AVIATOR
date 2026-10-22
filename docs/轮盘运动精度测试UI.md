# 轮盘运动精度测试 UI

在仓库根目录执行：

```bash
cmake --build build --target flight_gateway -j4
./scripts/start_wheel_accuracy_ui.sh
```

模拟双手、机械臂仍连接真机：

```bash
./scripts/start_wheel_accuracy_ui.sh --fake-hand
```

完整 MuJoCo 仿真（保留测试窗口，隐藏仿真窗口）：

```bash
./scripts/start_wheel_accuracy_ui.sh --simulation --headless
```

`--logger`、`--tts` 与原启动脚本相同。`--dry-run` 仅打印启动命令，不连接设备。
UI 默认用 `/usr/bin/python3`，需要 `tkinter` 和可用的桌面显示；可用 `UI_PYTHON` 指定解释器。
启动前会检查解释器、显示环境以及网关是否支持测试模式。

## 操作

1. 等待 Core 初始化完成；真机保留原 X1 光电初始化条件。
2. 点击 **Home**，等待 `STANDBY`。
3. 点击 **抓握**，等待完整接近、闭手和刚度切换完成，进入 `FOLLOWING`。
4. 点击一个点位按钮。首次运动自动请求 `start_control`，确认 `CONTROL` 后下发目标。
5. 等待“指令参考已到点，保持中”，使用外部测量记录精度；继续点击其它点位或修改坐标后执行。
6. 点击 **松手** 执行完整松盘、撤离、回 Home 流程。也可点击 **停止 / 退出控制**，减速并回到 `FOLLOWING`。

Home、抓握、松手分别使用原 `enter_standby`、`grasp_wheel`、`leave_wheel` 状态机操作。
按钮是否可用由反馈状态决定；操作拒绝、超时和故障显示在窗口底部，不自动重试。
修改坐标只修改待执行值，必须点击对应按钮才生效。

## 坐标与默认点位

UI 使用度和毫米，后台转换成轮盘绝对转角（rad）和绝对推拉位移（m）。
转角限制严格采用现有 `±0.87266 rad`，约 `±49.999734°`；位移为 `[-170, 0] mm`。
`(0°, 0 mm)` 是推拉行程端点，`(0°, -85 mm)` 是当前配置的行程中位。

| 点位 | 转角（°） | 绝对位移（mm） |
| --- | ---: | ---: |
| P1 | 0 | 0 |
| P2 | -25 | -42.5 |
| P3 | 25 | -42.5 |
| P4 | -25 | -127.5 |
| P5 | 25 | -127.5 |

点位运动采用现有 Managed Servo 链路，速度比例沿用网关的 `v=1`，实际速度、加速度、jerk
遵循 `config/robot.yaml`。输入检查范围后，运动仍须通过底层逆解、关节限制及已配置的检查。
首次进入控制前保持当前抓握参考位形；后续点位从当前运动状态平滑转换，两点之间不回 Home、不松手。

## 反馈含义与退出

“指令参考”来自 `wheel_reference`。参考误差小于等于 `0.05° / 0.1 mm`，且新鲜反馈连续保持
0.5 秒后，显示参考到点；这只是 UI 就绪提示，不是精度验收阈值。
真机没有轮盘直接位置反馈时，“轮盘实测”显示不可用；只有设备明确提供有效
`wheel_measurement` 时才显示测量值。机械臂跟踪偏差、手部滑移等应使用外部测量评价。

测试模式通过独占的本机 Unix socket 接入 `flight_gateway`，不启用 SDL 摇杆和键盘输入。
UI 每 20 ms 发送有时间戳的心跳，网关不自行刷新它。UI 卡住、连接中断或输入过期后，
原 Core 输入超时机制减速并保持，进入错误状态；重连不重发历史操作。

关闭 UI 或在启动终端按 Ctrl+C，复用原脚本的 Core → 手节点 → 其它节点有序退出流程。
监视器仍在 `http://127.0.0.1:8081` 提供状态；测试模式不会再自动打开全屏 Chrome。
一次只能启动一套控制系统；启动锁与原 `start_aviator.sh` 共用。

## 离线验证

```bash
ctest --test-dir build -R '^(wheel_accuracy_process|gateway|gateway_config|gateway_keyboard|gateway_sdl|scripts)$' --output-on-failure
```

`wheel_accuracy_process` 使用独立端口运行真实 UI 控制器、网关、Managed Core 和 MuJoCo，
检查五点运动、修改目标、完整 Home/抓握/松手、输入越界和过期拒绝、UI 心跳丢失；不连接真机。
