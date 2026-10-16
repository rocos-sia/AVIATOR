# Flight Simulator — 双路 RS422 飞控模拟器

这是部署在第二台工控机（B）的独立 Linux/C++17 项目。将本目录完整复制出去即可构建，不需要 AVIATOR 仓库、ZMQ、Core、机器人 SDK 或 Python 运行环境。它读取 USB 手柄，通过两路全双工 RS422 向机器人工控机（A）发送控制帧，同时显示 A 返回的状态。

当前实现采用 `aviator-bench-v1` 台架协议，线格式见 [PROTOCOL.md](PROTOCOL.md)。它模拟飞控通信与操作输入；尚未接入的机器人传感器以未知值回传。通信互通不等同于实际飞控的认证或硬件时序验证。

## 构建与启动

Ubuntu/Debian 上安装编译依赖和中文字体：

```bash
sudo apt-get install build-essential cmake libsdl2-dev libfreetype6-dev libyaml-cpp-dev fonts-noto-cjk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build --parallel 2
./build/flight_simulator --list-joysticks
# 修改 config/config.yaml 中的两个串口路径、手柄索引与轴映射
./build/flight_simulator --config config/config.yaml
```

SDL2 需至少 2.0.14。`device: auto` 自动选择可用手柄，也可填设备索引。两路路径必须不同，建议使用 `/dev/serial/by-id/...` 或稳定 udev 别名；板载串口可填实际 `/dev/tty...`。运行用户需要串口读写权限。设备文件使用独占锁，避免两个模拟器争用同一串口。

有桌面时显示中文操作窗口。仅用手柄且无桌面时：

```bash
SDL_VIDEODRIVER=dummy ./build/flight_simulator --config config/config.yaml
```

默认 `require_joystick: true`，未接手柄、拔出手柄或输入处理超时均停止两路下行。需要纯键盘测试时设为 `false`；没有手柄且窗口失焦会立即停发。退出可关闭窗口或按 Ctrl+C。串口断开后每秒尝试重连，启动/重连先发送空闲指令，不重放先前按钮操作。

## 两台机器如何连接

```text
USB 手柄 → 工控机 B / flight_simulator
                     ├─ RS422 A ⇄ RS422 A ─┐
                     └─ RS422 B ⇄ RS422 B ─┤ 工控机 A / flight_gateway → Core → 机器人
```

每台机器需两个独立的全双工 RS422 通道，按硬件文档交叉连接发送/接收差分对，并配置参考地和终端。软件使用 115200、8N1，不配置串口卡的电气工作模式。

A 端继续使用 AVIATOR，选择 `config/flight-rs422.yaml`、`config/system-rs422.yaml`，设置 Core 的 `flight_source: FLIGHT`，通过 `./scripts/start_aviator.sh --rs422` 启动；原有本机手柄方式仍通过默认配置启动。B 端只部署本项目。

## 输入和反馈

| 默认按钮 / 数字键 | 指令 |
| --- | --- |
| 1 | 待命 enter_standby |
| 2 | 握盘 grasp_wheel |
| 3 | 开始控制 start_control |
| 4 | 退出控制 exit_control |
| 5 | 松盘 leave_wheel |
| 6 | 复位错误 reset_error |

FOLLOWING 和 CONTROL 中可直接用按钮 1 或 5 松盘、撤离并回 home。故障清除后按钮 6 复位到 READY；仍握盘时按钮 5 撤离，已脱离时按钮 1 回 home。

`buttons` 恰好 11 项，编号对应 SDL 按钮索引 0～10，未分配填 `none`。键盘 `1`～`9`、`-` 对应第 1～9、11 项，`0` 只用于轴目标归零。方向键按住累加、松开保持；移动对应摇杆轴可重新接管，反向与限位在配置中设置。需实际核对手柄编号和坐标方向。

发送周期 12.5 ms（80 Hz），两路发送相同逻辑帧。每次有效按钮点击连续发送 10 拍操作，再发送至少 10 拍空闲；忙碌期间的新点击不排队，终端提示 `BUSY_OR_NEUTRAL` 后需重新点击。调度延迟时跳过错过的周期，不突发补帧。

窗口标题显示两路机器人状态；终端每秒显示两路 LIVE/STALE、计数器、选中通道、通信状态、指令/位置回绕、实测位置、机构状态、故障、版本及软件短码。STALE 行保留最后数据，仅供诊断。操作回绕不代表动作完成，应等待机器人状态变化。

`simulator.send_a/send_b: false` 可模拟一路停发；`corrupt_every: N` 每 N 个周期破坏 A 路帧校验，0 为关闭。配置修改后重启生效。

## 独立部署和打包

安装到自定义目录：

```bash
cmake --install build --prefix "$PWD/dist"
./dist/bin/flight_simulator
```

默认配置按可执行文件位置寻找 `../share/flight_simulator/config.yaml`，随后查找同目录的 `config.yaml`；也可始终通过 `--config` 指定。构建目录的默认配置是 CMake 配置阶段的副本，修改源配置后应显式指定源配置或重新运行 CMake。

生成源码包和运行包：

```bash
(cd build && cpack --config CPackSourceConfig.cmake)
(cd build && cpack)
```

源码包可在另一台 Linux 工控机按架构重新编译。运行包仅包含程序、配置、文档和许可证，动态依赖目标机器的 SDL2、FreeType、yaml-cpp、C++ 运行库及中文字体；需匹配 CPU 架构和系统库版本。实际安装的 `.so` 依赖可用 `ldd build/flight_simulator` 查看。Python 仅用于可选测试，不进入运行包。

## 验证

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

安装 Python 3 后额外运行两个标准库测试，无需 pip：配置拒绝测试，以及使用两组 PTY 和 SDL 虚拟手柄的生产程序集成测试。覆盖独立协议字节向量、校验/重同步、10 拍操作、轴量化、双路一致、反馈解析及手柄拔出停发。PTY 测试不能验证 RS422 电气接线、真实 USB 掉线检测时延或机器负载下的发送抖动，这些需现场联调。

本项目的编解码与输入代码独立维护，不链接 AVIATOR 内部库。修改双方线协议时应同时更新 `PROTOCOL.md`、编解码和固定字节测试；不通过增加串口字段偷偷改变 `aviator-bench-v1`。
