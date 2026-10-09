# change_stiffness

基于 xCoreSDK 0.7.1 的 `joint_impedance_control.cpp`，用于单台 xMateER7 Pro 的关节刚度交互测试。启动时读取当前 7 个关节角作为固定目标，持续运行关节阻抗控制；不执行 MoveAbsJ 或正弦轨迹。外力使机械臂偏离后，目标仍是启动位置。

## 独立编译

从 AVIATOR 仓库根目录执行，无需编译 AVIATOR 主项目：

```bash
cmake -S examples/change_stiffness -B examples/change_stiffness/build -DCMAKE_BUILD_TYPE=Release
cmake --build examples/change_stiffness/build -j2
```

依赖 Linux x86_64、CMake ≥ 3.16、C++17 编译器、pthread，以及仓库内的 xCoreSDK 头文件、Eigen 头文件、`libxCoreSDK.a` 和 `libxMateModel.a`。不需要 MuJoCo、Pinocchio 或 ROS。SDK 路径可用 `-DXCORE_SDK_ROOT=/path/to/xCoreSDK-0.7.1` 指定；缺失预编译库时参考 SDK 的 `lib/README.md`。

## 桌面 UI

`change_stiffness_ui` 使用 Qt5，基于简化 demo 的暂停运动、设置刚度、恢复保持流程，调参期间保持 RT 模式。Ubuntu 上额外安装 `qtbase5-dev` 后可单独编译窗口：

```bash
cmake -S examples/change_stiffness -B examples/change_stiffness/build -DCMAKE_BUILD_TYPE=Release
cmake --build examples/change_stiffness/build --target change_stiffness_ui -j2

# 离线演示，不连接机械臂；打开后点击“开始离线演示”
./examples/change_stiffness/build/change_stiffness_ui --dry-run

# 真机界面：打开窗口不会自动连接或上电
./examples/change_stiffness/build/change_stiffness_ui
```

窗口可选择左臂/右臂 IP 预设或自定义 IP，一次控制一台机械臂。点击“连接并原地保持（上电）”后，读取当前关节角并保持该目标，直到停止。支持以下操作：

- J1～J7 滑块调节（分辨率 0.1）或精确输入（3 位小数，回车或移开焦点提交输入）。
- 默认自动应用：滑块松开且停止调节 500 ms 后提交最新整组刚度；也可关闭自动应用，点击“应用刚度”。
- 每次应用依次执行 `stopLoop()`、`stopMove()`、`stopReceiveRobotState()`，通过现有控制器设置刚度，再恢复状态接收、回调及运动。全程保持 RT 模式，复用控制器，不重新上电。更新期间会暂停运动，完成后继续保持**本次连接时的原目标位置**。
- “已应用”显示最近一次成功设置并启动保持的参数，是客户端的成功记录；不表示从机器人回读的刚度。离线模式的该列仅表示模拟结果。
- 参数应用期间禁止继续编辑，停止按钮始终可用。关闭窗口或 `Ctrl+C` 会请求停止，等待工作线程完成停止运动、下电和断开连接后退出。
- 日志显示 SDK 操作名称和错误；参数设置或恢复运动失败时结束控制并执行清理。若清理报错，界面保留错误状态，需确认设备实际状态。

SDK 操作在独立工作线程串行执行，界面不会被网络请求阻塞。停止请求在 SDK 调用返回后处理，不能打断正在执行的 SDK 网络调用。实时回调只发送固定位置并更新心跳；回调超过 1 秒不再更新时，工作线程停止控制并检查 SDK 错误。UI 不修改 `robot.yaml`，不提供阻尼调节。

`--dry-run` 启动时勾选离线演示；停止后可在窗口切换真机/离线模式。没有 Qt5 时 CMake 跳过 UI，仍可编译两个命令行 demo；也可显式传入 `-DCHANGE_STIFFNESS_BUILD_UI=OFF`。

离线界面测试（Qt5 Test，无真机连接）：

```bash
cmake --build examples/change_stiffness/build --target change_stiffness_ui_test -j2
ctest --test-dir examples/change_stiffness/build --output-on-failure
```

测试覆盖自动/手动应用、拖动合并、精确值与范围校验、应用中停止、重新启动、连接中关闭窗口及退出清理交互。真机的暂停/恢复运动、参数生效和下电仍需现场验证。

## 运行

```bash
# 离线检查键盘操作，不连接机器人
./examples/change_stiffness/build/change_stiffness --dry-run

# 左臂：机器人 IP、本机网卡 IP
./examples/change_stiffness/build/change_stiffness 192.168.1.160 192.168.1.100

# 右臂：单独启动一次，仅控制指定机械臂
./examples/change_stiffness/build/change_stiffness 192.168.2.160 192.168.2.100
```

真机运行会切换自动/实时模式并上电。请在交互终端运行，并确保指定机械臂没有其他控制程序占用。本机 IP 需已配置到网卡。程序不读取或修改 `config/robot.yaml`。

| 按键 | 功能 |
| --- | --- |
| `1`～`7` | 选择 J1～J7，默认 J1 |
| `+` / `-` | 按步长增减所选关节刚度，默认步长 10；`=` 也可增加 |
| `v` | 输入所选关节的精确刚度，回车提交 |
| `s` | 输入调整步长，回车提交，范围 `(0, 3000]` |
| `Esc` | 取消当前数值输入 |
| `p` / `h` | 显示全部刚度 / 按键帮助 |
| `q` / `Ctrl+C` / `Ctrl+D` | 退出；真机模式停止控制并下电 |

例如按 `7`、`v`、输入 `0.1` 后回车，即将 J7 刚度设置为 0.1 Nm/rad。除数值输入外，按键不需要回车。支持小数、科学计数法和退格。

初始刚度为 `[500, 500, 500, 500, 50, 50, 50]` Nm/rad。J1～J4 输入范围 `[0, 3000]`，J5～J7 为 `[0, 300]`；SDK/控制器可能根据硬件状态进一步限制。程序修改的是 `setJointImpedance()` 的刚度系数，不提供独立阻尼设置。

键盘线程发送参数设置请求，实时回调持续发送固定目标位置。仅在 SDK 设置成功后更新显示值；设置失败会打印错误并停止控制。SDK 实时异常也会触发退出清理，最后恢复终端。在线参数生效情况需在对应控制器上实测；`--dry-run` 只验证交互逻辑。

## 最简定时修改 demo

`change_stiffness_simple.cpp` 无键盘调参逻辑：读取启动位置，设置初始刚度 `[500, 500, 500, 500, 50, 50, 50]`，调用 `startMove()` 和非阻塞的 `startLoop(false)`。从 `startMove()` 返回时开始计时，两秒后在主线程依次执行：

```cpp
rtCon->stopLoop();
rtCon->stopMove();
robot.stopReceiveRobotState();
rtCon->setJointImpedance({500, 500, 500, 500, 50, 50, 0.1}, ec);
```

上述更新期间保留 RT 模式、上电状态和原控制器，在下一次 `startMove()` 前设置刚度。随后重新开启状态接收并设置回调，调用 `startMove()` / `startLoop(false)` 恢复原目标位置保持。再保持约 8 秒后停止并下电，支持 `Ctrl+C` 提前退出。总时长约为 10 秒加参数更新耗时。源码检查各步骤的错误，修改失败时退出清理，不重新启动运动。计时使用系统单调时钟，实际调用可能有线程调度延迟。异步实时控制错误由 `stopLoop()` 报告。

```bash
cmake -S examples/change_stiffness -B examples/change_stiffness/build
cmake --build examples/change_stiffness/build --target change_stiffness_simple -j2
./examples/change_stiffness/build/change_stiffness_simple 192.168.1.160 192.168.1.100
```
