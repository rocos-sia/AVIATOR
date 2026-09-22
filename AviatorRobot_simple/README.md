# AviatorRobot_simple

参考 `AviatorRobot` 的单进程双臂操纵盘控制器。仿真和真机使用同一个 `aviator`
可执行文件、`Aviator.cpp` 状态机、PIN-IK 逆解、Pinocchio 正运动学/碰撞检测、轨迹规划和命令处理，
仅 `DataLink` 后端不同。无需启动 `rocos_mujoco`、共享内存或 ROS。

运动学实现在 [src/Kinematics_pin_ik.cpp](src/Kinematics_pin_ik.cpp)：每臂一条
`aircraft → 法兰` 运动链，使用 `Distance` 模式、5 ms 求解预算、`7e-7` 精度，
J2 求解范围由姿态配置及规划余量决定。求解成功后再检查限位和 FK 残差。
5 ms 是求解预算，不是硬实时保证；IK 在规划/Servo 工作线程运行，不进入 xCore 的 1 ms 回调。
位姿统一使用 `pinocchio::SE3`，Eigen 处理旋转及插值；配置四元数保持 `[w,x,y,z]`。
项目已移除 KDL、kdl_parser 和 TRAC-IK 的源码与构建依赖；xCore 闭源 SDK 内嵌实现保留。
运动学测试使用 MuJoCo 独立核对 FK/IK。验证见 [validation/NO_KDL.md](validation/NO_KDL.md)，
最初的 PIN-IK 集成记录见 [validation/PIN_IK.md](validation/PIN_IK.md)。

## 构建与运行

已验证平台为 **Ubuntu 22.04 x86_64 / GCC 11**，需要 CMake 3.22+；控制代码使用 C++17，
MuJoCo 源码需要支持 C++20 的编译器。开源依赖的源码直接放在 `third_party/`，
构建时编译到 `build/dependencies/`，不解压依赖包、不访问网络，也不引用兄弟项目目录。
xCore SDK 没有提供实现源码，保留厂商头文件和静态库。
详细版本、来源、平台约束见 [third_party/README.md](third_party/README.md)。
源码集成与离线构建的验证记录见 [validation/DEPENDENCY_SOURCES.md](validation/DEPENDENCY_SOURCES.md)。

系统需安装编译工具及窗口开发库（一次性准备；随后项目构建可离线）：

```bash
sudo apt install build-essential cmake ninja-build python3 pkg-config \
    libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

首次构建会编译第三方库，耗时和磁盘占用明显多于之前的预编译包。
`-j4` 控制同时构建的项目数，每个依赖默认使用 2 个编译任务；内存较小时可用
`-DAVIATOR_DEPENDENCY_JOBS=1` 并将构建参数改为 `-j2`。

```bash
cd AviatorRobot_simple
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure

# 无头仿真：接近 → 锁定 → ±50° → 回中 → 推拉 170 mm → 回中 → 解锁 → 失能
./build/bin/aviator --demo --headless

# 有窗口的自动演示 / 交互
./build/bin/aviator --demo
./build/bin/aviator

# ServoWheel 周期发布示例；可加 --headless 关闭窗口，仍按真实时间更新
./build/bin/aviator --servo-demo
```

`--config` 接受 YAML 文件或配置目录。默认从可执行文件旁的 `config/aviator.yaml`
加载；模型与其他资源按 YAML 所在目录解析，从任意工作目录启动均可。
未知参数、配置错误和演示失败返回非零退出码。

```bash
# 将可执行程序、动态库、配置、模型和授权说明安装到独立目录
cmake --install build --prefix "$PWD/dist"
./dist/bin/aviator --demo --headless
```

只需要仿真时可加 `-DAVIATOR_WITH_ROKAE=OFF`；只需要真机时可加
`-DAVIATOR_WITH_SIMULATION=OFF`；不编译窗口时加 `-DAVIATOR_WITH_VIEWER=OFF`。
真机专用构建的程序不链接 MuJoCo 或 GLFW。

## 从哪里阅读、修改示例

演示流程直接写在 [src/main.cpp](src/main.cpp) 的 `main()` 中：
`Init → Enable → ApproachHandles → LockHandles → 目标循环 → UnlockHandles → Disable`。
`--demo` 分支直接调用 `MoveWheel`；目标表每行是 `{转角 rad, 推拉 m, 速度倍率 v}`，
修改该表即可改变演示目标。`--servo-demo` 分支直接展示每 20 ms 调用 `ServoWheel`，
然后 `Stop` 并等待 Servo 结束。两个分支均使用同一个 `robot` 对象，仿真与真机由配置选择。

入口不再使用 `application.cpp/.hpp`。窗口渲染留在主线程，阻塞动作在工作线程；
同一源文件末尾只有状态打印和键盘交互两个普通函数。交互仍支持运动中 `status/stop`。

测试按用途独立运行，主体均按顺序调用接口并检查结果：

| 文件 | 检查内容 |
|---|---|
| [test_pose.cpp](tests/test_pose.cpp) | SDK 行优先矩阵、工具变换、接近插值及旋转边界 |
| [test_kinematics.cpp](tests/test_kinematics.cpp) | 实际双臂的 MuJoCo FK/IK 校验、J2 限位、连续性及耗时 |
| [test_basic.cpp](tests/test_basic.cpp) | 启动姿态、初始化、使能、失能 |
| [test_acceptance.cpp](tests/test_acceptance.cpp) | 接近、锁定、完整目标表、实测误差和全程 J2 范围 |
| [test_move_wheel.cpp](tests/test_move_wheel.cpp) | 速度倍率、下发速度上限、关节限速自动延时 |
| [test_servo_wheel.cpp](tests/test_servo_wheel.cpp) | 周期目标、换目标、停止、超时、恢复和碰撞故障 |
| [test_backend_selection.cpp](tests/test_backend_selection.cpp) | 后端选择、无效配置在联网前拒绝 |

速度专项测试保留一个指令记录器，Servo 专项测试保留一个碰撞故障注入器；
使能回滚测试只保留右臂使能失败的替身。它们用于检查实际行为，不参与演示流程。
相机、实时节拍、开环状态和命令行检查仍保留在各自的短测试中，无统一测试框架。

## 相机操作

窗口内沿用 MuJoCo 的鼠标操作：

| 操作 | 功能 |
|---|---|
| 鼠标左键拖动 | 旋转视角 |
| 鼠标右键拖动 | 平移视角 |
| Shift + 左键 / 右键拖动 | 切换旋转 / 平移方向 |
| 鼠标滚轮或中键拖动 | 缩放 |
| R | 恢复初始视角，不重置机器人 |
| Esc | 停止任务并退出窗口 |

带窗口的 `--demo` 按真实时间播放；只有 `--demo --headless` 加速推进仿真。
实时节拍现在不会被控制线程的消费通知提前唤醒。修复与实测记录见
[窗口与节拍验证](validation/VIEWER_FIX.md)。

## 仿真与真机切换

默认配置为仿真：

```yaml
backend: mujoco
model: ../model/aviator.xml
```

真机使用同一个入口，改配置为：

```yaml
backend: rokae
rokae:
  left_ip: 192.168.0.100
  right_ip: 192.168.0.101
  local_ip: 192.168.0.1
  grasp_mode: open_loop
  joint_stiffness: [500, 500, 500, 500, 50, 50, 50] # Nm/rad，J1…J7
```

填写实际 IP，并确认模型中的双臂安装变换、工具变换及轴方向与实物一致。
按用户指定，真机**仅有双臂状态反馈，轮盘开环控制**：

- 双臂关节位置、速度及 TCP 位姿来自 xCore 的 1 ms 实时反馈。
- 初始轮盘参考为回中位置 `(0 rad, 0 m)`；之后随共用轨迹更新。
- `lock/unlock` 切换软件抓取阶段，不发送外部夹爪 IO，也不表示物理锁扣已反馈。
- `Status.open_loop` 为 true；轮盘角度、推拉量为指令参考，不是轮盘实测值。
- 对准判定根据实测 TCP 与参考把手位姿计算。保留关节限位、规划碰撞检查、
  跟踪误差检查、反馈超时及持续偏离检测；这些检查不能判断实际是否握住轮盘。

真机使能后，接近、操纵、Servo 和保持都运行在 `RtControllerMode::jointImpedance`；
不会为接近动作切换到位置模式。失能/退出时结束实时控制并下电。
`rokae.joint_stiffness` 配置两臂相同的七轴刚度，默认值取自本地 SDK 的
`example/rt/joint_impedance_control.cpp` 示例；此默认值尚未在当前装置上验证。
每项必须大于零，J1…J4 不超过 3000，J5…J7 不超过 300 Nm/rad，非法值在连接前拒绝。
SDK 调用参考 `xCoreSDK-CPP-main/example/rt/` 的关节阻抗控制与状态订阅示例，
配置工具坐标变换并保留控制器既有负载标定。使能前重新读取当前位置，启动失败时回滚下电，
任一臂使能失败时也清理另一臂。SDK 内嵌 KDL 被隔离在 `libaviator_rokae_sdk.so` 中。
本次没有连接真机，真机时序、轴映射与实际运动效果仍需上机验证。

## 交互命令

```text
enable
approach
lock
wheel 0.87266 0 0.5
wheel -0.87266 0 0.5
wheel 0 0 0.5
wheel 0 -0.17 0.5
wheel 0 0 0.5
unlock
disable
status
stop
reset
quit
```

运动在工作线程执行，`status/stop` 在运动期间可用；并发运动命令会被拒绝。
`stop` 取消当前轨迹并保持关节位置，保留抓取阶段；可先 `unlock` 再 `reset/disable`。
关闭窗口或输入 `quit` 会停止任务、等待线程退出，再释放模型。

## 轮盘速度与实时控制接口

```cpp
// 已 Init → Enable → ApproachHandles → LockHandles
robot.MoveWheel(0.1, -0.02, 0.5); // 阻塞：目标转角 rad、推拉 m、速度倍率 v

// 持续发布最新绝对目标；实际接入时每次从操纵输入获取 angle / displacement。
for (int i = 0; i < 100; ++i) {
    robot.ServoWheel(0.2, -0.03, 0.5); // 非阻塞，不等待到达目标
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}
robot.Stop();
while (robot.GetState() == "SERVO")
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
// 检查 GetState / GetStatus().motion_error 后再执行下一动作。
```

**第三个参数已从秒数改为速度倍率 `v`，范围 `(0, 1]`，默认 0.5。**
两个坐标均为绝对目标：转角范围 ±0.87266 rad，推拉范围 [-0.17, 0] m。
`v=1` 对应配置中的 `wheel_angular_speed`（rad/s）、`wheel_linear_speed`（m/s）
以及各关节 `min(joint_speed, URDF 速度限位)`；`v=0.5` 将这些速度上限减半。
这是上限倍率，不是恒定轮盘速度；同时转动和推拉时两个分量按同一进度协调。

`MoveWheel` 先生成平滑轮盘路径并逐点求逆解、检查碰撞，再按轮盘和关节速度限制
自动计算执行时间。降低 `v` 会延长轨迹时间，不会增加离线逆解采样数量。
结束后仍有 `settle_duration` 驻留时间，此时间不随 `v` 缩放。

`ServoWheel` 只发布最新目标，后续调用覆盖旧目标；一个常驻工作线程按 `servo_period`
（默认 20 ms）生成短段，检查逆解、关节限位、碰撞和速度，再通过底层节拍平滑插值。
每段使用五次曲线，端点速度和加速度为零；大目标不会直接跳变。
IK/碰撞计算发生在工作线程中，50 Hz 是名义更新频率，实际还受计算时间影响，
不承诺硬实时；xCore 的 1 ms 回调只负责双臂反馈和关节指令。

建议应用每 20 ms 更新一次目标。停止发布超过 `servo_timeout`（默认 250 ms，按墙钟计时），
或调用 `Stop()` 后，会保持当前位置并回到 `LOCKED`；持续发布的新指令可以重新启动 Servo，
因此主动停止时也应停止目标发布。逆解、碰撞、跟踪等错误进入 `FAULT`，原因记录在
`GetStatus().motion_error`，需处理原因并 `unlock → reset → 重新对准/lock` 后恢复。
运行 Servo 时，其余运动/阶段切换命令会立即拒绝，不会在后台排队。
命令行对应 `servo <angle_rad> <displacement_m> [v]`；单次输入只控制到下一次更新或超时，
持续控制应由调用程序按周期发布。仿真和真机共用以上接口与规划流程。

## 仿真实现与检查范围

MuJoCo 使用项目内 `model/aviator.xml` 及 `aviator_home` 关键帧。与原版一致，
无 actuator 的 14 个关节通过 `qfrc_applied = qfrc_bias + 1000*(target-qpos)` 伺服，
增加 80 的关节阻尼；weld 开关使用 `mjData::eq_active`。实时模式持续推进物理，
无头演示使用仿真时间和锁步同步。渲染只在复制物理状态时持锁。

碰撞检测保留原版的圆柱掩码：抓取圆柱只检查另一抓取圆柱，其他非圆柱几何按 SRDF 检查。
它并不检测抓取圆柱与座舱等所有物体的接触；此处与参考项目保持一致。

CTest 包含启动姿态、后端选择（无网络连接）、完整仿真验收、共用入口演示、
开环抓取阶段与故障判据、双臂使能回滚，运动中交互停止、速度倍率和速度上限、Servo 连续换目标、停止/超时及碰撞故障。验收使用 MuJoCo 被动关节的实际位置检查转角误差 < 0.005 rad、
推拉误差 < 1 mm，接近误差 < 2 mm / 0.01 rad，并逐物理步统计两臂 J2 在 [85°, 95°]。
测试结果见 [validation/REPORT.md](validation/REPORT.md)。

窗口相机回归测试（需要可用的 DISPLAY）：

```bash
./build/bin/aviator_viewer_test
```

`aviator_realtime` 已加入默认 CTest：检查活跃控制循环、模式切换和不同模型步长下，
仿真时间均不会明显快于真实时间。相机测试直接调用真实窗口注册的回调，检查旋转、平移、
缩放、失焦释放、复位和退出；无头构建不要求显示服务。

速度接口与关节阻抗修改的测试记录见 [validation/WHEEL_CONTROL.md](validation/WHEEL_CONTROL.md)。

入口与测试简化后的回归记录见 [validation/SIMPLE_TESTS.md](validation/SIMPLE_TESTS.md)。
