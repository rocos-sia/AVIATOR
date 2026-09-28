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
MuJoCo 源码需要支持 C++20 的编译器。Ubuntu 22.04 提供的通用库由 apt 安装；
Pinocchio、Coal、PIN-IK、MuJoCo 及少量没有系统包的依赖保留源码，编译到
`build/dependencies/`。xCore SDK 保留厂商头文件和静态库。
详细包名、来源和限制见 [根目录 third_party/README.md](../../third_party/README.md)。
本次构建与 14 项回归结果见 [系统依赖验证记录](validation/APT_DEPENDENCIES.md)。
`validation/` 内既有全源码集成记录作为历史记录保留。

从仓库根目录执行一次依赖安装（需要已启用 Ubuntu jammy 的 universe 软件源）：

```bash
./examples/AviatorRobot_simple/scripts/install_dependencies.sh --dry-run
./examples/AviatorRobot_simple/scripts/install_dependencies.sh
```

安装脚本自动使用 sudo；只适用于 Ubuntu 22.04，不添加 PPA。软件包准备好后构建无需下载源码。
由旧版全源码方案切换时，请使用新的构建目录，避免 `build/dependencies/` 的旧库覆盖系统版本。
首次构建仍需编译保留的机器人库。
`-j4` 控制同时构建的项目数，每个依赖默认使用 2 个编译任务；内存较小时可用
`-DAVIATOR_DEPENDENCY_JOBS=1` 并将构建参数改为 `-j2`。

```bash
cd examples/AviatorRobot_simple
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

当前姿态配置对应 `aviator_control.urdf` 中基座位置 `(-0.91333, ±0.010, 0.12156)` m
和 RPY `(∓1.5707, 1.3962671, 0)` rad。碰撞 URDF 和 MuJoCo 使用同一基座变换。
抓取圆柱轴线与把手轴线重合，抓取中心位于把手中心沿轴线正向 10 mm 处；
相对于旧配置，两侧抓取点均沿把手轴线移动 30 mm，右侧抓取朝向绕该轴旋转 15°。
home 选在零位抓取目标沿法兰负 Z 方向后退 120 mm 的位置，预接近种子对应后退 60 mm。
`posture.json`、`grasp.json` 与 MuJoCo 的 `aviator_home` / handle site 已配套更新；
修改基座后，应重新验证这些姿态和完整运动路径。

真机故障会输出阶段（`home trajectory`、`home settling`、`pre-approach`、
`final approach`、`TCP settling`）以及首次故障快照：左右臂 IP、反馈年龄与最大回调间隔
（ms）、回调次数、关节实际角/目标角（rad）、关节速度（rad/s）、TCP 位置误差（mm）
和姿态误差（rad）。故障原因会区分反馈超时与锁定后对齐丢失，反馈恢复也会保留首次快照。
SDK 状态单独标注为当前值；异步 SDK 原始异常若在 `stopLoop()` 返回，会在退出/停止时继续打印，
因此排查时需保留 `Execution failed` 之后的输出。普通 `status` 也会显示两侧 TCP 误差。
若 `startMove(jointImpedance)` 被拒绝，还会在清理前读取并打印控制器的实际上电状态、
手自动模式、运行状态，以及最近 10 条警告/错误的时间、ID、内容和修复说明。
控制器日志可能包含历史事件，应按时间对应本次启动；诊断查询不会自动清除报警或恢复运动。

真机在 `startMove(jointImpedance)` 前按官方 `getCurrentJointPos` 的逻辑更新实时状态队列，
读取 `jointPos_m` 并初始化保持目标；尚无实时位置时回退到 `jointPos()`，查询失败则终止启动。
启动日志中的 `initial_position_rad` 会打印数据来源和七个关节角（弧度）。
首次实时回调仍以该帧实测位置对齐目标，使能就绪判断不会被启动前的查询提前满足。

若仍在 `startMove(jointImpedance)` 被拒绝，可手动使用单臂诊断程序比较同一姿态下的启动结果：

```bash
./build/bin/aviator_rokae_hold_test 192.168.1.160 192.168.1.100
```

它直接使用同一份 xCore SDK，仅连接指定机械臂，上电后以关节阻抗保持当前关节角 5 秒，
结束后下电。没有 `MoveAbsJ`、home、轮盘运动或第二台机器人连接，不读取 `aviator.yaml`。
Ctrl+C 请求结束保持并清理连接。此程序会操作真机，不加入自动测试。
若同一姿态下也在 `FcStart` 失败，应进一步比较控制器状态与官方示例预运动后的姿态条件；
若成功，再排查 Aviator 的双臂初始化和运行环境差异。

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
| [test_continuity.cpp](tests/test_continuity.cpp) | 调度延迟、阶段衔接、停止加速度、反馈故障和双臂停止失败处理 |
| [test_home.cpp](tests/test_home.cpp) | home 指令顺序、允许位置偏差及未停稳超时 |
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
  left_ip: 192.168.1.160
  right_ip: 192.168.2.160
  left_local_ip: 192.168.1.100
  right_local_ip: 192.168.2.100
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
  反馈有效性、反馈超时及锁定后持续偏离检测；这些检查不能判断实际是否握住轮盘。

真机使能后，接近、操纵、Servo 和保持都运行在 `RtControllerMode::jointImpedance`；
不会为接近动作切换到位置模式。失能/退出时结束实时控制并下电。
`rokae.joint_stiffness` 配置两臂相同的七轴刚度，默认值取自本地 SDK 的
`example/rt/joint_impedance_control.cpp` 示例；此默认值尚未在当前装置上验证。
每项必须大于零，J1…J4 不超过 3000，J5…J7 不超过 300 Nm/rad，非法值在连接前拒绝。
SDK 调用参考 `xCoreSDK-CPP-main/example/rt/` 的关节阻抗控制与状态订阅示例，
当前工具设置代码保持注释状态，使用控制器既有工具/负载配置；TCP 对准前需确认它与 grasp.json 一致。使能前重新读取当前位置，启动失败时回滚下电，
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

## Home 与预接近的 Ruckig 规划

源码集成在 `third_party/ruckig-0.15.3`，版本与提交见其中 `README.vendor.md`，
保留 MIT 许可证。C++ 库静态编入控制库，关闭云端、Python 和示例构建，不在运行时联网。

```yaml
home_speed: 0.1          # 当前位置 → home，rad/s
approach_speed: 0.1      # home → 预接近位置，rad/s
joint_acceleration: 0.2  # 上述两段，rad/s²
joint_jerk: 1.0          # 上述两段，rad/s³
```

一个 14 自由度 Ruckig 规划器自动求时长并进行时间同步。速度取阶段配置、
`joint_speed` 和 URDF 上限的最小值。轨迹先计算并检查，再按 1 ms 取样，
起止速度/加速度为零；零位移保持原指令。`approach_duration` 已删除。
最后沿法兰方向对准把手仍使用 `final_approach_duration`，轮盘几何路径规划保持原方式。
Ruckig 同步的是指令轨迹时间，不提供两台控制器的硬件时钟同步。

`tracking_tolerance` 已删除，有限的实际关节角与指令角偏差不再触发停止。
反馈异常、SDK 错误、关节限位及指令跳变仍会拒绝运动。
Home 指令轨迹结束后不检查实际关节位置与 home 的误差，仅确认所有关节速度
<0.02 rad/s 并持续 0.15 s。最多等 5 s，仍未停稳则报 `Home settling timeout`。
有位置偏差但已停稳时继续预接近，下一段从上一段指令末点衔接，不跳回实测位置。

## 真机周期线程

双臂先完成上电/订阅/阻抗参数准备，然后分别紧接执行 `startMove`/`startLoop`。
两臂均返回首帧后检查双方 SDK 状态与反馈时效。所有实际运动均为关节阻抗模式。

SDK 回调只读反馈、接收目标并返回关节指令。反馈锁内只复制/检查数值；
TCP 计算与日志格式化使用锁外快照。回调尝试获取共享锁，竞争时重发上一条已发送目标，
不确认新序号；业务线程等待双臂取走目标后再推进，不跨点追赶。
SDK 的 `JointPosition` 按值接口仍涉及内部存储管理，并非整个 SDK 都保证无分配。

上电前通过短命线程探测实时调度权限；通过 SDK 请求 `SCHED_FIFO` 优先级 80，
首帧再次检查实际周期线程策略/优先级。
未生效会明确报 `RT scheduling unavailable` 并终止使能，不静默以普通优先级运行。
运行账户需要适当的 `rtprio` 限额（至少 80）或进程具备 `CAP_SYS_NICE`；
仅安装实时内核并不足够。程序不会修改系统权限配置。
停止时输出 SDK 包装回调计数、最大回调间隔（ms）、包装回调耗时（µs）、
实际调度策略/优先级；耗时不包含 SDK 在回调外的收发、滤波及按值返回复制。
反馈年龄同时出现在故障快照中。仍需通过真机双臂保持/运动验证收发稳定性，
离线仿真不能证明真实网卡、控制器和线程每周期均满足 1 ms。

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

首次使能以后端的实测角度初始化保持目标；后续阶段始终从上一条指令衔接，
实测角度用于反馈有效性和关节限位检查，不按指令跟踪偏差中止，也不覆盖指令起点。执行线程每拍最多推进 1 ms 轨迹，
调度延迟会延长运动时间。真机还等待两臂 SDK 回调都取走上一条目标，防止覆盖未消费的点；
这是软件指令顺序同步，两台控制器仍没有硬件时钟同步。
SDK 回调按 URDF 关节速度上限检查每拍增量，异常时拒绝新指令并报错，不对两臂分别限幅后继续运动。

`ServoWheel` 只发布最新目标，后续调用覆盖旧目标；一个常驻工作线程按 `servo_period`
（默认 20 ms）生成短段，检查逆解、关节限位、碰撞和速度，再通过底层节拍平滑插值。
每段使用五次曲线，端点速度和加速度为零；大目标不会直接跳变。
IK/碰撞计算发生在工作线程中，50 Hz 是名义更新频率，实际还受计算时间影响，
不承诺硬实时；xCore 的 1 ms 回调只负责双臂反馈和关节指令。

建议应用每 20 ms 更新一次目标。停止发布超过 `servo_timeout`（默认 250 ms，按墙钟计时），
或调用 `Stop()` 后，会减速并保持最终指令位置，正常停止后回到 `LOCKED`；持续发布的新指令可以重新启动 Servo，
因此主动停止时也应停止目标发布。逆解、碰撞、反馈等错误进入 `FAULT`，原因记录在
`GetStatus().motion_error`，需处理原因并 `unlock → reset → 重新对准/lock` 后恢复。
运行 Servo 时，其余运动/阶段切换命令会立即拒绝，不会在后台排队。
命令行对应 `servo <angle_rad> <displacement_m> [v]`；单次输入只控制到下一次更新或超时，
持续控制应由调用程序按周期发布。仿真和真机共用以上接口与规划流程。

普通停止使用 `stop_acceleration`（默认 2.0 rad/s²）约束关节指令减速度，
双臂及轮盘参考采用同一减速比例。停止过程可能沿原速度方向继续一小段，
逐点检查关节范围和碰撞；不会瞬间把目标切到实测位置。反馈故障、
SDK 错误或制动路径不安全时尝试停止两臂 SDK 控制并进入 `FAULT`，不继续生成保持位置指令。
该参数约束程序生成的制动指令，不替代控制器保护，也不承诺故障下机械臂的实际减速度。

## 仿真实现与检查范围

MuJoCo 使用项目内 `model/aviator.xml` 及 `aviator_home` 关键帧。与原版一致，
无 actuator 的 14 个关节通过 `qfrc_applied = qfrc_bias + 1000*(target-qpos)` 伺服，
增加 80 的关节阻尼；weld 开关使用 `mjData::eq_active`。实时模式持续推进物理，
无头演示使用仿真时间和锁步同步。渲染只在复制物理状态时持锁。

碰撞检测保留原版的圆柱掩码：抓取圆柱只检查另一抓取圆柱，其他非圆柱几何按 SRDF 检查。
它并不检测抓取圆柱与座舱等所有物体的接触；此处与参考项目保持一致。

CTest 包含启动姿态、后端选择（无网络连接）、完整仿真验收、共用入口演示、
开环抓取阶段与故障判据、双臂使能回滚，运动中交互停止、速度倍率和速度上限、Servo 连续换目标、停止/超时及碰撞故障。验收使用 MuJoCo 被动关节的实际位置检查转角误差 < 0.005 rad、
推拉误差 < 1 mm，接近误差 < 2 mm / 0.01 rad，并逐物理步检查两臂 J2 满足
`posture.json` 的限制（当前为 [85°, 94°]）。验收还覆盖推拉 -170 mm 时左右转动 50°。
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
