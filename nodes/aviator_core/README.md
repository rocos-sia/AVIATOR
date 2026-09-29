# aviator_core

从 `examples/AviatorRobot_simple` 迁移的双臂业务控制节点。保留 Home、接近、软件锁定、MoveWheel、ServoWheel、停止、复位和交互操作。PIN-IK、Pinocchio 碰撞检查和 Ruckig 规划都在本进程；不连接设备、不加载 xCore SDK，也不创建 MuJoCo 窗口。

从 AVIATOR 根目录构建：

```bash
cmake -S . -B build
cmake --build build --target aviator_bus aviator_core manipulator -j3
```

三个终端分别启动：

```bash
./build/bin/aviator_bus --config config/system.yaml
./build/bin/manipulator --config config/system.yaml
./build/bin/aviator_core --config config/system.yaml --demo
```

`manipulator --headless` 关闭仿真窗口。Core 的 `--servo-demo` 演示每 20 ms 更新目标；不加 Demo 选项进入交互模式：

```text
enable
approach
lock
wheel 0.1 -0.01 0.5
servo 0.02 -0.002 0.5
status
stop
unlock
disable
reset
quit
```

阻塞动作在工作线程执行，过程中可输入 status 或 stop；其他动作需等待 `[ok]`。Servo 超过 robot.yaml 的 servo_timeout 没有新目标则停止并保持。v 是速度倍率 `(0,1]`，不是时间。主线程保持任务监督心跳，通信线程独占 PUB/SUB，规划线程只操作强类型接口。

`main.cpp` 保留易读的顺序 Demo 和目标数组。`Aviator.cpp` 实现业务与规划；`RemoteLink` 将已检查的轨迹分成有界窗口发布，等待执行进度，不以实际关节/TCP 到位误差判断完成。初次使能从设备返回的实际保持目标开始，后续从上一条已下发目标衔接。仿真与真机共用上述代码。

本次采用 `local.task`：已显式使能的本地 Demo/交互任务，由 Core 主线程持续监督。它与 flight.command 分开授权，不伪造 FLIGHT/JOYSTICK 输入。当前未接入 flight.command 到轮盘目标的标定映射，也不实现视觉控制或手部驱动；flight.state 中 control_source=NONE，task_phase 表达本地任务阶段，缺失手部和视觉反馈报告无效。

已废除的 TCP 对齐、grasp.ready、跟踪误差及锁定丢失判据没有迁移。软件锁定只表示操作阶段。限位、IK 失败、碰撞规划、来源/时效、指令连续性、设备故障和取消仍独立生效。Home 完成后保留关节速度停稳检查。MuJoCo 实测轮盘跟踪只作为测试断言，不能反向成为运行准入条件。

协议能力、窗口与时序见 [ZMQ 文档第 19 节](../../docs/AVIATOR_ZMQ协议格式说明.md)。配置入口为根目录 `config/system.yaml`，不是旧示例的 `build/bin/config/aviator.yaml`。安装后默认查找 `share/aviator/config/system.yaml`；开发构建默认使用源码根目录配置，不依赖工作目录。
