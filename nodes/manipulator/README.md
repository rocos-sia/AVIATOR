# manipulator

双臂设备执行节点，统一提供 MuJoCo 和 Rokae 后端。后台执行线程管理设备生命周期、消费有界轨迹窗口、执行本地插值并采样；ZMQ 和 JSON 位于非实时域。Rokae SDK 自己的两条周期回调只读写数值状态与指令，保持 SCHED_FIFO 80 和关节阻抗模式，保留实际调度及回调耗时诊断。

```bash
./build/bin/manipulator --config config/system.yaml
./build/bin/manipulator --config config/system.yaml --headless
```

`config/robot.yaml` 的 backend 选择 mujoco/rokae，viewer 控制 MuJoCo 窗口；--headless 优先。窗口支持鼠标旋转、平移、缩放、R 复位视角、Esc 退出。真机配置分别包含 left_ip、right_ip、left_local_ip、right_local_ip、joint_stiffness；只有此进程需要设备和实时调度权限。

启动保持未使能。Core 先通过可靠服务读取本次 server_session 和关节保持目标，再显式授权/使能。重启更换会话，旧请求不能对新进程生效；重连不自动使能。初次使能后的当前位置保持允许最多 1 s 的命令接入窗口，进入目标流后使用配置的 50 ms watchdog。没有指令时不会自动向 Home 运动。

设备指令采用双臂整体快照。2 ms 间隔的轨迹点在本地展开为 1 ms 位置点；两个 SDK 回调均取走上一条指令后才推进下一点，延迟不跳点追赶。执行进度反馈用于更新窗口，不能解释为两台控制器硬件同步，也不表示实际位置到位。

SDK 回调有独立命令截止时间，通信或执行线程停顿不能无限维持授权。执行线程在正常取消时从最后下发指令减速；命令失效、反馈失效或执行异常锁存错误并停止双臂。停止路径检查关节限位；设备故障不能完成减速时转为停止 SDK 控制。现场仍需验证实际时序和物理停止效果。

发布 arm.state（100 Hz）、hand.state（100 Hz）。ArmState 包含每侧原始采样时间、实际关节角/速度、TCP、已接纳命令和执行游标。Rokae TCP 使用各自 left_base/right_base，MuJoCo 使用 aircraft；四元数均为 qx/qy/qz/qw。SDK 适配分别设置工具的 trans/rpy 与实时齐次矩阵，保留控制器原有标定负载。真机 TCP/安装标定一致性需要现场验证。

当前硬件只有双臂反馈：hand.state 明确 OFFLINE/valid=false，关节测量为 null，grasp_verified=false。lock/unlock 是软件阶段；MuJoCo 额外切换把手 weld 以模拟物理连接，不做 TCP 对齐判断。wheel_reference 为指令参考，只有 MuJoCo 提供独立 wheel_measurement，不能把软件参考冒充真机传感器。

可靠服务包括 describe、authorize、enable、disable、stop、lock、unlock、reset_fault、get_result。单帧 REQ/REP，长操作先返回 ACCEPTED，客户端使用同一请求身份查询已有状态。服务缓存有界，不重复执行同 ID 请求；同 ID 改内容被拒绝。请求绑定 server_session，服务重启后旧请求必须对账，不能自动重放。接口约束见 [协议文档](../../docs/AVIATOR_ZMQ协议格式说明.md)。

独立 `nodes/simulation` 保留为另一套仿真测试入口；同一控制总线只能启动一个机械臂状态生产者。本节点使用迁移后的 `models/control` 模型，未覆盖原来的完整手部模型。

## 轮盘初始参考

`robot.yaml` 的 `wheel_initial: {angle: 0.0, displacement: -0.085}` 指定启动参考（rad/m）。角度范围 ±0.87266 rad，位移范围 [-0.170, 0] m；缺省保持旧版 (0, 0)。Rokae 仅初始化软件参考，不驱动机械轮盘归位；MuJoCo 在启动物理线程前设置 roll_input_joint / pitch_input_joint 实体初值。初始化后的 `arm.state.wheel_reference` 与该配置一致，后续由执行轨迹更新；使能、停止、重新授权不会重复重置初值。启动日志打印有效数值。修改后重启 Manipulator 及相连 Core。
