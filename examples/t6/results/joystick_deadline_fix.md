# T6 摇杆闭环 10 ms 超时排查

2026-09-24，设备为 LiteStar PXN-F16 (`/dev/input/js0`)，驱动没有向当前用户
提供可用的震动接口。T6 控制回调此前每 20 ms 调用一次 `Device::poll()`；该函数
在震动不可用时每 2 秒重试，遍历对应的 `/dev/input/event*` 节点并执行 ioctl。
独立 350 次、每 10 ms 一次的实机摇杆采样中，第 197 次调用耗时
**17.356 ms**，发生在启动约 **1.983 s**，与用户日志的 1.89 s 停止点吻合。
ONNX 推理不是该次超时的原因：原记录的 T6 计算耗时峰值为 0.1135 ms。

T6 摇杆控制器现在以 `Device(path, false)` 读取轴和按钮，跳过震动设备探测；
独立采样的最大 `poll()` 耗时降为 **0.0107 ms**。网页摇杆程序仍默认启用震动功能。
`RunJointFeedback` 的超时日志现在包含周期序号和迟到毫秒数，保留原有 5 ms
迟到保护阈值。

修复后在独立 MuJoCo 总线上，以无 `-v` 的 headless 模式、同一 PXN-F16、
60k ONNX 权重运行 5 s：**501/501 帧 `ok`、500 条关节命令、0 次安全保持、
没有周期超时**，仿真反馈采样间隔均为 10 ms。结果见
[CSV](joystick_rumble_fix_5s.csv) 与 [日志](joystick_rumble_fix_5s.log)。

启动阶段的 `joints[0] is not Enabled` 来自底层 `Robot::IsEnabled()` 在真正
`SetEnabled()` 前的状态检查；此函数对“尚未使能”记录 error 级日志。
控制器随后成功使能并执行 190 帧（用户原记录）及 501 帧（修复后记录），
因此这些启动日志不是本次停止原因。
