# 入口与测试代码简化

- 删除 `src/application.cpp` 和 `src/application.hpp`，CMake 的入口只编译 `src/main.cpp`。
- `main()` 直接展示 Init、Enable、ApproachHandles、LockHandles、目标循环、UnlockHandles、Disable。
  `--demo` 使用 MoveWheel；`--servo-demo` 每 20 ms 直接调用 ServoWheel，并明确演示 Stop 和等待结束。
- 交互命令在同一文件末尾处理，去掉 dispatch 回调和 std::function 动作封装。
  必要的窗口主线程、阻塞动作线程仍保留，运动期间可 status/stop。
- 基础测试从姿态配置读取期望值，使用配置入口初始化；完整验收删除 Probe 辅助类及重复后端创建。
- 后端选择测试删除未使用的配置生成器，合并重复的模型初始化与异常检查。
- 原 test_wheel_control 拆为 test_move_wheel 和 test_servo_wheel，各自按调用顺序展示测试。
  保留测量逐周期下发速度所需的记录器、注入异步碰撞所需的检查器。
- 使能回滚只保留一个右臂使能失败的替身，删除不必要的运动学与碰撞替身。
- 命令行测试改为顺序读取进程输出，删除后台读取线程和队列；增加非法参数与无换行 EOF 检查。
- Aviator 控制算法、DataLink、Viewer 和 SDK 关节阻抗实现未改动。

## 验证

| 项目 | 结果 |
|---|---|
| Release 编译，包含 MuJoCo 与 xCore SDK | 通过 |
| 默认 CTest | 11/11 通过，86.18 s |
| 完整 MoveWheel 演示与实际轮盘/J2 验收 | 通过 |
| Servo 连续目标、跟踪、停止、超时、碰撞错误与恢复 | 通过 |
| 窗口 Servo 示例 | 通过，Demo completed |
| 窗口交互，运动中 status/stop、quit、EOF | 通过 |
| 相机旋转、平移、缩放、复位与退出 | 通过 |

日志：[编译](simple-build.txt)、[CTest](simple-ctest.txt)、[测试详细输出](simple-ctest-details.txt)、
[窗口 Servo](simple-servo-gui.txt)、[窗口交互](simple-interactive-gui.txt)、[相机](simple-camera.txt)。

真机未连接，本次未发送真机控制命令。
