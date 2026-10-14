# RH56 SDK 最小顺序示例

`main.cpp` 只有一个 `main()`，参数处理、报文构造、SDK 调用、应答检查和角度打印均在其中顺序执行，无辅助函数、回调、通信封装类或后台线程。默认读取一次六通道角度后断开；也支持仅初始化 485。新增的张开/闭合循环已用 `#if 0` 屏蔽，保留源码供后续排查。

## 构建与运行

在 AVIATOR 仓库根目录执行：

```bash
cmake -S examples/xCoreSDK_hand_minimal -B examples/xCoreSDK_hand_minimal/build
cmake --build examples/xCoreSDK_hand_minimal/build -j2
ctest --test-dir examples/xCoreSDK_hand_minimal/build --output-on-failure

# 只读一次角度后退出，ID 默认为 1
./examples/xCoreSDK_hand_minimal/build/rh56_sdk_demo 192.168.1.160 1

# 离线打印请求，不构造 SDK 对象、不连接设备
./examples/xCoreSDK_hand_minimal/build/rh56_sdk_demo 192.168.1.160 1 --dry-run

# 只初始化末端串口；成功后需要重启 AR 控制器
./examples/xCoreSDK_hand_minimal/build/rh56_sdk_demo 192.168.1.160 --init-485
```

用法：`rh56_sdk_demo ROBOT_IP [ID] [--init-485] [--dry-run]`。ID 范围为 1～254。`--init-485` 也可以和 `--dry-run` 一起使用。多余参数（包括旧版的六通道目标角度）会在连接前被拒绝。

## main() 中的 SDK 调用顺序

正常读取：

```text
rokae::ArRobot robot
robot.connectToRobot(ip)
robot.XPRS485SendData(int(tx.size()), int(rx.size()), tx, rx, ec)
检查应答，打印六通道角度
robot.disconnectFromRobot(ec)
```

初始化分支：连接后依次调用 `xPanelRS485Init(baud115200, none, stopbit1, ec)` 和 `setxPanelRS485Overtime(500, ec)`，然后断开，不发送手部报文。按[官方说明](https://docs.rokae.com/docs/SDK--0-7-x-AR/cpp/cpp_method/#xpanelrs485init)，AR 初始化后需重启控制器生效。普通读取沿用控制器当前参数和超时，不重复设置或读回。程序不保证初始化时请求的超时值在重启后保留。

机器人类型为 SDK 的 AR 专用 `rokae::ArRobot`。接收缓冲区先分配 20 字节，再按 `tx.size()` / `rx.size()` 传入长度。

ID=1 的读取请求为 `EB 90 01 04 11 0A 06 0C 32`，读取地址 1546 的 12 字节数据，期望 20 字节应答。ID 改变时在 `main()` 中重新计算累加校验和。收到完整应答后检查帧头、ID、长度、指令、地址和校验和，再按小端序显示角度协议刻度（不是度数）。

已屏蔽的循环代码原本只写目标角度寄存器 1486 (`0x05CE`)，前五通道在 1000/0 之间切换、保留拇指旋转，每条写入成功后等待 5 秒。当前构建不包含这段控制逻辑。

`TX（待发送）` 是准备好的请求，不保证已在物理线上发出；SDK 失败时打印的 RX 只是缓冲内容。若末端仅接电脑监听而没有设备回复，即使请求正常发出，也不会收到所需的 20 字节应答。

## SDK 与退出行为

使用 CMake 的 `XCORE_SDK_ROOT`，默认指向 `third_party/xCoreSDK-0.7.1`。头文件来自其 `include/`、`external/`，静态链接 `lib/Linux/x86_64/libxCoreSDK.a` 和 `libxMateModel.a`，更新 SDK 后需重新构建。SDK 日志保存在启动时打印的 `/tmp/rh56-sdk-XXXXXX/` 目录。

程序不调用电机上下电、模式切换或机械臂运动接口，张开/闭合控制已屏蔽。运行前关闭其他机器人客户端，保持机械臂静止。读取/初始化完成或报错均尝试断开，错误返回非零；为保持最小实现，不安装 Ctrl+C/SIGTERM 处理器，中断或强制终止不保证执行显式断开步骤，也不发送停止或回位指令。

`rh56.hpp` 和 `protocol_test.cpp` 保留为旧版协议参考及其离线测试，不参与当前 `rh56_sdk_demo` 的源码逻辑。CTest 另有当前程序的离线请求、ID 校验及旧写入参数拒绝测试；这些测试不证明真实硬件通信正常。
