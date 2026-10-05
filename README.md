# AVIATOR

Autonomous Versatile Intelligent Arm-based Teleoperation and Operation Robot

按 [软件架构设计方案](docs/AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md) 第 15 章建立新工程目录。当前已实现 common 的通信公共库（ZMQ 两帧收发、公共协议编解码、时效与授权检查），独立 `aviator_bus` 总线、`aviator_monitor` Web 监控与 `flight_gateway` 的 USB 输入路径已接入（见节点说明中的输入时效限制）；RS422、其余业务节点、完整业务 Schema 和运行配置仍待实现。

```text
AVIATOR/
├── CMakeLists.txt / CMakePresets.json
├── cmake/          # 依赖与构建规则
├── third_party/    # 共享第三方依赖
├── common/         # protocol、transport、runtime、recording
├── nodes/          # 运行节点
│   ├── aviator_bus/
│   ├── flight_gateway/
│   ├── aviator_core/
│   ├── manipulator/
│   ├── aviator_hand/
│   ├── camera/
│   ├── aviator_logger/
│   ├── aviator_monitor/
│   ├── aviator_plotter/
│   └── aviator_replay/
├── config/         # system、robot、camera、recording YAML 占位
├── schemas/        # 消息与记录信封定义
├── tests/          # CTest 测试入口
├── deploy/         # systemd 与设备权限规则
└── docs/           # 架构与迁移说明
```

C++17，最低 CMake 3.22。通信库需要 libzmq、cppzmq、nlohmann/json 和 spdlog（Ubuntu：`sudo apt install libspdlog-dev`）；完整构建还包含机器人与示例依赖，详见 [构建说明](docs/构建系统说明.md)。默认编译所有已接入的节点、示例和测试，以及 vendored Coal、Pinocchio、PIN-IK、MuJoCo 依赖：

```bash
mkdir -p build
cd build
cmake ..
make
```

应用程序输出到 `build/bin`，测试可执行文件（包括手动查看器测试）统一输出到 `build/tests`；通过 `ctest --test-dir build --output-on-failure` 运行自动测试。

默认使用 Release；首次构建会自动编译第三方库并安装到 `build/third_party/install`，无需预先安装 Pinocchio 或手动设置其头文件路径。系统开发包仍需按构建说明安装。第三方库默认使用 2 个编译任务，可通过 `-DAVIATOR_DEPENDENCY_JOBS=N` 调整。

仅构建通信库：

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel
ctest --test-dir build/communication --output-on-failure
```

提供 linux-debug、linux-release、simulation、replay 四组 configure/build/test presets。simulation 和 replay 当前只预留独立构建目录，尚未实现模式差异。通信测试已接入 CTest，接口和实现边界见 [common](common/README.md)。安装包含公共库、头文件、aviator_bus、flight_gateway、aviator_monitor 和文档，不安装占位配置。

旧工程、模型和辅助工具保留原位置；默认完整构建包含已接入的 examples，通信独立构建跳过这些示例。目录归属与后续迁移见 [项目目录与迁移说明](docs/项目目录与迁移说明.md)。

### 节点启动信息

手和相机的操作入口：

| 模块 | 使用说明 |
| --- | --- |
| 因时手 | [构建、双 CAN 配置、ZMQ 手动控制、实际位置反馈与只读检查](nodes/aviator_hand/README.md) |
| RealSense 相机 | [ChArUco/AprilTag 选择、实时识别预览、位姿打印与订阅](nodes/camera/README.md) |
| 联合录制 | [相机与 Logger 启动顺序、共用 session、业务/图像分文件保存](nodes/camera/README.md#检测与-mcap-录制) |

以上操作示例从仓库根目录运行。手和相机可复用同一个 `aviator_bus`：发布者连接 `5555`、
订阅者连接 `5556`；图像单独发送至 Logger 的 `5557`。视频预览和位姿打印默认关闭，
调试时通过 `--show --print-pose` 开启。手控制通过独立的 `hand_command.py` 测试发布器完成。

`simulation`、`aviator_bus`、`aviator_monitor` 和 `flight_gateway` 启动后会打印统一的信息面板，列出实际 PUB/SUB 或 XSUB/XPUB 地址、发布/订阅话题，以及 Web 地址或运行模式。端口会随命令行参数更新。终端支持彩色标题；重定向日志或设置 `NO_COLOR=1` 时输出纯文本。

`bind` 表示本地监听，`connect` 表示异步连接配置，不代表已收到总线消息。bus 透明转发订阅的话题，monitor 订阅所有话题；实际收包情况请在 monitor 的话题列表中查看。
