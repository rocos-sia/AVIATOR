# AVIATOR

Autonomous Versatile Intelligent Arm-based Teleoperation and Operation Robot

按 [软件架构设计方案](docs/AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md) 第 15 章建立新工程目录。当前已实现 common 的通信公共库（ZMQ 两帧收发、公共协议编解码、时效与授权检查），独立 `aviator_bus` 总线、`aviator_monitor` Web 监控与 `flight_gateway` 的 USB 输入路径已接入（见节点说明中的输入时效限制）；RS422、其余业务节点、完整业务 Schema 和运行配置仍待实现。

```text
AVIATOR/
├── CMakeLists.txt / CMakePresets.json
├── cmake/          # 依赖与构建规则
├── third_party/    # 共享第三方依赖
├── common/         # protocol、transport、runtime、recording
├── nodes/          # 九个运行节点
│   ├── aviator_bus/
│   ├── flight_gateway/
│   ├── aviator_core/
│   ├── manipulator/
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

C++17，最低 CMake 3.22。通信库需要 libzmq、cppzmq 和 nlohmann/json；完整构建还包含机器人与示例依赖，详见 [构建说明](docs/构建系统说明.md)。仅构建通信库：

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --parallel
ctest --test-dir build/communication --output-on-failure
```

提供 linux-debug、linux-release、simulation、replay 四组 configure/build/test presets。simulation 和 replay 当前只预留独立构建目录，尚未实现模式差异。通信测试已接入 CTest，接口和实现边界见 [common](common/README.md)。安装包含公共库、头文件、aviator_bus、flight_gateway、aviator_monitor 和文档，不安装占位配置。

旧工程、模型和辅助工具保留原位置；默认完整构建包含已接入的 examples，通信独立构建跳过这些示例。目录归属与后续迁移见 [项目目录与迁移说明](docs/项目目录与迁移说明.md)。
