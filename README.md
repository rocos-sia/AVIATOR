# AVIATOR

Autonomous Versatile Intelligent Arm-based Teleoperation and Operation Robot

按 [软件架构设计方案](docs/AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md) 第 15 章建立新工程目录。当前阶段仅完成目录与 CMake 入口，节点、公共库、协议及运行配置尚未实现。

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

C++17；当前骨架最低 CMake 3.22，无外部运行库依赖。配置与构建：

```bash
cmake --preset linux-debug
cmake --build --preset linux-debug --parallel
ctest --preset linux-debug
```

提供 linux-debug、linux-release、simulation、replay 四组 configure/build/test presets。simulation 和 replay 当前只预留独立构建目录，尚未实现模式差异。当前没有可执行 target 或测试；构建成功仅验证工程入口。安装规则目前只安装文档，不安装占位配置。

旧工程、模型和辅助工具保留原位置，不进入新根工程构建。目录归属与后续迁移见 [项目目录与迁移说明](docs/项目目录与迁移说明.md)。
