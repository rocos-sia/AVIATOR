# 构建与依赖配置

本目录管理第三方源码构建和导入 target。交叉编译需求确定后再添加 toolchains/。

当前骨架使用 CMake 3.22 及 Preset 格式 3，以兼容当前开发机；架构建议的 3.24 及发布工具链版本仍需在依赖集成阶段冻结。每个实际 target 使用 target_compile_features(... cxx_std_17)。

默认完整构建启用机器人库、MuJoCo、SDK、查看器及已接入的示例和测试；第三方库不自动下载或追踪浮动分支。可作为子工程的源码使用 add_subdirectory；独立构建的依赖通过 ExternalProject 安装到 build/third_party/install，由 imported target 传递链接路径和构建顺序。同一依赖只选一种来源，统一编译器、Eigen 与 ABI，默认关闭无关示例、测试和语言绑定。

在 `build` 目录执行 `cmake .. && make` 即可构建，无需先单独编译第三方库。使用 Pinocchio/PIN-IK 的目标应链接 `aviator_robotics_deps`，使用 MuJoCo 的目标应链接 `mujoco_imported`，以传递头文件路径及依赖构建顺序。缺少已启用的第三方源码时，配置阶段会直接报错。
