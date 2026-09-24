# 构建与依赖配置

本目录用于后续依赖版本、校验值、导入 target 与工具链维护。交叉编译需求确定后再添加 toolchains/。

当前骨架使用 CMake 3.22 及 Preset 格式 3，以兼容当前开发机；架构建议的 3.24 及发布工具链版本仍需在依赖集成阶段冻结。每个实际 target 使用 target_compile_features(... cxx_std_17)。

第三方库仅在实际使用时接入，不自动下载或追踪浮动分支。可作为子工程的源码使用 add_subdirectory；独立构建的依赖安装到 build/third_party/install 后 find_package。同一依赖只选一种来源，统一编译器、Eigen 与 ABI，默认关闭无关示例、测试和语言绑定。
