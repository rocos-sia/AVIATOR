> 历史记录：仓库现为纯 C++ 项目，Python 绑定及 SWIG 依赖已删除，文中相关内容不再适用。

# 本地验证记录

2026-09-20，Linux x86_64，GCC 11.4，CMake，Python 3.10.12，SWIG 4.0.2。
NLopt 2.7.1 从 Ubuntu deb 解压到 `/tmp/pin-ik-deps/root/usr`，没有安装 ROS 或新增系统软件包。
环境已有 KDL、Eigen 和 urdfdom；测试使用独立系统库。此验证不是无 ROS 容器测试，
但 `ldd` 确认核心库及 Python 扩展的完整动态依赖没有 ROS 库。

- Release：构建核心库、Python 扩展、示例和测试；CTest 3/3 通过。
- Debug（Python 关闭）：构建及 CTest 2/2 通过。
- C++：五种求解模式；FK 位姿误差及限位；100 个 6R 串联臂目标（初值为目标关节值加 0.1）；
  URDF 固定工具变换和旋转关节原点的解析校验；安全限位、连续关节、无效链/模型/初值/限位及不可达超时。
- 示例：混合移动/旋转关节模型，100 个固定随机种子的目标，统一中点初值，100/100 成功；
  FK 容差 1e-4，检查全部关节限位。
- Python：五种模式的已知位姿、关节名称、限位读写、不可达目标、无效模型/模式/四元数/初值。
- 安装到 `/tmp/pin-ik-install` 后，独立 CMake 工程通过 `find_package(pin_ik)`
  构建并完成 100/100 示例目标；安装后的 Python 包也通过测试。
- `git diff --check` 通过。

复现构建命令见根 README。测试中的 malformed XML 日志是预期的负例输出。
测试不等于所有机器人模型的性能保证。历史 ROS 文件及 MoveIt 插件现已删除。
CI 已改为独立 Release/Debug 构建，本地任务未触发远端 CI。
