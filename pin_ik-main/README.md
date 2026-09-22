# PIN-IK (Pinocchio Inverse Kinematics)

基于 [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 与 [NLopt](https://nlopt.readthedocs.io/) 的独立 C++17 逆运动学求解器，是 [TRAC-IK](https://github.com/traclabs/trac_ik) 从 Orocos KDL 到 Pinocchio 的移植版本。

PIN-IK 完全移除了 KDL、ROS、MoveIt 与参数服务器依赖，只需一个 URDF 字符串即可构造求解器，适合嵌入到非 ROS 的机器人软件中。

**特性**

- **双线程并行求解**：Jacobian 迭代求解器与 NLopt 全局优化器同时运行，先得到解者胜出
- **五种求解模式**：`Speed`、`Distance`、`Manip1`、`Manip2`、`Manip3`
- **支持连续关节**：正确区分配置空间 `nq` 与速度空间 `nv`，连续关节支持任意圈数
- **仅需 URDF**：自动提取 base → tip 的串联链，位姿以 base 为参考系
- **标准 CMake 包**：提供 `pin_ik::pin_ik` 导入目标与 `find_package(pin_ik)` 支持
- **无 ROS 依赖**：无 catkin/ament、无 MoveIt、无参数服务器

## 目录结构

```text
include/pin_ik/   公共头文件（库的对外接口）
src/              核心库实现（pin_ik / pinocchio_tl / nlopt_ik / urdf）
examples/         示例程序（ik_tests）
tests/            自动化测试与测试模型（robot.urdf）
tests/manual/     按需构建的手动诊断程序
scripts/          维护脚本（含历史迁移脚本）
docs/             技术文档
cmake/            安装包配置（pin_ikConfig.cmake.in）
```

各子目录的说明见 [tests/README.md](tests/README.md)、[examples/README.md](examples/README.md)、[scripts/README.md](scripts/README.md)。

## 快速开始

### 1. 依赖

必需：

| 依赖 | 版本 | 说明 |
| --- | --- | --- |
| CMake | ≥ 3.16 | 构建系统 |
| C++ 编译器 | C++17（GCC ≥ 7、Clang ≥ 5） | 需支持 `std::thread` |
| Eigen3 | 3.x | 线性代数 |
| Pinocchio | **3.9.0** | 运动学与雅可比计算 |
| NLopt | 2.x | 非线性优化 |
| urdfdom | — | URDF 解析（独立版本，非 ROS） |
| Boost | — | 序列化（Pinocchio 传递依赖） |

可选：`pkg-config`，用于简化依赖查找。

Ubuntu / Debian 上安装：

```bash
# 构建工具与核心依赖
sudo apt-get install build-essential cmake pkg-config \
  libeigen3-dev libnlopt-cxx-dev liburdfdom-dev

# Pinocchio 3.9.0（源码构建）
git clone --recursive --branch v3.9.0 https://github.com/stack-of-tasks/pinocchio
cd pinocchio && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openrobots \
  -DBUILD_PYTHON_INTERFACE=OFF
make -j"$(nproc)"
sudo make install
```

### 2. 构建与测试

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

ctest --test-dir build --output-on-failure
```

若 Pinocchio 安装在非默认前缀（如 `/opt/openrobots`），配置时指定前缀路径：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots
```

### 3. 运行示例

```bash
# 用法：ik_tests <urdf> <base_link> <tip_link> [随机样本数]
./build/examples/ik_tests tests/robot.urdf base tip 100
```

示例会随机采样目标位姿，对每个解做正运动学残差（容差 `1e-4`）与关节限位校验，最后打印成功率：

```text
Solved 100/100; FK tolerance 1e-4; joint limits checked
```

### 构建选项

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `PIN_IK_BUILD_EXAMPLES` | `ON` | 构建 `examples/ik_tests` |
| `BUILD_TESTING` | `ON` | 构建测试并注册 CTest 用例 |

仅构建核心库：

```bash
cmake -S . -B build-lib -DCMAKE_BUILD_TYPE=Release \
  -DPIN_IK_BUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build build-lib -j"$(nproc)"
```

## 安装与集成

### 安装

```bash
# 系统安装
sudo cmake --install build --prefix /usr/local

# 用户安装
cmake --install build --prefix ~/.local
```

安装产物：

```text
<prefix>/include/pin_ik/          头文件
<prefix>/lib/libpin_ik.so         共享库
<prefix>/lib/cmake/pin_ik/        CMake 包配置
<prefix>/bin/ik_tests             示例程序（若启用）
```

### 在自己的 CMake 项目中使用

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app LANGUAGES CXX)

find_package(pin_ik REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE pin_ik::pin_ik)
```

非默认安装前缀需要告知 CMake：

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="/path/to/pin-ik-prefix;/opt/openrobots"
```

`pin_ikConfig.cmake` 会自动 `find_dependency` Eigen3、Pinocchio、NLopt 与 Threads，因此使用方无需重复查找。

## 使用

### 从 URDF 构造求解器

```cpp
#include <pin_ik/pin_ik.hpp>
#include <fstream>
#include <iostream>

std::ifstream file("robot.urdf");
std::string urdf_xml((std::istreambuf_iterator<char>(file)), {});

// base/tip 为 URDF 中的 link 名，超时 5ms，位姿精度 1e-5
PIN_IK::PIN_IK solver("base_link", "tip_link", urdf_xml, 0.005, 1e-5,
                      PIN_IK::Speed);

pinocchio::Model model;
Eigen::VectorXd lower, upper;
solver.getModel(model);
solver.getLimits(lower, upper);

// seed / 解 / 限位均为 model.nv 个标量
Eigen::VectorXd seed = Eigen::VectorXd::Zero(model.nv);
Eigen::VectorXd solution;                       // 输出

pinocchio::SE3 target;                          // 目标位姿（以 base 为参考）
target.translation() << 0.5, 0.0, 0.5;
target.rotation().setIdentity();

const int n_solutions = solver.CartToJnt(seed, target, solution);
if (n_solutions < 0) {
  std::cerr << "未找到解\n";
} else {
  std::cout << "解: " << solution.transpose() << "\n";
}

// 本次求解得到的全部解（可能为空）
std::vector<Eigen::VectorXd> solutions;
solver.getSolutions(solutions);
```

### 从已有 Pinocchio 模型构造

```cpp
PIN_IK::PIN_IK solver(model, lower_limits, upper_limits, tip_frame_id,
                      0.005, 1e-5, PIN_IK::Distance);
```

`model` 可以是任意串联链模型，`tip_frame_id` 为要控制的目标 frame。

### API 速览

| 接口 | 说明 |
| --- | --- |
| `PIN_IK(base, tip, urdf_xml, timeout, eps, type)` | 从 URDF 字符串构造（自动提取链） |
| `PIN_IK(model, lb, ub, tip_frame_id, timeout, eps, type)` | 从 Pinocchio 模型构造 |
| `int CartToJnt(seed, target_pose, q_out, bounds = zero)` | 求解 IK，返回本次解的数量；`< 0` 表示失败 |
| `getModel(model)` / `getLimits(lb, ub)` | 取回模型与关节限位 |
| `getSolutions(solutions)` / `getSolutions(solutions, errors)` | 取回全部解（后者附带误差排序信息） |
| `setLimits(lb, ub)` | 更新关节限位 |
| `SetSolveType(type)` | 切换求解模式 |

`CartToJnt` 是阻塞调用：返回时要么已找到解，要么超时（`maxtime`）。

### 求解模式

| 模式 | 说明 | 适用场景 |
| --- | --- | --- |
| `Speed` | 最快得到任一解，不比较解的优劣 | 实时控制、笛卡尔轨迹跟踪 |
| `Distance` | 在所有解中选择关节空间距离 seed 最小者 | 减少不必要的关节运动 |
| `Manip1` | 最大化可操作度（最大化最小奇异值） | 远离奇异位形 |
| `Manip2` | 最小化条件数的倒数 | 兼顾各方向灵活性 |
| `Manip3` | 最小化可操作度变化 | 追求位形稳定性 |

`Distance` 与 `Manip*` 模式会等待到超时才返回，因此比 `Speed` 更慢但解质量更高。

### 连续关节与配置空间

PIN-IK 的对外接口统一使用 **`model.nv` 个标量**（连续关节为弧度，可传入/返回多圈角度）；而 Pinocchio 的原生配置 `q` 为 `model.nq` 维，连续关节占两个分量 `(cos θ, sin θ)`。两者不等价：

```cpp
// ✅ 直接调用 Pinocchio FK 前先转换
Eigen::VectorXd q = PIN_IK::toPinocchioConfiguration(model, angles);
pinocchio::forwardKinematics(model, data, q);

// ❌ 不要把 IK 返回的角度向量直接当作 Pinocchio 配置
```

要点：

- 连续关节的限位为 `(-inf, +inf)`；求解结果会给出靠近 seed 的等价角度。
- `getModel()` 返回原生 Pinocchio 模型，因此 `model.nq >= model.nv` 是正常现象。
- 支持任意轴向的 revolute、continuous、prismatic 关节；URDF `<limit>` 为唯一限位来源，不解析 `safety_controller` 软限位。

### 线程安全

- 单个 `PIN_IK` 实例不可被多个线程同时调用 `CartToJnt`（内部复用 `pinocchio::Data` 与解缓存）。
- 每个线程各持一个实例即可并行求解；同一次调用内部已经使用两个线程，无需再外包一层并行。

## 测试与诊断

自动化测试：

```bash
ctest --test-dir build --output-on-failure
```

| 用例 | 内容 |
| --- | --- |
| `cpp_standalone` | 五种求解模式、连续关节与跨 ±π、非根 base、非法参数、超时 |
| `example_random_ik` | 100 个固定随机种子目标，校验 FK 残差与关节限位 |

手动诊断程序（默认不构建，需显式指定 target）：

```bash
cmake --build build --target pin_ik_smoke pin_ik_model_debug
./build/tests/manual/pin_ik_smoke
./build/tests/manual/pin_ik_model_debug
```

细节见 [tests/README.md](tests/README.md) 与 [tests/manual/README.md](tests/manual/README.md)。

## 已知限制

- base 必须是 tip 的祖先，只能提取二者之间的关节链；链中至少需要一个可动关节。
- 不支持 mimic、floating、planar 及多自由度关节。
- 只使用 URDF 的位置限位，不解析 `safety_controller` 软限位。
- 求解失败时 `CartToJnt` 将 `q_out` 置为 seed 并返回负值，调用方需自行判断。

## 故障排除

**找不到 Pinocchio**

```bash
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

安装了多个 Pinocchio 版本时，可用 `-Dpinocchio_DIR=/opt/openrobots/lib/cmake/pinocchio` 显式指定。

**链接或运行时报错找不到 `libpin_ik.so` / `libpinocchio_*.so`**

安装时已写入 `INSTALL_RPATH_USE_LINK_PATH`，通常无需设置。若仍失败，确认库存在并在搜索路径中：

```bash
ls /opt/openrobots/lib/libpinocchio*
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
```

**升级依赖版本后构建失败**

更换 Pinocchio 版本后请使用全新的构建目录（`rm -rf build`），避免复用旧版本的 CMake 缓存。

## 文档

- [docs/README.md](docs/README.md)：文档索引与连续关节说明
- [docs/MIGRATION_STATUS.md](docs/MIGRATION_STATUS.md)：KDL → Pinocchio 迁移状态
- [docs/JACOBIAN_DETAILS.md](docs/JACOBIAN_DETAILS.md)：雅可比与配置空间/速度空间详解
- [docs/README_PINOCCHIO.md](docs/README_PINOCCHIO.md)：Pinocchio 使用指南
- [docs/FINAL_STATUS.md](docs/FINAL_STATUS.md)、[docs/NEXT_STEPS.md](docs/NEXT_STEPS.md)：项目状态与后续工作
- [CLAUDE.md](CLAUDE.md)：面向 Claude Code 的开发指南

## 许可证

BSD 3-Clause License，详见 [LICENSE.txt](LICENSE.txt)。

## 致谢

- 原始 TRAC-IK：TRACLabs, Inc.（PIN-IK 保留了其双线程求解架构）
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)：LAAS-CNRS / INRIA
- [NLopt](https://nlopt.readthedocs.io/)：Steven G. Johnson

## 引用

若在研究中使用本项目，请引用原始 TRAC-IK 论文：

```bibtex
@inproceedings{beeson2015trac,
  title={TRAC-IK: An open-source library for improved solving of generic inverse kinematics},
  author={Beeson, Patrick and Ames, Barrett},
  booktitle={IEEE-RAS International Conference on Humanoid Robots},
  year={2015}
}
```
