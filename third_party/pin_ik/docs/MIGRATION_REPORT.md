> 历史记录：仓库现为纯 C++ 项目，Python 绑定及 SWIG 依赖已删除，文中相关内容不再适用。

> 历史迁移记录：当前 Pinocchio 3.9.0 实现及连续关节接口见 [文档首页](README.md)。本文中的进度与版本差异描述可能已过时。

# PIN-IK 从 KDL 迁移到 Pinocchio - 完成报告

## 已完成的工作

### 1. 核心库文件替换

已将所有 KDL 数据结构和函数替换为 Pinocchio 等价物：

#### 新增文件：
- `include/pin_ik/pinocchio_types.hpp` - Pinocchio 类型定义和辅助函数
- `include/pin_ik/pinocchio_tl.hpp` - Pinocchio 版本的 IK 求解器头文件
- `src/pinocchio_tl.cpp` - Pinocchio 版本的 IK 求解器实现

#### 修改文件：
- `include/pin_ik/nlopt_ik.hpp` - 更新为使用 Pinocchio 数据类型
- `src/nlopt_ik.cpp` - 更新为使用 Pinocchio 运动学函数
- `include/pin_ik/pin_ik.hpp` - 更新主接口为 Pinocchio
- `src/pin_ik.cpp` - 更新主实现为 Pinocchio
- `include/pin_ik/urdf.hpp` - 更新 URDF 加载接口
- `src/urdf.cpp` - 使用 Pinocchio 的 URDF 解析器
- `CMakeLists.txt` - 替换 orocos_kdl 依赖为 pinocchio

#### 删除文件：
- `include/pin_ik/kdl_tl.hpp` (已删除)
- `src/kdl_tl.cpp` (已删除)

### 2. 数据类型映射

| KDL 类型 | Pinocchio 类型 |
|---------|---------------|
| `KDL::Chain` | `pinocchio::Model` |
| `KDL::JntArray` | `Eigen::VectorXd` |
| `KDL::Frame` | `pinocchio::SE3` |
| `KDL::Twist` | `pinocchio::Motion` |
| `KDL::Rotation` | `Eigen::Matrix3d` / `Eigen::Quaterniond` |
| `KDL::Vector` | `Eigen::Vector3d` |

### 3. 运动学函数映射

| KDL 函数 | Pinocchio 函数 |
|---------|---------------|
| `ChainFkSolverPos_recursive::JntToCart()` | `pinocchio::forwardKinematics()` + `pinocchio::updateFramePlacements()` |
| `ChainIkSolverVel_pinv` | Jacobian 伪逆 (使用 `pinocchio::computeFrameJacobian()`) |
| `ChainJntToJacSolver::JntToJac()` | `pinocchio::computeFrameJacobian()` |
| `KDL::diffRelative()` | 自定义 `PIN_IK::diffRelative()` (使用 `pinocchio::log6()`) |

### 4. 测试和示例更新

- `examples/src/ik_tests.cpp` - 更新为使用 Pinocchio API
- `tests/standalone_tests.cpp` - 更新为使用 Pinocchio API

### 5. Python 绑定更新

- `python/swig/pin_ik_wrap.i` - 更新 SWIG 接口为 Pinocchio

### 6. CMake 配置更新

- `cmake/pin_ikConfig.cmake.in` - 替换 orocos_kdl 依赖为 pinocchio
- 所有 CMakeLists.txt 文件已更新

## 依赖要求

### 必需依赖：
1. **Pinocchio** (>= 2.0) - 已在系统中找到：
   - `/opt/openrobots/lib/cmake/pinocchio/pinocchioConfig.cmake`
   - `/opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio/pinocchioConfig.cmake`

2. **Eigen3** (>= 3.0) - Pinocchio 的依赖，通常已安装

3. **NLopt** - 优化库，**需要安装**：
   ```bash
   sudo apt-get install libnlopt-dev libnlopt-cxx-dev
   ```

4. **casadi** (>= 3.4.5) - Pinocchio 的依赖，已在 `/opt/openrobots` 中找到

5. **Boost** - 文件系统和序列化组件

6. **urdfdom** - URDF 解析

## 后续步骤

### 1. 安装 NLopt
```bash
sudo apt-get update
sudo apt-get install libnlopt-dev libnlopt-cxx-dev
```

### 2. 编译项目
```bash
cd /home/think/Documents/GitHub/pin_ik
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots
make -j$(nproc)
```

### 3. 运行测试
```bash
# 假设有一个测试 URDF 文件
cd build
./tests/standalone_tests <path_to_test_urdf>
./examples/ik_tests <path_to_urdf> base_link tip_link 100
```

### 4. 安装库
```bash
cd build
sudo make install
```

## 主要变更说明

### 1. URDF 加载
- **之前**: 使用 KDL 的 URDF 解析器手动构建链
- **现在**: 使用 Pinocchio 的 `urdf::buildModelFromXML()` 构建完整模型，然后使用 `buildReducedModel()` 提取子链

### 2. 正运动学
- **之前**: `KDL::ChainFkSolverPos_recursive`
- **现在**: `pinocchio::forwardKinematics()` + `pinocchio::updateFramePlacements()`

### 3. 逆运动学求解
两个并行求解器：
- **NLOPT 求解器**: 使用非线性优化，基于 Pinocchio 正运动学
- **雅可比求解器**: 使用 Pinocchio 的雅可比计算和伪逆法

### 4. 关节限制处理
- 保持相同的限制检查逻辑
- 支持连续关节（无限制）、旋转关节和平移关节

### 5. 操纵度指标
使用 Pinocchio 的雅可比计算来计算奇异值，用于操纵度优化模式

## API 兼容性

### C++ API 变更：
```cpp
// 旧 API (KDL)
KDL::Chain chain;
KDL::JntArray q_min, q_max;
PIN_IK::PIN_IK solver(chain, q_min, q_max, timeout, epsilon);
KDL::Frame target;
KDL::JntArray seed, result;
solver.CartToJnt(seed, target, result);

// 新 API (Pinocchio)
pinocchio::Model model;
Eigen::VectorXd q_min, q_max;
pinocchio::FrameIndex tip_frame_id;
PIN_IK::PIN_IK solver(model, q_min, q_max, tip_frame_id, timeout, epsilon);
pinocchio::SE3 target;
Eigen::VectorXd seed, result;
solver.CartToJnt(seed, target, result);
```

### Python API：
Python API 保持不变，内部实现已更新为 Pinocchio。

## 性能考虑

Pinocchio 相比 KDL 的优势：
1. **更快的运动学计算** - 使用现代 C++ 和 Eigen 优化
2. **更好的算法** - 使用递归牛顿-欧拉算法 (RNEA)
3. **更丰富的功能** - 支持更多的关节类型和约束
4. **活跃维护** - Pinocchio 是一个活跃开发的项目

## 已知问题和注意事项

1. **帧索引**: Pinocchio 使用 `FrameIndex` 来引用末端执行器，而不是自动从链结构推断
2. **关节类型识别**: 通过 `joint.shortname()` 识别关节类型，可能需要根据实际 Pinocchio 版本调整
3. **URDF 子链提取**: 使用 `buildReducedModel()` 实现，需要测试各种 URDF 配置

## 测试建议

建议使用以下测试场景：
1. 简单的串联机器人（如 6R 臂）
2. 带有连续关节的机器人
3. 混合旋转和平移关节的机器人
4. 不同的求解模式（Speed, Distance, Manip1/2/3）
5. 边界约束
6. 各种初始位姿种子

## 联系和支持

如有问题或需要进一步的帮助，请参考：
- Pinocchio 文档: https://stack-of-tasks.github.io/pinocchio/
- PIN-IK 原始文档: https://bitbucket.org/traclabs/pin_ik/

---
迁移完成时间: 2024
