> 历史记录：仓库现为纯 C++ 项目，Python 绑定及 SWIG 依赖已删除，文中相关内容不再适用。

> 历史迁移记录：当前 Pinocchio 3.9.0 实现及连续关节接口见 [文档首页](README.md)。本文中的进度与版本差异描述可能已过时。

# PIN-IK with Pinocchio

PIN-IK (Track Inverse Kinematics) 现在使用 Pinocchio 作为其运动学后端，替代了原来的 KDL (Kinematics and Dynamics Library)。

## 依赖项

### 必需依赖

1. **CMake** >= 3.16
2. **C++17** 编译器
3. **Eigen3** >= 3.0
4. **Pinocchio** >= 2.0
5. **NLopt** - 非线性优化库
6. **Boost** - filesystem 和 serialization 组件
7. **urdfdom** - URDF 解析

### 可选依赖

- **Python 3** (用于 Python 绑定)
- **SWIG** (用于 Python 绑定)

## 安装依赖

### Ubuntu/Debian

```bash
# 安装基础依赖
sudo apt-get update
sudo apt-get install cmake build-essential libeigen3-dev libboost-all-dev

# 安装 NLopt
sudo apt-get install libnlopt-dev libnlopt-cxx-dev

# 安装 urdfdom
sudo apt-get install liburdfdom-dev

# 安装 Pinocchio (如果还没有安装)
# 方法 1: 从源码编译 (推荐)
# 参考: https://stack-of-tasks.github.io/pinocchio/download.html

# 方法 2: 使用 ROS 2
sudo apt-get install ros-humble-pinocchio

# 方法 3: 使用 robotpkg
# 参考: http://robotpkg.openrobots.org/
```

## 编译

```bash
cd /path/to/pin_ik
mkdir build && cd build

# 如果 Pinocchio 安装在 /opt/openrobots
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots

# 或者使用 ROS 2 环境
source /opt/ros/humble/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
make -j$(nproc)

# 安装 (可选)
sudo make install
```

### 编译选项

- `PIN_IK_BUILD_PYTHON=ON/OFF` - 构建 Python 绑定 (默认: ON)
- `PIN_IK_BUILD_EXAMPLES=ON/OFF` - 构建示例 (默认: ON)
- `BUILD_TESTING=ON/OFF` - 构建测试 (默认: ON)

```bash
cmake .. -DPIN_IK_BUILD_PYTHON=OFF -DPIN_IK_BUILD_EXAMPLES=OFF
```

## 使用

### C++ API

```cpp
#include <pin_ik/pin_ik.hpp>
#include <pinocchio/parsers/urdf.hpp>

// 方法 1: 从 URDF 字符串初始化
std::string urdf_xml = "...";  // 你的 URDF XML 字符串
PIN_IK::PIN_IK ik_solver("base_link", "tip_link", urdf_xml, 
                            0.005,  // 超时时间（秒）
                            1e-5,   // 精度
                            PIN_IK::Speed);  // 求解类型

// 方法 2: 从 Pinocchio Model 初始化
pinocchio::Model model;
pinocchio::urdf::buildModelFromXML(urdf_xml, model);
Eigen::VectorXd q_min(model.nq), q_max(model.nq);
// 设置关节限制...
pinocchio::FrameIndex tip_frame_id = model.getFrameId("tip_link");
PIN_IK::PIN_IK ik_solver(model, q_min, q_max, tip_frame_id, 0.005, 1e-5);

// 设置目标位姿
pinocchio::SE3 target_pose;
target_pose.translation() = Eigen::Vector3d(0.5, 0.2, 0.3);
target_pose.rotation() = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();

// 初始关节配置
Eigen::VectorXd q_init = Eigen::VectorXd::Zero(model.nq);

// 求解 IK
Eigen::VectorXd q_solution;
int result = ik_solver.CartToJnt(q_init, target_pose, q_solution);

if (result >= 0) {
    std::cout << "找到 " << result << " 个解" << std::endl;
    std::cout << "最佳解: " << q_solution.transpose() << std::endl;
} else {
    std::cout << "未找到解" << std::endl;
}
```

### Python API

```python
from pin_ik_python.pin_ik import IK

# 读取 URDF
with open('robot.urdf', 'r') as f:
    urdf_str = f.read()

# 创建 IK 求解器
ik_solver = IK("base_link", "tip_link",
               timeout=0.005,
               epsilon=1e-5,
               solve_type="Speed",
               urdf_string=urdf_str)

# 初始关节位置
q_init = [0.0] * ik_solver.number_of_joints

# 目标位姿 (x, y, z, qx, qy, qz, qw)
target_pose = (0.5, 0.2, 0.3,   # 位置
               0.0, 0.0, 0.0, 1.0)  # 四元数

# 求解 IK
solution = ik_solver.get_ik(q_init, *target_pose)

if solution:
    print(f"找到解: {solution}")
else:
    print("未找到解")
```

### 求解类型

- `Speed` - 快速找到第一个解（默认）
- `Distance` - 最小化与初始配置的距离
- `Manip1` - 最大化操纵度（行列式）
- `Manip2` - 最大化条件数
- `Manip3` - 最大化最小奇异值

### 边界约束

可以指定位置和姿态的容差：

```cpp
pinocchio::Motion bounds;
bounds.linear() = Eigen::Vector3d(0.001, 0.001, 0.001);  // xyz 容差
bounds.angular() = Eigen::Vector3d(0.01, 0.01, 0.01);    // 旋转容差

int result = ik_solver.CartToJnt(q_init, target_pose, q_solution, bounds);
```

Python:
```python
solution = ik_solver.get_ik(q_init, *target_pose,
                            bx=0.001, by=0.001, bz=0.001,  # 位置容差
                            brx=0.01, bry=0.01, brz=0.01)  # 旋转容差
```

## 运行测试

```bash
cd build

# 运行单元测试（需要测试 URDF 文件）
./tests/standalone_tests /path/to/test.urdf

# 运行 IK 测试
./examples/ik_tests /path/to/robot.urdf base_link tip_link 100
```

## 与 KDL 版本的区别

### 主要变更：

1. **数据类型**:
   - `KDL::Frame` → `pinocchio::SE3`
   - `KDL::JntArray` → `Eigen::VectorXd`
   - `KDL::Twist` → `pinocchio::Motion`

2. **API**:
   - 需要显式指定末端执行器帧 ID (`pinocchio::FrameIndex`)
   - 使用 Pinocchio Model 而不是 KDL Chain

3. **性能**:
   - Pinocchio 通常比 KDL 更快
   - 更好的算法实现和优化

### 迁移指南

查看 `MIGRATION_REPORT.md` 了解详细的迁移说明和 API 对应关系。

## 故障排除

### CMake 找不到 Pinocchio

```bash
# 设置 CMAKE_PREFIX_PATH
cmake .. -DCMAKE_PREFIX_PATH=/opt/openrobots

# 或者设置 pinocchio_DIR
cmake .. -Dpinocchio_DIR=/opt/openrobots/lib/cmake/pinocchio
```

### 找不到 NLopt

```bash
# 确保安装了开发包
sudo apt-get install libnlopt-dev libnlopt-cxx-dev
```

### 编译错误

确保使用 C++17 或更高版本：
```bash
cmake .. -DCMAKE_CXX_STANDARD=17
```

## 文献引用

如果在研究中使用 PIN-IK，请引用：

```
@article{beeson2015trac,
  title={PIN-IK: An open-source library for improved solving of generic inverse kinematics},
  author={Beeson, Patrick and Ames, Barrett},
  journal={IEEE-RAS International Conference on Humanoid Robots},
  year={2015}
}
```

## 许可证

PIN-IK 使用 BSD 3-Clause 许可证。详见 LICENSE 文件。

## 贡献

欢迎贡献！请提交 Pull Request 或报告 Issues。

## 相关链接

- [Pinocchio 文档](https://stack-of-tasks.github.io/pinocchio/)
- [原始 PIN-IK](https://bitbucket.org/traclabs/pin_ik/)
- [Eigen 文档](https://eigen.tuxfamily.org/)
