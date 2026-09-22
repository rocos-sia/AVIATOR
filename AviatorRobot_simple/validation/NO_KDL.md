# 移除项目 KDL 依赖

日期：2026-09-22。范围：AviatorRobot_simple；不修改原 AviatorRobot、pin_ik-main 或厂家 SDK。

## 实现

- `Kinematics`、抓取几何、运动规划和真机坐标变换统一使用 `pinocchio::SE3`。
- PIN-IK 直接接收 SE3，Pinocchio 直接返回 FK；删除往返 KDL 的转换。
- 正式 IK 验收保留物理限位、J2 规划限位、位置误差 1e-6 m 和姿态误差 1e-6 rad。
  位置误差使用平移差的模长，姿态误差使用相对旋转角度。
- 接近运动使用位置线性插值、四元数最短路径 slerp 和原五次平滑进度。
  不使用会耦合平移和旋转的 SE3 螺旋插值。
- `Pose.hpp` 只有位姿插值、姿态误差及 SDK 数组读写四个自由函数，没有新增类层次。
- 配置四元数仍为 wxyz；URDF 按字段显式构造 Eigen 四元数。
  SDK 的 4x4 行优先矩阵逐元素读写，不依赖 Eigen 的内部存储布局。
- 保留 URDF 固定安装链检查、工具逆变换、法兰局部 -Z 退让方向。
  IK 目标为 aircraft 系下的法兰；SDK TCP 为臂基座系下的工具圆柱中心。
- 删除 orocos-kdl、kdl_parser、trac_ik 源码及构建目标、旧参考求解器。
  保留 PIN-IK 需要的 NLopt、urdfdom 等依赖。安装规则排除旧依赖目录可能遗留的独立 KDL 动态库。
- 运动学回归改用 MuJoCo 在相同关节角下的 aircraft→法兰位姿，独立于 Pinocchio 求解器。
  原 PIN-IK 小模型测试继续覆盖解析目标、连续关节及窄限位边界。

## 验证方法

旧构建目录移出项目，重新在 `build/` 配置和编译；网络命名空间关闭网络：

```sh
unshare --user --map-root-user --net cmake -S AviatorRobot_simple -B AviatorRobot_simple/build -DCMAKE_BUILD_TYPE=Release
unshare --user --map-root-user --net cmake --build AviatorRobot_simple/build -j4
unshare --user --map-root-user --net env -u LD_LIBRARY_PATH LD_BIND_NOW=1 ctest --test-dir AviatorRobot_simple/build --output-on-failure
```

测试覆盖：

- SDK 已知行优先矩阵的双向转换、已知工具偏移和逆变换。
- 非单位起始姿态下的平移直线性、最短旋转、端点及 0/微小角/π 附近旋转。
- 双臂共 200 组物理范围内的 FK，与 MuJoCo 对比。
- 双臂共 80 组附近 IK 目标，包括 J2 规划上下界；MuJoCo 独立核对逆解残差。
- 不可达目标、非有限种子、失败时输出不变、关节限位、种子连续性。
- 原有 MoveWheel、ServoWheel、完整 demo、状态机和异常处理测试。

## 验证结果

- 从空构建目录离线编译完成，包含真机 SDK 后端。首次构建发现新增位姿测试缺少项目头文件路径，补齐后最终构建通过。
- CTest **14/14 通过，79.48 秒**；包括 MoveWheel、ServoWheel 和两种完整演示。
- 200 组 FK 与 MuJoCo 对比：最大位置差 **1.8053e-13 m**，最大姿态差 **2.17621e-13 rad**。
- IK **80/80 成功**；MuJoCo 核对的最大位置残差 **1.18517e-7 m**，姿态残差 **6.47316e-7 rad**。
- IK P50 **5.1152 ms**、P95 **5.15685 ms**、最大 **5.25407 ms**；这是本机本次测试数据，不是实时上界。
- 最大种子关节变化 **0.00353045 rad**；仿真实测 J2 为 **85.4956°～94.5002°**。
- 窗口输入测试通过：左键旋转、右键平移、滚轮/中键缩放、失焦释放、R 重置、Esc 关闭。
- 安装到新的临时目录并移动目录后，取消 `LD_LIBRARY_PATH`，`ldd` 全部解析成功；非系统依赖均来自安装目录。
- 从项目外运行移动后的安装程序 `aviator --servo-demo`，开启真实 GLFW 窗口，取消 `LD_LIBRARY_PATH` 并启用 `LD_BIND_NOW=1`；完整运行至 `Demo completed`，退出码 0。
- 控制库及 PIN-IK 动态符号无 KDL/TRAC-IK；新构建和安装目录均无独立 KDL/TRAC-IK 库。
- `AVIATOR_WITH_SIMULATION=OFF`、`BUILD_TESTING=OFF` 的独立 CMake 配置通过。该配置仅验证生成；完整编译使用仿真和真机后端均开启的默认配置。

日志：[构建](no-kdl-build.txt)、[CTest](no-kdl-ctest.txt)、[测试详细输出](no-kdl-ctest-details.txt)、
[安装依赖检查](no-kdl-installed-audit.txt)、[窗口输入测试](no-kdl-viewer.txt)、[安装后窗口演示](no-kdl-servo-gui.txt)。

## 厂家 SDK 边界

xCore SDK 0.7.1 的闭源静态库内部包含 KDL。本次保留厂家库和已有授权说明，
并保留独立 DSO 与局部符号绑定。移除的是项目自身的源码、构建和独立动态库依赖，
不宣称闭源 SDK 内部不存在 KDL。PIN-IK 源码的上游版权和来源声明同样保留。

未连接真机；真机部分仅验证构建、链接及共用位姿转换。全程关节阻抗的 SDK 调用不变，
实际双臂运行仍需连接硬件验证。
