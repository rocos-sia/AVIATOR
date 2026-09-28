# 新基座布局的抓取与姿态验证

项目：`examples/AviatorRobot_simple`。从仓库根目录 `CMakeLists.txt` 构建，输出位于 `build/bin` 和 `build/tests`。

## 模型与配置

以用户修改后的 `config/aviator_control.urdf` 为基准，保留基座位置和朝向：

| 机械臂 | 基座 XYZ (m) | 基座 RPY (rad) |
|---|---|---|
| 左 | -0.91333, 0.010, 0.12156 | -1.5707, 1.3962671, 0 |
| 右 | -0.91333, -0.010, 0.12156 | 1.5707, 1.3962671, 0 |

原碰撞 URDF 仍使用旧基座变换，已同步。MuJoCo 基座四元数由上述 RPY 精确转换，统一使用 `[w, x, y, z]` 顺序。

- `joint2_limits_deg`：85°～94°；保留 0.5° 规划余量，逆解范围为 85.5°～93.5°。
- 抓取位置：两侧均从原位置沿把手轴线正向移动 30 mm，目前位于把手中心沿轴线正向 10 mm 处。
- 抓取朝向：左侧保留；右侧绕抓取坐标系 Z 轴（把手轴线）旋转 15°。抓取圆柱与把手保持同轴。
- home：零位抓取法兰目标沿法兰负 Z 方向后退 120 mm 所对应的关节解。
- approach seed：后退 60 mm 所对应的关节解。
- MuJoCo 的 handle site 和 `aviator_home` 关键帧与配置同步。
- 保留用户设置的轮盘速度上限 0.08 rad/s、0.008 m/s，以及网络和 SDK 修改。

以下角度仅为便于阅读的近似值，程序使用 `config/posture.json` 中的完整精度：

| 配置 | J1～J7 (°) |
|---|---|
| left_home_deg | 28.2939, 89.9304, -77.1307, 125.9052, 11.0487, 2.1802, 9.4231 |
| right_home_deg | -27.1571, 85.5000, 71.9546, 123.5535, -6.3939, 4.4791, -17.9500 |
| left_approach_seed_deg | 15.9308, 91.5427, -78.6826, 120.9931, 9.4824, -5.1071, 7.5000 |
| right_approach_seed_deg | -15.3845, 87.2852, 73.9423, 118.9630, -3.4551, -2.5391, -15.9219 |

## 验证

在仓库根目录执行：

```bash
cmake --build build --target aviator aviator_basic_test aviator_kinematics_test aviator_acceptance_test -j4
./build/bin/aviator --demo --headless
ctest --test-dir build -R '^aviator_(basic|kinematics|acceptance|servo_demo)$' --output-on-failure -V
```

本次验证在无外部网络的命名空间中进行，未连接真机。完整 demo 成功，4 项 CTest 全部通过：

- `basic`：home 关键帧与关节配置一致，抓取配置与 MuJoCo handle site 一致。
- `kinematics`：控制 URDF、碰撞 URDF、MuJoCo 的 FK 一致；80/80 个逆解成功。
- `acceptance`：从偏离 home 的关节位置启动，回 home、预接近、抓握后依次执行下表，再解锁失能。生产程序的轨迹限位、速度、碰撞和跟踪检查均保持启用。
- `servo_demo`：接近、抓握、实时小幅轮盘控制与停止流程通过。

| 目标转角 (rad) | 推拉 (m) | 速度倍率 |
|---|---|---|
| 0.87266 | 0 | 0.5 |
| -0.87266 | 0 | 0.5 |
| 0 | 0 | 0.5 |
| 0 | -0.170 | 0.5 |
| 0.87266 | -0.170 | 0.5 |
| -0.87266 | -0.170 | 0.5 |
| 0 | 0 | 0.5 |

验收结果：J2 实测范围 **85.4901°～93.5002°**；七个目标的最大最终转角误差约 **5.79×10⁻⁷ rad**，最大最终推拉误差约 **0.000516 mm**。
运动学测试最大 FK 差异约 **5.71×10⁻¹⁶ m / 1.40×10⁻¹⁵ rad**，最大 IK 残差约 **5.73×10⁻⁷ m / 6.52×10⁻⁷ rad**。

这些结果覆盖上述配置与测试路径，不构成任意轮盘目标或任意真机初始姿态的可达性保证。
