后续已移除项目 KDL/TRAC-IK，当前测试见 [NO_KDL.md](NO_KDL.md)。本文为初次 PIN-IK 集成的历史记录。

# PIN-IK 替换验证

日期：2026-09-22。Ubuntu 22.04 x86_64，GCC 11，Release。

## 实现

- `third_party/pin_ik/`：从工作区 `pin_ik-main` 导入 2.2.0 源码快照，复用已有
  Pinocchio 3.9.0、Eigen 3.4.0、NLopt 2.7.0 及其依赖，不下载或引用原目录。
- `src/Kinematics_pin_ik.cpp`：每臂独立模型、FK Data 和 PIN-IK 实例；按
  `aircraft → 法兰` 提取七关节链，并校验关节名称与顺序。
- 原运动学实现移至 `tests/reference_kinematics_tracik.cpp`。TRAC-IK/kdl_parser
  只在测试中编译，正式控制库不包含旧求解器符号。
- 保留 KDL 位姿接口及原工具/安装变换，以旋转矩阵和平移逐元素转换，避免四元数及存储顺序混淆。
- 保留 Distance 模式、5 ms 求解预算、7e-7 精度；物理限位仍取自 URDF，
  J2 求解限位仍取姿态配置与规划余量。返回成功前检查有限值、规划限位及 FK 残差。
- 修正窄限位角度回绕及边界舍入、单边数值梯度分母、提前停止时目标函数未初始化的问题。
  PIN-IK 内部 NLopt 命名空间改为 PIN_IK_NLOPT，避免与参考 TRAC-IK 串用同名符号。
- Aviator 控制流程、MoveWheel/ServoWheel 接口、配置、真机关节阻抗模式保持不变。

原 `pin_ik-main` 源码快照校验一致，未修改。导入来源与本地源码校验记录在
`third_party/manifest.json`；第三方适配明细见 `third_party/pin_ik/README.vendor.md`。

## 测试结果

默认 CTest **13/13 通过，77.47 秒**，由原 11 项和新增的运动学对比、PIN-IK 独立测试组成。

| 检查 | 结果 |
|---|---|
| 200 组实际双臂姿态，Pinocchio FK 对比 KDL FK | 最大位置差 2.22045e-16 m |
| 80 个实际双臂附近目标，5 ms / Distance / 7e-7 | PIN-IK 80/80，TRAC-IK 80/80 |
| PIN-IK 返回解经独立 KDL FK 校验 | 最大位置残差 6.43988e-7 m；位置/姿态均通过 2e-6 阈值 |
| 附近目标相对种子的最大单轴变化 | 0.00353045 rad |
| J2 窄限位、边界目标、整圈种子 | 通过；另检查物理限位、不可达目标及非法种子 |
| PIN-IK 原独立测试 | 五种模式、连续关节、非根 base、100 个 6R 目标及超时检查通过 |
| 完整仿真验收 | 通过；实际 J2 范围 85.4957°～94.5002°，满足 [85°,95°] |
| MoveWheel 速度倍率、关节限速、完整演示 | 通过 |
| Servo 连续目标、停止、超时、故障与恢复 | 通过 |
| 实时节拍、后端参数、使能回滚和交互 | 通过 |
| 安装目录改名、项目外启动、清除 LD_LIBRARY_PATH、带窗口 Servo 演示 | Demo completed；动态库无缺失、无构建目录依赖 |

本轮单臂耗时（附近目标样本，包含线程创建/回收）：

| 求解器 | P50 | P95 | 最大值 |
|---|---:|---:|---:|
| PIN-IK | 5.13638 ms | 5.24086 ms | 5.46299 ms |
| TRAC-IK | 5.10692 ms | 5.18370 ms | 5.27256 ms |

这些数据不代表全部工作空间的成功率或硬实时保证。Distance 模式使用接近完整的求解预算，
双臂当前按顺序求解；IK 仍在规划/Servo 工作线程执行，xCore 的 1 ms 回调不求解 IK。

构建与完整 CTest 在 `unshare --user --map-root-user --net` 网络隔离环境中完成。
ELF 符号/依赖检查确认控制程序使用 libpin_ik.so，不包含 TRAC_IK/makeTracIkKinematics。
另配置验证 `BUILD_TESTING=OFF` 的构建图不包含 trac_ik、kdl_parser；未另做全量无测试构建。
真机未连接；本次真机部分只完成编译链接及连接前参数检查。

## 复现

```bash
cmake -S AviatorRobot_simple -B AviatorRobot_simple/build -DCMAKE_BUILD_TYPE=Release
cmake --build AviatorRobot_simple/build -j4
ctest --test-dir AviatorRobot_simple/build --output-on-failure
./AviatorRobot_simple/build/bin/aviator --demo
./AviatorRobot_simple/build/bin/aviator --servo-demo
```

日志：[构建](pin-ik-build.txt)、[CTest](pin-ik-ctest.txt)、
[详细测试输出](pin-ik-ctest-details.txt)、[安装版窗口 Servo](pin-ik-servo-gui.txt)、
[安装版依赖](pin-ik-installed-ldd.txt)。
