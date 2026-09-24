# PIN-IK 文档

当前版本基于 **Pinocchio 3.9.0**，保留 TRAC-IK 的 Newton / NLopt 双求解器。

## 仓库目录

- `include/`：核心 C++ 库公共头文件。
- `src/`：核心 C++ 库及求解器实现。
- `examples/`：C++ 示例，构建产物为 `build/examples/ik_tests`。

- `tests/`：自动化测试、测试模型和 `manual/` 手动诊断程序。
- `scripts/`：维护及历史迁移脚本。

## 构建与使用

参见 [主 README](../README.md)。CMake 固定要求 Pinocchio 3.9.0，升级或降级依赖后请使用新的构建目录。

## 连续旋转关节

C++ 的关节输入、输出、限位长度均为 `model.nv`，每个关节一个标量。连续旋转关节使用弧度，允许多圈角度，限位为 `(-inf, +inf)`，返回靠近 seed 的等价解。

Pinocchio 3.9.0 的原生连续关节仍使用 `(cos θ, sin θ)`，即 `nq=2, nv=1`。例如六个关节中有一个 continuous 时，IK 输入长度为 6，Pinocchio 配置长度为 7。调用原生 FK 前使用 `PIN_IK::toPinocchioConfiguration(model, angles)` 转换。

求解器在标量关节空间更新角度、执行随机重启和 NLopt 优化；运动学和雅可比计算前统一转换配置，确保连续关节满足单位圆约束。支持 X/Y/Z 轴及任意旋转轴。

URDF 加载只保留 base 到 tip 的链，返回模型和目标位姿均以 base 为参考。有限关节采用 `<limit>`，不采用 `safety_controller` 软限位。

## 验证

```bash
cmake -S . -B build-3.9 -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/pinocchio-3.9.0
cmake --build build-3.9 -j2
ctest --test-dir build-3.9 --output-on-failure
```

测试覆盖五种求解模式、独立 Newton / NLopt 连续关节求解、跨 ±π 和多圈 seed、非根 base、100 个六轴目标、非法参数与超时。

## 历史文档

其余迁移报告记录早期迁移过程，不代表当前实现。尤其“3.x 连续关节只有一个配置分量”的历史描述不正确；当前 API 与构建方式以本页及主 README 为准。
