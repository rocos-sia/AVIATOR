# 测试

- `standalone_tests.cpp`：C++ 回归测试。
- `robot.urdf`：测试及随机 IK 示例共用的机器人模型。
- `manual/`：手动诊断程序，按需构建，见 [使用说明](manual/README.md)。
- `VALIDATION.md`：历史本地验证记录，当前结果应以实际运行 CTest 为准。

在仓库根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

示例测试随 `PIN_IK_BUILD_EXAMPLES` 开关启用。
