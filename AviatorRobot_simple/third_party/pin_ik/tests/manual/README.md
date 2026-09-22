# 手动诊断程序

这些程序保留早期开发时的检查用途，不加入 CTest，也不默认构建。

- `simple_test.cpp`：打印模型、限位和一次 IK 调用结果。它允许 IK 失败，因此退出成功不代表 IK 求解通过。
- `test_debug.cpp`：打印包含连续关节的模型维度和限位维度。

在仓库根目录执行（配置时需开启 `BUILD_TESTING`，默认开启）：

```bash
cmake --build build --target pin_ik_smoke pin_ik_model_debug -j2
./build/tests/manual/pin_ik_smoke
./build/tests/manual/pin_ik_model_debug
```

自动化回归测试请使用 `ctest --test-dir build --output-on-failure`。
