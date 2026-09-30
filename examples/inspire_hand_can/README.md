# inspire_hand_can（兼容入口）

因时手节点已迁移到 [nodes/aviator_hand](../../nodes/aviator_hand/README.md)，主工程生成
`build/bin/aviator_hand`。CAN 驱动、反馈采集、命令守卫、发布器和测试仅维护在新目录；
本目录的 C++/Python 文件转接同一份实现，保留旧调用方式。

推荐从仓库根目录运行：

```bash
cmake -S . -B build
cmake --build build --target aviator_hand --parallel 2
./build/bin/aviator_hand --config config/inspire_hand.yaml
# 另开终端，使用已安装 pyzmq 的 Python：
python nodes/aviator_hand/hand_command.py
```

原有独立构建仍可使用：

```bash
cmake -S examples/inspire_hand_can -B examples/inspire_hand_can/build
cmake --build examples/inspire_hand_can/build --parallel 2
./examples/inspire_hand_can/build/inspire_hand_node --config config/inspire_hand.yaml
ctest --test-dir examples/inspire_hand_can/build --output-on-failure
```

旧独立程序不传 `--config` 时，仍读取可执行文件旁的 `config/inspire_hand.yaml`，
该文件由本目录的 `config/inspire_hand.yaml` 复制生成；推荐显式使用仓库根目录配置，避免改错文件。
旧的 `python examples/inspire_hand_can/hand_command.py` 也继续可用。

新旧程序不能同时控制同一套 CAN 设备。完整启动顺序、手动指令、反馈字段、只读检查、
会话绑定和故障排查见 [新节点 README](../../nodes/aviator_hand/README.md)。
