# aviator_hand 协议文档同步设计

## 目标

以 `nodes/aviator_hand`、`config/inspire_hand.yaml` 和 Core 当前 hand 发布/消费代码为事实来源，更新以下文档中的 hand 相关内容：

- `docs/AVIATOR_ZMQ协议格式说明.md`
- `docs/AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md`

本次只更新文档，不修改线上 JSON、节点行为或配置。

## 更新范围

协议文档完整记录当前 `hand.command` 与 `hand.state` 线格式：

- `aviator_core` 以 50 Hz 发布命令，独立 `aviator_hand` 节点以 10 Hz 发布状态。
- 有效命令支持 `NORMALIZED_POSITION` 和 `GRASP_SETPOINT`；失效命令可省略 `hands`。
- 命令 watchdog 默认 100 ms；反馈有效期默认 300 ms。
- `HandState` 顶层、每侧反馈、诊断计数和 `accepted_command` 按当前代码字段及语义描述。
- 顶层 `sample_mono_us` 是状态生成时刻；每侧 `sample_mono_us` 才是反馈快照时间。
- 明确 `enabled`、`error_code`、`grasp_verified` 和 `STALE` 的当前能力边界。

架构文档同步节点职责、频率、超时和状态摘要，不复制全部字段级规范；字段细节继续由 ZMQ 协议文档负责。

## 兼容与措辞

当前实现定义为现行协议事实，不保留与实现冲突的旧 hand 定义。`JOINT_POSITION` 和需要 `profile_id` 的抓握配置不再描述为当前 `aviator_hand` 能力。手动测试发布器使用 `hand_manual_test`，生产链路仍使用 `aviator_core`。

## 验证

1. 搜索两份文档，确认不再把 `hand.state` 生产者写成 `manipulator` 或 Hand Controller 内部模块。
2. 搜索旧的 100 Hz hand 频率、30/50 ms hand 时限、`profile_id` 必填和旧 `HandState` 示例。
3. 检查两个完整 JSON 示例可解析，并逐字段对照 `hand_node.cpp` 与 `HandControl.cpp`。
4. 运行现有 `aviator_hand` 文档相关单元测试，确认引用的实际格式仍通过。