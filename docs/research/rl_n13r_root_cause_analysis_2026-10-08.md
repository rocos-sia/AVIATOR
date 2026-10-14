# RL-N13R 50k 检查点 Root Cause Analysis

日期：2026-10-08

## 结论

旧 Seed 0 在 50k 的失败已经可以明确归因为 **clearance 主导的策略性能坍塌**：20 条 held-out 轨迹中，15 条（75%）以 clearance 失败，1 条（5%）以 speed 失败；grid/phase boundary、branch 和 joint-limit 均为 0。50k 时 completion 为 20%，平均 progress shortfall 从 40k 的 0.0495 增至 0.5413，mean reserve 从 0.6546 降至 0.4299，说明失败发生在任务推进和 clearance 余量同时恶化的阶段。

旧 Seed 0 的 learner.log 没有逐更新的 critic loss、Q、actor loss、entropy 或 temperature 数值。因此，**不能把旧 50k 事件事后写成 Q explosion、entropy collapse 或 alpha oscillation**。这部分证据缺失，而不是“没有发生”的证据。

修复版 Seed 1/2 使用固定 50% demo + 50% online（每个 batch 为 128+128），critic global-norm clipping=1.0，并记录逐 100 update 诊断。两个 seed 的 Q 均值在 40k--50k 期间平滑上升并跟随 target Q，entropy 约在 -1 附近，alpha 没有尖峰或归零；Seed 1 在 50k 保持 90% completion，Seed 2 在 50k 降至 65% 后于 60k 恢复到 95%。因此，固定 demo 比例和 clipping 改善了可观测性并降低了 Seed 0 式灾难性坍塌的严重程度，但尚未证明它们是唯一根因。

## 1. 旧 Seed 0 的直接失败归因

| checkpoint | completion | clearance | speed | grid/phase | branch | joint-limit |
|---|---:|---:|---:|---:|---:|---:|
| 40k | 95% | 0% | 5% | 0% | 0% | 0% |
| 50k | 20% | **75%** | 5% | 0% | 0% | 0% |
| 60k | 95% | 0% | 5% | 0% | 0% | 0% |

50k 并非持续性失败：60k 恢复到 95%，所以它更像是随机策略分布在 clearance 边界附近的暂态漂移，而不是持久的模型结构失效。

## 2. 40k--50k 训练诊断

修复版逐更新诊断见：

- `outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/train_seed_1/learner_diagnostics.jsonl`
- `outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/train_seed_2/learner_diagnostics.jsonl`
- `outputs/differential_lift_feasibility_20261008/rl_n13r_repaired_fixed50_clip1/root_cause_metric_windows.csv`

### Seed 1

- 40k--50k：Q mean 222.6--269.4，Q std 12.8--34.0，target Q 与 predicted Q 同步；
- entropy 均值约 -1.00，alpha 均值约 0.195，范围 0.182--0.210；
- critic loss 有瞬时尖峰，但没有伴随 Q 均值暴涨、entropy 归零或 alpha 发散；
- held-out completion：40k 85%、50k 90%、60k 90%，没有 50k clearance 坍塌。

### Seed 2

- 40k--50k：Q mean 317.5--374.0，Q std 6.5--30.5，target Q 与 predicted Q 同步；
- entropy 均值约 -0.985，alpha 均值约 0.128，范围 0.118--0.137；
- critic loss 的尖峰比 Seed 1 更明显，但没有 Q 或温度失稳；
- held-out completion：40k 90%、50k 65%、60k 95%；50k 的 30% 失败为 clearance，5% 为 speed，0% 为 phase/grid。

Seed 2 表明固定 50/50 和 clipping 不能保证每个 seed 在 50k 单调稳定，但它把失败限制在同一 clearance 机制且能在 60k 恢复。

## 3. Demo buffer 与正则化判断

旧 Seed 0 的采样不是固定 50/50。启用 extra demo buffer 后，在 online buffer 超过 1500 后，每个 batch 实际为：

- online：230/320（71.875%）；
- regular demo：26/320（8.125%）；
- extra demo：64/320（20%）；
- 合计 demo：90/320（28.125%）。

因此旧实现是一个分段 prior schedule：从早期 50% demo 降到后期 28.125% 总 demo，而不是随 online 样本无限连续稀释。修复版改为恒定 128 online + 128 demo，且不使用 extra demo buffer。两个 repaired seed 的诊断中 demo fraction 始终只有 `0.5` 一个值。

本轮 clipping 只加在 critic optimizer，norm=1.0。由于没有单独的“无 clipping、固定 50/50”对照，不能把改善唯一归因于 clipping；同样不能把 Seed 2 的中等 50k 下降归因于 clipping 失败。

## 4. 最可信的整体原因

当前证据支持的因果层级是：

1. **直接触发机制：** 策略在某些 held-out 任务段产生了 clearance 不足的动作分布，触发 SDF/clearance 终止；
2. **放大因素：** 低 clearance 任务几何、探索噪声和随机 seed 共同造成策略分布漂移；
3. **可能但未证实的训练因素：** critic loss 瞬时尖峰或 prior schedule 改变；旧 Seed 0 没有足够日志，无法确定它们是否先于失败发生；
4. **排除项：** phase/grid boundary 越界、branch descriptor 跳变和 joint-limit 不是 50k 失败的主要原因。

因此目前最准确的表述是：

> 50k 事件表现为 clearance 边界附近的随机策略漂移；固定 demo 比例、critic clipping 和显式退出握手提高了训练稳定性与可诊断性，但尚未构成对单一训练根因的因果证明。

## 5. 对论文和下一步的影响

不要在论文中写“50k 崩溃由 Q explosion 导致”，也不要写“固定 50/50 已证明解决了崩溃”。可以写：

- 旧运行的失败以 clearance 为主，phase boundary 未参与；
- 修复版保持固定 demo prior、记录训练诊断，并在 60k 恢复到 90--95% held-out completion；
- 训练曲线仍存在 seed-dependent 的非单调性，所以 RL 结果应作为训练稳定性和安全监管证据，而不是普适收敛保证。

若要真正区分 buffer 与 critic 的因果作用，下一实验必须是同 seed、同 held-out 集合的 2×2 消融：

| demo prior | critic clipping |
|---|---|
| historical schedule | off |
| historical schedule | on |
| fixed 50/50 | off |
| fixed 50/50 | on |

每格至少重复相同的 seeds，并保留本轮 JSONL 诊断。当前不应继续凭单次曲线修改理论。
