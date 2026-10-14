# GFR-1 零完成率原因分析（2026-10-12）

## 判断

当前证据首先支持“短训练尚未收敛、示范覆盖不足”，不支持“RL 在该任务上无法完成全程”。全程可行性仍未被证明。

## 证据一：修正后 B0--B4 仍是短预检

修正后的 B0--B4 每个条件只有 256 个 online transitions / learner updates，actor 和 learner 均正常退出并生成 checkpoint_128、checkpoint_256。4 个 val episode 的 completion 均为 0/4，终止原因为 `governor_exhausted`。这能说明训练链路和 checkpoint 逻辑可运行，不能说明 SAC 已经收敛。

修正后评估汇总位于：

`outputs/differential_lift_feasibility_20261011/gfr_gfr01/post_fix_smoke_B0_B4_runner_v2/gfr1_post_fix_eval_summary.csv`

## 证据二：demo buffer 没有完整成功示范

修正后 demo manifest 中，structured-visible 和 raw-hidden 两组的 4 条示范全部以 `governor_exhausted` 结束；structured-visible 的最大任务进度为 0.301，raw-hidden 的最大任务进度为 0.214。没有一条 demo 到达终点，也没有完整恢复轨迹。

因此固定 50% demo / 50% online 的采样协议会让早期 critic 和 actor 反复看到失败边界附近的动作。这个现象可以解释短预检的 0 completion，但不能证明正式 RLPD 在足够训练后一定失败。

## 证据三：前 100 步存在局部可行动作

独立 oracle 探针在相同 manifold、dt、d_safe、beta schedule 和 nonlinear endpoint/swept checker 下，直接扫描物理 `r` 候选，不使用 RL。4 条 val 轨迹的前 100 步均找到至少一个可接受动作：

| trajectory | 扫描步数 | 可行率最小值 | 可行率中位数 | 最后一步 progress | 最快候选 beta | 最后一步 clearance |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 100 | 0.5625 | 1.0000 | 0.0616 | 1.0 | 8.62 mm |
| 1 | 100 | 0.4125 | 0.8313 | 0.0532 | 1.0 | 8.62 mm |
| 2 | 100 | 0.2125 | 0.7688 | 0.0615 | 1.0 | 8.70 mm |
| 3 | 100 | 0.1750 | 0.5625 | 0.0548 | 1.0 | 8.95 mm |

原始结果：
`outputs/differential_lift_feasibility_20261011/gfr_gfr01/feasibility_probe/gfr1_local_oracle_probe_100step_20261012.json`

这项探针只证明前 100 步的局部可行性。它不证明从初始状态到终点存在一条全程连续可行轨迹，也不构成 RL 成功证据。

## 原因排序

1. **短训练未收敛**：256 update 远低于可用于判断 SAC 收敛的训练预算。
2. **示范不可用**：demo 全部是提前停止轨迹，没有完整成功示范或恢复示范。
3. **探索不足**：当前 smoke 使用 `random_steps=0`，策略没有获得系统性的安全截面探索；初期策略动作很快把系统送到 governor exhaustion。
4. **全程可行性尚未验证**：oracle 只覆盖前 100 步。终点附近是否仍有可行动作，需要延长 oracle 或构造成功参考轨迹。

## 对 GFR-1 的结论

当前不能写“RL 已经有效”，也不能写“RL 无法完成任务”。准确表述是：

> The corrected GFR-1 pipeline is operational, but the short run provides no effectiveness evidence. Incomplete demonstrations and insufficient safe exploration prevent a convergence judgment; a local oracle finds accepted phase-rate actions throughout the first 100 validation steps, while full-horizon feasibility remains unverified.

下一步应先生成一条在相同 checker/governor 下到达终点的 oracle 或高质量示范轨迹，再进行冻结训练预算和多 seed B0--B4 对照。不要通过降低 d_safe、放宽 beta_stop 或关闭 checker 来制造成功率。
