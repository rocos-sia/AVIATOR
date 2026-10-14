# RL-N14：高成功率 checkpoint 初始化与安全探索 pilot（2026-10-08）

## 目的

检验是否可以从 seed 2 的 60k 高成功率完整 SAC 状态继续训练，并通过安全可行初始 phase 随机化扩大探索。

## 冻结条件

- 初始化：`rl_n13r_repaired_fixed50_clip1/train_seed_2` 的 checkpoint 60000；
- 继续训练：10,000 learner steps / online transitions；
- 64 条完整 demo，固定 50% demo + 50% online；
- critic global clipping=1.0；`w_m=0.1`；
- 初始 phase 随机化：每个臂在安全区间的 0.2--0.8 分位之间采样；
- held-out：相同 20 条验证轨迹；
- 原 N13/N13R JSON 未覆盖。

## 结果（checkpoint 10000，20 episodes）

| 条件 | 完成率 | clearance | speed | 平均最小 clearance |
|---|---:|---:|---:|---:|
| 原 seed 2 checkpoint 60000 | 0.95 | 0.00 | 0.05 | 7.314 mm |
| N14 随机初始 phase | 0.95 | 0.00 | 0.05 | 7.976 mm |
| N14 固定初始 phase 评估 | 1.00 | 0.00 | 0.00 | 7.385 mm |

## 判断

1. 完整 SAC checkpoint 可以作为 N14 的连续初始化，训练进程正常退出并产生 5k/10k checkpoint。
2. 随机初始 phase 的 N14 策略在相同 held-out 集合上保持 95% 完成率和 0% clearance 失败，说明没有立即破坏原有安全表现。
3. 固定初始 phase 评估达到 100%，但这只是 20 条轨迹的短期结果，不能据此声称泛化提升。
4. 当前证据支持“继续探索是可行的”，尚不足以证明“随机初始 phase 带来因果收益”。

## 下一步

完成预注册的 seed 1/2 多 seed N14 gate，并同时报告随机初始 phase 与固定初始 phase。只有两个 seed 都保持完成率、clearance 和 speed 指标不劣于 60k 基线，才进入 N15 的更大探索或任务几何泛化实验。

## 输出

- 训练：`outputs/differential_lift_feasibility_20261008/rl_n14_reset_exploration_from_seed2_60k_retry/train_seed_2/`
- 随机评估：`outputs/differential_lift_feasibility_20261008/rl_n14_reset_exploration_from_seed2_60k_retry/train_seed_2/convergence_eval_random_reset.json`
- 固定评估：`outputs/differential_lift_feasibility_20261008/rl_n14_reset_exploration_from_seed2_60k_retry/train_seed_2/convergence_eval_fixed_reset.json`
- 汇总：`outputs/differential_lift_feasibility_20261008/rl_n14_reset_exploration_from_seed2_60k_retry/n14_exploration_summary.json`
