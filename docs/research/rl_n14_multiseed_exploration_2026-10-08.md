# RL-N14：多 seed 安全探索稳定性验证（2026-10-10）

## 协议

- 从 N13R seed 1/2 的 60k 完整 SAC 状态初始化；每个条件继续 20k steps。
- fixed-reset 与 random-reset（phase fraction 0.2--0.8）各运行 seed 1/2，共四个独立训练条件。
- 固定 64 demo、50/50 demo-online、critic clip=1.0、`w_m=0.1`；未使用 BC 覆盖。
- 每个 checkpoint 评估 50 条同一 val 轨迹，同时做 fixed-reset 与 random-reset 评估。
- 最终 checkpoint 和完整 5k--20k 曲线均保留，不事后挑选。

## 20k 最终 checkpoint 结果

| 训练条件 | 评估 reset | 完成率 | clearance | speed | 平均最小 clearance |
|---|---|---:|---:|---:|---:|
| seed1_fixed_reset | fixed | 0.80 | 0.14 | 0.06 | 7.287 mm |
| seed2_fixed_reset | fixed | 0.98 | 0.00 | 0.02 | 7.662 mm |
| seed1_random_reset | random | 0.94 | 0.00 | 0.06 | 8.037 mm |
| seed2_random_reset | random | 0.86 | 0.02 | 0.12 | 7.943 mm |

## 门槛判断

- 四个训练进程均正常退出，且每个条件均产生 5k、10k、15k、20k checkpoint。
- diagnostics `instrumentation_unknown=0`。
- random-reset seed 1 在 20k random 评估为 94% completion、0% clearance、6% speed。
- random-reset seed 2 在 20k random 评估为 86% completion、2% clearance、12% speed，未达到门槛。
- fixed-reset seed 1 在 20k fixed 评估为 80% completion、14% clearance，未达到门槛。
- 因此 N14 screening **未通过**，不启动 60k continuation，也不进入 N15。

## 解释

随机 phase 探索在 seed 1 上保持了较好性能，但 seed 2 在 10k/15k 出现明显波动后于 20k 部分恢复；fixed-reset seed 1 也出现 clearance 失败增加。当前证据说明“从高成功率 checkpoint 继续训练可运行”，但不支持稳定的多 seed 泛化提升。不得把这些失败解释为 branch jump。

phase 覆盖审计使用同一环境 reset 机制、400 个确定性 reset 样本，结果写入 `phase_coverage.csv`。它验证了 random-reset 的配置覆盖区间；训练本身未记录每次 reset 的独立日志，因此不把该审计误称为实际 replay 中的状态分布。

## 输出

- 汇总：`outputs/differential_lift_feasibility_20261008/rl_n14_multiseed_gate/n14_summary.json`
- completion/termination：`completion_termination_summary.csv`
- phase 覆盖：`phase_coverage.csv`
- 各条件 raw JSON：每个 `seed*_*/n13r_summary.json` 与 `train_seed_*/eval_*_50.json`
- 各条件 diagnostics：`train_seed_*/learner_diagnostics.jsonl`
