# RL-N13R Clearance Margin Pilot（2026-10-08）

## 目的

验证增强 clearance warning-layer 惩罚是否减少策略向 clearance 边界漂移。

## 先验检查

当前 `reward.py` 已经实现

\[C_{margin}=\operatorname{clip}((d_{safe}+w_{band}-d_{min})/w_{band},0,1)^2,\]

并在奖励中减去 `w_m C_margin`。本实验因此是权重消融，不是首次注入该项。冻结的警戒线为 `d_safe + margin_width = 5 mm + 5 mm = 10 mm`。

## 协议

- 基线：`w_m=0.1`，来自 fixed50+clip1 同 seed pilot；
- 消融：`w_m=0.5`；
- seed=1，10,000 learner steps / online transitions；
- 50% demo + 50% online，critic gradient clip=1.0，BC warm start；
- 同一 manifold、同一 held-out split、同一 20 个评估轨迹；
- 未修改 `d_safe`、终止规则、QP、安全检查器或历史正式 JSON。

## 结果（checkpoint_10000，20 episodes）

| 指标 | `w_m=0.1` | `w_m=0.5` | 变化 |
|---|---:|---:|---:|
| 完成率 | 0.75 | 0.80 | +5 pp |
| clearance 失败率 | 0.10 | 0.20 | +10 pp |
| speed 失败率 | 0.10 | 0.00 | -10 pp |
| grid 失败率 | 0.05 | 0.00 | -5 pp |
| 最小 clearance 均值 (mm) | 6.564 | 6.587 | +0.023 |
| 进度短缺均值 | 0.162 | 0.110 | -0.051 |

## 结论

增强权重在本轮短 pilot 中可运行，并将完成率提高 5 个百分点，但 clearance 失败率反而提高 10 个百分点。因此当前证据不支持“更强 margin 惩罚已经降低安全失败”的结论，也不足以把 `w_m=0.5` 纳入正式算法。`w_m=0.1` 继续作为现有冻结基线。

这轮结果不能证明奖励 shaping 无效：它只有一个 seed、10k steps 和 20 个 held-out episodes。若要继续，应预注册 `w_m∈{0.1,0.5}`、至少 seeds={1,2}、相同 checkpoint 选择规则，再看 clearance 失败率的置信区间和完整训练曲线。不要根据单个 checkpoint 事后挑选权重。

## 可审计输出

- 训练：`outputs/differential_lift_feasibility_20261008/rl_n13r_margin_pilot_w05/train_seed_1/`
- 原始评估：`outputs/differential_lift_feasibility_20261008/rl_n13r_margin_pilot_w05/train_seed_1/convergence_eval.json`
- 汇总：`outputs/differential_lift_feasibility_20261008/rl_n13r_margin_pilot_w05/margin_pilot_summary.json`
- 代码：`hil-serl/examples/experiments/aviator_manifold/env.py`、`config.py`、`tools/run_rl_n13r_complete_demo.py`
