# GFR-0/GFR-1 结果记录（2026-10-11）

## 范围

本轮在固定闭链流形、固定 `dt=0.005`、`d_safe=0.005 m`、`beta_stop=0.125` 和相同 nonlinear endpoint/swept checker 下验收 GFR-0，并运行 GFR-1 的短消融预检。没有覆盖 N1--N17 的历史结果。

## GFR-0

通过三组检查：

- 代数 reduced-action contract：`gfr0_algebraic_contract.json`；
- 真实 MuJoCo Node-0：`gfr0_real_contract.json`；
- 64 个真实状态的固定 `beta_g` 二维截面、退化截面和空截面：`gfr0_two_dimensional_contract.json`。

合并结论见 `gfr0_contract_report.json`，状态为 **pass**。这只支持局部微分和动作截面的数值契约，不支持全局 branch preservation 或连续时间安全保证。

## GFR-1 接口

新增独立环境 `hil-serl/examples/experiments/aviator_manifold/gfr_env.py` 和实验配置 `hil-serl/examples/experiments/aviator_gfr/config.py`。Actor 输出二维 `r=[r_L,r_R]`；governor 选择 `beta_g`；结构化模式在固定 `beta_g` 截面中投影，随后由 endpoint/swept checker 决定接受、降速或停止。

B4 的可选执行反馈项为

```text
L_actor = L_RL + lambda_corr * l_corr
```

其中 `lambda_corr=0.1` 仅在本轮 B4 smoke 开启，监督来自下一状态中记录的实际执行 `r`，并只对 intervention 标志有效的样本启用。

## 短消融预检

每个条件使用 seed 11、256 个在线 transition、256 个 learner updates、固定 50% demo/50% online 采样，并保存 checkpoint。B0--B3 使用 CPU smoke；B4 在 GPU smoke 中验证了 correction loss 和完整训练链路。详细诊断在各 `smoke_*` 目录的 `diagnostics.jsonl`。

| 条件 | 含义 | 4-episode completion | 平均 progress | 平均最小 clearance (m) | 平均干预步数 |
|---|---|---:|---:|---:|---:|
| B0 | fixed | 0.000 | 0.0869 | 0.006699 | 150.0 |
| B1 | raw hidden | 0.000 | 0.0363 | 0.005534 | 0.8 |
| B2 | raw + error | 0.000 | 0.0099 | 0.005387 | 0.5 |
| B3 | structured + error | 0.000 | 0.0981 | 0.005000 | 89.8 |
| B4 | structured + correction | 0.000 | 0.1017 | 0.005518 | 92.8 |


这些 4-episode 结果全部是短预检，不能作为 RL 有效性或泛化性结论；B0--B3 与 B4 还没有完成设备匹配的正式多 seed 统计。短预检的作用是确认训练、checkpoint、RLPD 50/50 采样、GFR endpoint backend 和 B4 correction loss 可以运行。

## 当前判断

1. GFR-0 已通过，可以冻结接口继续正式训练。
2. GFR-1 工程链路已打通，但论文级证据尚未完成。
3. 不能根据本轮短预检声称 RL 提高任务完成率、降低总控制周期或提高泛化。
4. 正式下一步是使用同一 GPU、至少 3 个 seeds、相同训练步数和 held-out episodes 运行 B0--B4，并记录 intervention、recovery、endpoint rejection、progress 和 total cycle latency。

## 失败原因诊断（post-reward contract run）

本轮 post-reward B0--B4 都能正常完成 256 transitions、checkpoint 和诊断日志，但 4-episode held-out completion 仍为 0。现有证据按优先级指向：

1. **示范覆盖不足**：现有 demo 是零动作/结构化动作产生的短片段，manifest 中的 demo episode 均以 `governor_exhausted` 结束，progress 约为 0.038 和 0.301；没有完整成功轨迹或恢复轨迹可供 RLPD 学习。
2. **探索覆盖不足**：GFR-1 配置 `random_steps=0`，短 smoke 每个条件只覆盖 3--11 个 episode，且没有安全截面内的显式随机 phase-rate 探索。
3. **任务/执行后端可行性需先验检查**：固定动作和随机策略都在完整轨迹前段停止，说明不能把失败直接归因于网络；必须先用 oracle feasible-center controller 检查同一初始状态是否存在可完成路径。
4. **critic 受终止样本影响**：post-reward 最后诊断仍可见 `target_qs_min≈-100.075`，B2 的 `critic_loss≈116.2`、B4 `≈74.9`；这说明短运行尚未达到稳定学习阶段。

因此当前判断是“成功示范不足 + 安全探索不足 + 任务可行性尚未独立确认”，而不是单一的 RL 泛化失败。post-reward 结果只能证明接口和训练链路已打通。

## 接口与可行性审计补充

- GFR-0 的观测审计确认：可见闭合误差位于 state[18:30]，初始范数为 `6.56e-5`；关闭误差观测时同一切片严格为 0。误差观测没有使用隐藏仿真状态替代可观测输入。
- 每个 GFR candidate 和 governor exhaustion 返回 `endpoint_validation` 与 `swept_validation` 字段，动作日志契约完整。
- `gfr1_transition_raw.jsonl` 已按修正后的字段语义重建，共 333 条记录；所有记录都包含名义/执行动作、`beta_g`、固定持续时间、endpoint/swept validation、checker 是否实际执行、进度和终止信息。修正前的 trace 保留为 `gfr1_transition_raw.pre_eval_flag_fix.jsonl`。
- 运行时单步审计发现并修正了一个 beta 缩放错误：原 `_candidate` 将已经按 `beta` 缩放的任务速度再次传入 `_lift`，导致 `beta<1` 时切向方程残差达到 `1.87e-2`。修正后五个 governor 速度档的预测点/中点残差均小于 `6e-17`；配对短轨迹仍在相同任务前段停止，但闭合残差由 `6.49e-4/1.37e-3` 降至 `8.61e-6/2.23e-5`（fixed 两个样本），结构化模式保持约 `1e-5` 量级。审计文件为 `gfr_gfr01/runtime_audit_20261012T110330Z/beta_pair_summary.json`。
- B4 correction supervision 也增加了资格条件：只有 `beta_executed>0` 且没有 recovery 的已接受动作可作为监督；governor exhaustion 不再使用上一个执行动作作为标签。对应回归测试 `tools/test_gfr_correction_mask.py` 为 `4 passed`。此前 B4 smoke 保留为修复前诊断，不能作为修复后效果证据。
- 在两个验证轨迹上用零 phase-rate、相同冻结 checker 和 governor 做独立预检：`fixed` 在进度 `0.0258/0.0278` 停止，`structured` 在进度 `0.0567/0.0418` 停止；四次均为 `governor_exhausted`/`endpoint_validation`，最小 clearance 约为 `d_safe`。该结果说明当前初始化和零相位动作不足以完成任务，但还不能区分“缺少可行 phase 策略”和“完整任务本身不可行”。预检文件为 `outputs/differential_lift_feasibility_20261011/gfr_gfr01/feasibility_probe/gfr1_feasibility_probe.json`，仅作诊断，不作为 RL 有效性证据。

## 修正后运行器与 B0--B4 预检（2026-10-12）

运行时审计后，使用修正后的 beta 单次缩放、B4 合法监督掩码和重新生成的 demo buffer，串行运行 seed 11 的 B0--B4，每个条件 256 个 online transitions / learner updates，固定 50% demo + 50% online，保存 checkpoint_128 和 checkpoint_256，并写入 diagnostics.jsonl。所有五个条件的 actor 和 learner 均正常退出；这确认运行器、checkpoint 持久化、诊断日志和五组接口配置已经打通。

修正后 checkpoint 与诊断输出位于：
`outputs/differential_lift_feasibility_20261011/gfr_gfr01/post_fix_smoke_B0_B4_runner_v2/`。

在固定 val split 上对每个 checkpoint_256 运行 4 个 deterministic episodes，结果如下：

| 条件 | mode | 可见误差 | completion | 平均 progress | 平均最小 clearance (m) | 平均干预步数 |
|---|---|---:|---:|---:|---:|---:|
| B0 | fixed | 是 | 0.000 | 0.0311 | 0.006563 | 54.75 |
| B1 | raw | 否 | 0.000 | 0.0181 | 0.005532 | 0.00 |
| B2 | raw | 是 | 0.000 | 0.0135 | 0.005444 | 0.25 |
| B3 | structured | 是 | 0.000 | 0.0083 | 0.008280 | 2.00 |
| B4 | structured + correction | 是 | 0.000 | 0.0729 | 0.006475 | 47.25 |

原始汇总：`post_fix_smoke_B0_B4_runner_v2/gfr1_post_fix_smoke_summary.json`；评估表：`post_fix_smoke_B0_B4_runner_v2/gfr1_post_fix_eval_summary.csv`。

这些结果仍是短预检，不构成 RL 有效性、泛化性或完成率优势证据。它们排除了“运行器没有保存 checkpoint/诊断”这一工程原因，但没有排除 demo 覆盖不足、探索不足或任务可行性不足。B4 的 correction loss 现在只使用 `beta_executed>0` 且无 recovery 的已接受动作；在当前短评估中，B4 的 0 completion 不能解释为 correction supervision 已经有效或无效。

因此 GFR-1 仍未通过论文级验证。下一步必须先构造至少一个可完成的 oracle/示范参考，确认完整任务在同一 checker/governor 下存在可行 phase-rate 序列，再进行冻结步数、多 seed 和 held-out 统计。

## 成功示范重建（2026-10-12）

上一轮零/随机 phase demo 已停止使用。基于 N17 的 `online_endpoint_governor`，重新重放并记录了 100 条完整成功 trial，共 10,000 个 accepted transitions。所有源轨迹都满足 `complete=true`、终止为 `end`，并保存了 GFR 所需的 40 维观测、归一化二维 `r` 动作、`beta`、next observation、endpoint/swept 结果和终止标记。

输出位于：
`outputs/differential_lift_feasibility_20261012/gfr_success_demo_100/`。

nominal GFR checker 兼容性审计显示：endpoint step 通过率为 0.9981，swept step 通过率为 0.9973；源模型成功轨迹在 nominal model 下存在少量安装误差导致的闭合残差，最大约 `2.57e-3`。因此这些数据适合作为“成功参考 + 模型扰动观测”的 demo buffer，但在正式训练前仍应报告 nominal compatibility，并保留独立 nonlinear checker 作为最终判定。

该 100:400 配额仍表示轨迹采集比例。训练采样计划保持预先声明的 50/50 warm-up 和 25/75 exploration 两个条件；不能把 100 条成功示范直接等同于 50% 或 20% 的 gradient exposure。

## 成功示范版短训练预检（2026-10-12）

使用上述 100 条成功示范重新串行运行 seed 11 的 B0--B4，每个条件 256 online transitions / learner updates，固定 50/50 demo/online。五个条件均正常退出并保存 checkpoint_128/256 和 diagnostics.jsonl。

| 条件 | completion (4 val episodes) | 平均 progress | 平均最小 clearance (m) | 平均干预步数 |
|---|---:|---:|---:|---:|
| B0 | 0.000 | 0.0311 | 0.006563 | 54.75 |
| B1 | 0.000 | 0.0204 | 0.005639 | 0.25 |
| B2 | 0.000 | 0.0090 | 0.005436 | 0.50 |
| B3 | 0.000 | 0.0192 | 0.005786 | 18.75 |
| B4 | 0.000 | 0.0182 | 0.005786 | 11.00 |

输出：`outputs/differential_lift_feasibility_20261012/gfr_success_demo_smoke_B0_B4_v2/`。

该结果证明成功 demo 已接入训练链路，但 256 update 仍不足以判断收敛或全程完成。B3/B4 的短评估没有形成最终性能结论；正式判断应使用更长训练、多 seed、同分布 source-task evaluation 和 held-out val evaluation。

## B4 60k、16 环境训练（2026-10-13）

成功示范版 B4 已完成 60,000 online transitions，使用 16 个并行环境、100 条成功 demo、50/50 demo/online batch，并正常保存 checkpoint_5000--checkpoint_60000。actor 和 learner 返回码均为 0。

训练诊断显示后半程仍存在明显 critic 分布扩张：预测 Q 的 batch 标准差均值从 32.3（0--10k）升至 92.5（50--60k），temperature 从 0.108 升至 0.372，critic loss 在多个窗口出现 100 以上尖峰。该结果说明“训练过程可运行”，但不能直接把 checkpoint_60000 视为收敛模型。

诊断汇总：
`outputs/differential_lift_feasibility_20261012/gfr_success_demo_B4_60k_16env_seed11/training_diagnostic_windows.json`。

50-episode 单环境评估因 endpoint/swept checker 计算成本过高而中止，尚未产生有效 val 结果。下一步应将评估器改为批量或分批 10 episode，并比较 checkpoint_40000、50000、60000 的 completion、progress、clearance、intervention 和安全停止原因。当前不应依据训练 reward 或 Q 值宣称 RL 已收敛。

## B4 checkpoint 分批 val 评估（2026-10-13）

在固定 val split 上分别运行 checkpoint_40000、50000、60000，每个 checkpoint 10 个 episode：

| checkpoint | completion | 平均 progress | 平均最小 clearance (m) | 平均 intervention 步数 | 平均 endpoint rejection 步数 | 安全停止 |
|---:|---:|---:|---:|---:|---:|---|
| 40000 | 0/10 | 0.2372 | 0.00500006 | 162.4 | 161.4 | governor_exhausted 10/10 |
| 50000 | 0/10 | 0.3099 | 0.00500003 | 299.7 | 298.7 | governor_exhausted 10/10 |
| 60000 | 0/10 | 0.2130 | 0.00531448 | 331.8 | 330.8 | governor_exhausted 10/10 |

汇总文件：
`outputs/differential_lift_feasibility_20261012/gfr_success_demo_B4_60k_16env_seed11/eval_val_10/checkpoint_comparison_summary.csv`。

50k 的前段 progress 最高，但没有完成任务；60k progress 回落且干预更多。因此当前没有证据支持 60k 收敛，也没有证据支持 B4 已提高完整任务成功率。下一步应先检查 source-task 分布和 demo/online 采样策略，再决定是否进行第二个 seed 或调整评估器。
