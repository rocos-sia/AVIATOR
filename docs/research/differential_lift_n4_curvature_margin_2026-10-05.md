# N4：有限步 curvature-margin 消融

**节点**：N4  
**日期**：2026-10-05  
**状态**：完成

## 1. 实验协议

N4 复用 N3 的 27 个场景--速度 cell、随机种子、初始状态生成、任务总时间和数值预算，只在**线性候选约束**中增加

\[
d_{\mathrm{linear}}\ge d_{\mathrm{safe}}+m_{\mathrm{curv}}.
\]

独立 nonlinear endpoint/swept checker 始终使用原始 `d_safe=5 mm`，没有把 margin 混入最终安全判定。预先冻结的 margin 为

\[
m_{\mathrm{curv}}\in\{0,0.05,0.10,0.25,0.50\}\ \mathrm{mm}.
\]

其余参数保持不变：`beta_stop=0.125`、QP 权重、closure tolerance、swept subdivisions、full/reduced QP 结构和 v3/N1/N2/N3 原始 JSON。

## 2. 数据完整性

- 135 个 cell：5 个 margin × 3 个 `dt` × 3 个场景 × 3 个速度。
- 2,700 个 filtered/full trial-row。
- 281,262 条逐步记录。
- `instrumentation_unknown=0`。
- 所有有限步候选都有 `d_linear`、`d_actual`、`Delta d` 和 margin 字段。
- margin=0 的 N4 trace 与 N3 trace 的 53,200 个共同记录逐字段一致（关键 clearance/beta 字段差异小于 `1e-13`）。

## 3. 主要结果

### 3.1 汇总结果

| `m_curv` | optimism rate | endpoint/swept failure | 完成率 | 中位任务进度 | 中位步延迟 |
|---:|---:|---:|---:|---:|---:|
| 0 mm | 0.526% | 0.515% / 0.526% | 48.15% | 0.9746 | 72.0 ms |
| 0.05 mm | 0.491% | 0.491% / 0.491% | 50.00% | 0.9917 | 81.4 ms |
| 0.10 mm | 0.491% | 0.491% / 0.491% | 50.00% | 0.9917 | 82.8 ms |
| 0.25 mm | 0.288% | 0.288% / 0.288% | 67.41% | 0.9998 | 80.2 ms |
| 0.50 mm | 0.133% | 0.133% / 0.133% | 83.70% | 0.9999 | 79.1 ms |

`full_joint_qp` 的结果与 `lift_filtered` 一致。所有 margin 下都没有 clearance-only conservative rejection；每种方法始终有 3 个相同的极小 joint-limit post-check rejection。

### 3.2 对不同步长的稳定性

对 `lift_filtered`，完成 trial 数如下：

| `m_curv` | `dt=0.0025` | `dt=0.005` | `dt=0.01` |
|---:|---:|---:|---:|
| 0 mm | 44/90 | 44/90 | 42/90 |
| 0.05 mm | 45/90 | 45/90 | 45/90 |
| 0.10 mm | 45/90 | 45/90 | 45/90 |
| 0.25 mm | 89/90 | 48/90 | 45/90 |
| 0.50 mm | 89/90 | 89/90 | 48/90 |

0.50 mm 在较小步长下明显降低了边界停止，但在 `dt=0.01` 仍有 41 个 endpoint-validation 停止，因此没有一个冻结 margin 在全部步长和场景中消除 finite-step optimism。

### 3.3 Governor 代价

margin 从 0 增加到 0.50 mm 后，`beta<1` 的逐步比例约从 7.7% 增至 13.9%，lift 的中位步延迟从约 72.0 ms 增至约 79.1 ms。margin 的主要作用是让 governor 提前缩小任务速度，而不是改变非线性 checker 的阈值。

### 3.4 reduced/full QP

每个 margin 和 `dt` 下，reduced/full 的记录数、停止数、optimism rate 和 `Delta d` 分布一致；中位误差差异在 `10^{-15}` m 量级。N4 支持数值一致性，不支持当前实现下 reduced QP 更快。

## 4. 对 curvature margin 的判断

N4 证明了一个有限范围内的经验现象：预留线性 clearance margin 可以降低线性化 optimism，并提高完成率和任务进度；代价是更多 governor 调速和更高计算延迟。

但 N4 没有证明存在跨场景、跨步长的统一安全 margin。特别是 `m_curv=0.50 mm` 在 `dt=0.01` 仍保留 endpoint-validation 失败。因此：

> 当前证据不足以把 curvature margin 纳入核心算法或写成普适理论保证；独立 nonlinear endpoint/swept checker 仍是必要机制。

如果后续继续研究 margin，应将其作为显式的速度--安全权衡或校准实验，而不是修改 differential lift 核心公式。

## 5. 论文修改建议

1. Results 中增加 margin 消融表，报告 optimism、完成率、任务进度和延迟。
2. Discussion 中说明 margin 能降低有限步乐观性，但没有得到统一的 `m_curv` 保证。
3. 保留“线性 QP 生成候选，独立 nonlinear checker 决定接受”的两层安全表述。
4. 不把 margin 消融解释为 branch preservation 或 branch-jump 证据。
5. 不声称 reduced QP 比 full QP 更快。
6. 若论文篇幅有限，可把完整 margin 曲线放入补充材料，正文只报告 `0`、`0.25` 和 `0.50 mm` 三个预先冻结点。

## 6. 输出文件

新增或修改的代码：

- `hil-serl/tools/validate_paired_baselines.py`：增加 `--clearance-margin-m`，仅改变线性候选约束，并记录 margin 字段。
- `hil-serl/tools/validate_branch_connector.py`：将 margin 传入 reduced action-space QP，非线性 checker 仍使用原始 `d_safe`。
- `hil-serl/tools/run_n4_curvature_margin.py`：复用冻结矩阵运行 135 个 cell。
- `hil-serl/tools/summarize_n4_curvature_margin.py`：生成逐步、分类、分 margin、reduced/full 和图表统计。

- Cell 原始 JSON 与 manifest：`outputs/differential_lift_feasibility_20261005/n4_curvature_margin/`
- 原始合并 trace：`outputs/differential_lift_feasibility_20261005/n4_curvature_margin_analysis/n4_step_trace_raw.json`
- 汇总 JSON：`n4_curvature_margin_summary.json`
- 每步 CSV：`n4_clearance_margin_steps.csv`
- 分类统计：`n4_classification_summary.csv`
- margin 汇总：`n4_margin_summary.csv`
- reduced/full 对照：`n4_reduced_full_comparison.csv`
- 图：`n4_margin_optimism_progress_stop_figures.png`
