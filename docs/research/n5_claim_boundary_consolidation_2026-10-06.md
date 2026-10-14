# N5：N1–N3 证据边界整合

## 节点结论

N1–N3 支持的论文版本是：在正则局部连通 component 内，differential lift 进行局部在线延拓；三维线性 QP 生成候选动作；独立 nonlinear endpoint/swept checker 决定是否接受；接近局部不可行区域时 governor 降速或安全停止。N1 没有找到严格的 Pointwise/DLS branch-jump 对照，因此不加入 branch-superiority 结论。N4 作为补充消融只支持经验性的 curvature-margin trade-off。

## Results 改动摘要

- numerical validation 保留解析 Jacobian、$H+Nr$ 与 full/reduced QP 的测试范围和误差数值。
- N1 改成明确的 negative control：严格候选和合法 branch jump 均为 0。
- N2 明确主要停止是 nonlinear endpoint mismatch，而不是已证明的真实局部不可行。
- N3 明确 slope 2.087 是描述性有限步敏感性，负尾部仍存在。
- 增加 N4 补充段：margin 降低观测 optimism，但以 governor 使用和延迟为代价，未得到统一 margin。

## Discussion 改动摘要

- 将 QP 定位为 candidate generator，将 nonlinear endpoint/swept checker 定位为最终 acceptance gate。
- 解释 completion--safety--latency trade-off。
- 明确 reduced/full 一致性不等于 reduced 更快。
- 明确当前证据没有全局 branch 保证，也没有 branch-jump prevention 证据。
- curvature margin 保留为经验消融和后续校准方向，不进入核心定理。

## 收紧、删除或避免的主张

- `globally branch-preserving`、`guarantees branch preservation`、`prevents branch jumps`：改为 local continuation 或 negative result。
- `always feasible`：改为 governor 在测试范围内 slowdown/stop。
- `formal safety guarantee`、`continuous-time safety guarantee`：改为 finite-step candidate plus independent nonlinear validation。
- `projection-free globally`：改为 nominal loop 中不重复 nonlinear IK projection，保留 drift/recovery 限制。
- `universally second-order`：改为测试范围内 descriptive slope 2.087。
- `reduced QP is faster`：改为 numerical consistency; no runtime advantage observed。

## 尚未验证命题

1. 全局 IK branch preservation 和跨奇异面/拓扑变化的连续延拓。
2. H+Nr 相比 Pointwise/DLS 的普遍 branch-survival 优势。
3. 线性 clearance QP 的连续时间不变性或模型不确定性下的形式安全保证。
4. 跨任务、跨步长和跨场景的统一 curvature margin。
5. LUT 重建时间、存储量和在线成本的完整公平比较。
6. 真实机器人物理执行、接触力和电机电流约束。

## 下一节点建议

先进行 manuscript audit 和独立编译检查；若正文边界通过，再决定是否开展新的任务段搜索或把物理执行作为后续节点。不得在没有新的严格 branch-jump 证据前恢复 branch-superiority 表述。

## Frozen JSON integrity check

The following SHA-256 values were recorded after N5 editing and match the pre-N5 snapshot:

| file | SHA-256 |
|---|---|
| `outputs/differential_lift_feasibility_20261004/v3/paired_matrix_yqp_90trials.json` | `8cedddef89518b35393890d3e18d24bfbb1ba24f211602e42d822534f223b45f` |
| `outputs/differential_lift_feasibility_20261005/n1_branch_search/n1_strict_summary.json` | `f7211234b972bfb880114176d544fac479ced380a5055c0f2d0e57a22b9cff0e` |
| `outputs/differential_lift_feasibility_20261005/n2_endpoint_analysis/n2_stop_classification_summary.json` | `71bf79ebef7d6cfe45ec528ef168d22f21c2f7745be09354ef8e5fe66d2bb6bc` |
| `outputs/differential_lift_feasibility_20261005/n3_dt_analysis/n3_dt_sensitivity_summary.json` | `2236ca266ce3d86d57ab54b0d93b6524c8ccfbb73975452d71190186d5a0d7de` |

## Verification

- LaTeX environment counts: 97 `\\begin` and 97 `\\end`, with matching counters.
- Manuscript and matrix contain no control characters; `git diff --check` passes.
- Existing AVIATOR manifold/safety tests: 31 passed.
- `pdflatex`, `latexmk`, `tectonic`, `lualatex`, and `xelatex` are unavailable in the current environment, so PDF compilation remains the next audit-gate task.
