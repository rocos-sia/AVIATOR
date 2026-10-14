# N5 主张—证据矩阵（N1–N3，含 N4 补充消融）

**整理日期**：2026-10-06
**证据范围**：Node 0、v3 paired matrix、N1、N2、N3；N4 仅作为独立的有限步 margin 补充，不改变 N1–N3 的原始结论。
**边界规则**：本文只主张正则局部连通 component 内的在线延拓、候选生成和安全接受流程；不主张全局 IK 分支保持、连续时间安全不变性或物理部署效果。

## 1. 主张—证据总表

| 主张/对象 | 证据 | 状态 | 允许写法 | 不允许写法 |
|---|---|---|---|---|
| 正则局部 component 内的 differential lift | Node 0 解析契约、32 个 held-out 状态、v3/N2/N3 轨迹 | **有条件支持** | 在满足满秩、局部图和非线性验证条件时提供局部在线延拓 | 对整个 IK 集合保持分支、跨奇异面延拓 |
| 解析任务 Jacobian $A_q,A_x$ | Node 0：解析式与中心差分最大误差分别 $3.84\times10^{-10}$、$4.26\times10^{-10}$；最小采样 $\sigma_{\min}(A_q)=0.087$ | **有条件支持（实现/数值契约）** | 在测试状态和冻结模型上解析 Jacobian 与独立差分一致 | 对所有构型、所有模型的精确性保证 |
| $H+Nr$ 分解 | 满秩假设下的代数推导；reduced/full QP 等价验证 | **有条件支持** | 在 $A_q$ 满行秩且 $N$ 张成零空间时，将任务项与自运动项分解 | H+Nr 必然优于 Pointwise/DLS |
| 三维安全 QP $y=[r_L,r_R,\beta]$ | Node 0 凸性与等价定理；v3、N2、N3 | **有条件支持** | 将线性化关节、限位、clearance 和 task-rate 约束映射为候选动作集合 | 线性 QP 单独提供连续时间安全保证 |
| phase governor | Node 0 的 $\beta=1$、$\beta=0.4$、线性不可行三类探针；v3/N2/N4 的 beta 和进度记录 | **有条件支持** | 近局部不可行时降低任务速率，低于阈值时停止 | 对任意任务必然找到可行速度、总能完成 |
| 独立 nonlinear endpoint/swept checker | N2：每种 filtered 方法 44/46 为 endpoint mismatch，另有 swept 与 joint-limit stop；所有 stop 均有日志 | **有条件支持** | QP 只生成候选，独立 endpoint/swept checker 决定接受或拒绝 | 把 endpoint rejection 当作真实全局不可行或 branch preservation 证据 |
| full/reduced QP 一致性 | Node 0 最大恢复速度差 $3.55\times10^{-15}$；N3/N4 分类和 $\Delta d$ 分布一致 | **有条件支持** | 在测试状态和相同参数下数值一致 | reduced QP 已被证明更快或更省资源 |
| N1 branch-preservation 比较 | 128 初值/臂、局部 atlas、24 个冻结 stress cases；严格候选 0，合法 Pointwise/DLS jump 0 | **负结果** | 明确报告未找到可区分的 branch-jump 对照；保留为 negative control | “防止 branch jumps”“H+Nr 保持全局分支” |
| N2 停止原因 | 92 个 filtered stop rows：每种方法 44 endpoint mismatch、1 swept、1 joint-limit；true local infeasibility=0、unknown=0 | **有条件支持** | 主要停止由有限步线性化乐观与独立 endpoint checker 触发 | 声称大多数停止证明了局部任务集不存在 |
| N3 有限步 clearance 误差 | 53,200 完整 step records；median $|\Delta d|$ 随 $dt$ 减小；描述性 log--log slope 2.087；负尾部保留 | **仅有诊断证据** | 报告测试范围内的有限步敏感性和边界误差分布 | universally second-order、统一 curvature bound |
| N4 curvature-margin 消融（补充） | 135 cells、281,262 rows；0.50 mm 将 optimism 0.526% 降至 0.133%，完成率 48.15% 升至 83.70%，但 $dt=0.01$ 仍 48/90 | **仅有诊断证据** | 报告经验 safety--progress--latency trade-off | 将 margin 写成核心算法或普适安全保证 |
| 物理执行/力控制 | 当前 N1–N4 仿真记录未包含物理部署 | **未证明** | 列为后续验证或工作范围 | 声称已完成 physical validation |
| 全局 IK branch preservation / 全局拓扑完备性 | 没有相应 atlas 覆盖和跨奇异面证据 | **未证明** | 在 Limitations/Discussion 中明确排除 | global branch preservation、globally complete |

## 2. 论文证据链

1. **表示层**：解析 Jacobian、满秩局部图和 $H+Nr$ 分解给出局部 tangent lift。
2. **候选层**：三维 QP 在局部线性模型中生成 $(r_L,r_R,\beta)$ 候选，reduced/full 形式在测试范围内一致。
3. **接受层**：非线性 endpoint/swept checker 对有限步候选做最终判断；N2 说明这一层不是可选的后处理。
4. **监管层**：当候选集合接近边界时 governor 降低 $\beta$，达到停止阈值后安全停止；这支持可解释的 completion--safety trade-off。
5. **分支边界**：N1 没有找到严格 branch jump 对照，因此当前稿件只能声称局部连续延拓和独立监测。

## 3. 必须保留的负结果

- N1 的严格 branch-jump 候选数为 0；descriptor 近失例的变化约为 0.0075 rad，低于冻结的 0.04 rad gap。
- N2 的主要停止不是 QP 在 $\beta_{\mathrm{stop}}$ 下不可行，而是 nonlinear endpoint mismatch。
- N3 的误差负尾部随步长变大，不能用近似二阶斜率替代独立非线性检查。
- reduced QP 与 full QP 在数值上相同，但当前测试没有运行时优势。
- 物理执行尚未被这些仿真结果证明。

## 4. 对正文的直接约束

- Results 使用“supports / is consistent with / in the tested range”，不使用“guarantees”描述全局性质。
- Discussion 明确写出：线性 QP 是 candidate generator，非线性 checker 是 acceptance gate。
- Conclusion 只落在“安全监管的局部在线延拓框架已经得到仿真支持”。
- curvature margin 只能作为 N4 的经验消融，不能改写 differential-lift 定理。
