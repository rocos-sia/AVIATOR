# GFR-1 B4 60k 零完成率原因诊断（2026-10-13）

## 结论

三个 checkpoint 的 0/10 完成率主要由四个因素叠加造成：

1. 策略在接近 `d_safe=5 mm` 后反复提出无法通过独立 endpoint/swept checker 的动作，环境没有接受动作，也没有任务进度；
2. 100 条成功示范与当前 val 任务分布不一致，示范不能直接覆盖当前任务的初始状态和速度；
3. 60k 训练后期的 critic/Q 分布明显扩张，`checkpoint_60000` 不能当作已收敛模型；
4. 当前评估只证明了局部前段存在可行动作，尚未证明完整 val 轨迹的全程可行性。

因此，当前结果不能归因于“示教轨迹完全错误”，也不能归因于“任务本身一定不可完成”。更准确的判断是：成功示范没有被正确迁移到当前评估分布，策略训练和安全动作选择仍未稳定。

## 直接证据

### 1. 停止是重复拒绝，而不是碰撞穿透

三个 checkpoint 的 10 个 episode 全部以 `governor_exhausted` 结束，平均最小 clearance 都贴近 5 mm：

| checkpoint | completion | progress | min clearance | intervention | endpoint rejection |
|---:|---:|---:|---:|---:|---:|
| 40k | 0/10 | 0.2372 | 5.00006 mm | 162.4 | 161.4 |
| 50k | 0/10 | 0.3099 | 5.00003 mm | 299.7 | 298.7 |
| 60k | 0/10 | 0.2130 | 5.31448 mm | 331.8 | 330.8 |

每个 intervention 几乎都对应一次 endpoint rejection。环境在候选动作被拒绝时保持当前状态，随后继续尝试，直到 governor 的五个速率档都失败。因此 `governor_exhausted` 表示“当前策略连续找不到被 checker 接受的下一步”，不等于“发生了碰撞”，也不等于“全局没有可行路径”。

### 2. 示范是成功的，但与 val 分布不匹配

100 条示范来自 N17 的安装误差任务。它们的源任务初始速度范围为约 `0.03--0.12`，而 val 轨迹初始速度范围约为 `0.015--0.661`。val 中多个轨迹的速度是示范最大速度的 7--20 倍；例如 val trajectory 0 的初始速度约为 `0.348`，trajectory 2 约为 `0.601`。

示范初始任务状态也只是在二维任务空间中接近部分 val 起点，最近距离约为 `0.016--0.126`，并不表示它们具有相同的局部切向动力学。示范中的 100 个 episode 覆盖四类安装误差和三种源速度，但没有覆盖 val 的完整速度和起点分布。

因此，示范提供的是“在另一组任务条件下的安全 phase-rate 参考”，不是当前 val 轨迹的逐点标签。50% demo batch 不能自动解决这种分布外问题。

这也说明当前示范设计不够合理：三档源速度不能称为充分的速度多样化。后续示范应以 val 轨迹的实际速度分布为依据分层采集，而不是继续重复 `0.03/0.08/0.12` 三档。

### 3. 示范与 nominal checker 存在小的后端不一致

`nominal_audit.json` 显示 10,000 个示范 transition 的 nominal endpoint 通过率为 `0.9981`，swept 通过率为 `0.9973`。源模型安装误差带来的 nominal closure residual 最大约为 `2.57e-3`。这说明示范总体可用，但其中少量 transition 并不是当前 nominal GFR 后端的精确同分布样本。

这不是 0/10 的唯一原因，却会让 correction supervision 学到带噪的动作目标，尤其是在边界状态附近。

### 4. 训练后期显示出稳定性恶化

`training_diagnostic_windows.json` 中，预测 Q 的 batch 标准差均值从 0--10k 的 `32.3` 增长到 50--60k 的 `92.5`；temperature 从 `0.108` 增长到 `0.372`；critic loss 在后期仍出现超过 `150` 的尖峰。correction loss 也从约 `0.74` 上升到约 `0.80`。

这说明策略没有在 60k 形成稳定的低干预解，反而在后期对动作价值和边界修正的估计更分散。50k 的平均 progress 最高只能说明它在这 10 个 val 轨迹前段走得更远，不能作为最佳模型或收敛证据。

### 5. 局部可行性尚未被排除

独立 oracle 在相同 checker、`d_safe`、`beta` schedule 和 manifold 上扫描四条 val 轨迹的前 100 步。每一步都发现可接受的 phase-rate 候选，最小 sampled feasible rate 为 `0.175--0.5625`，最快候选常能保持 `beta=1`，clearance 约为 `8--9 mm`。

这排除了“从初始状态开始就没有任何可行动作”这一解释，但只覆盖前 100 步，不能证明完整任务到终点都可行。

## 原因排序

| 优先级 | 原因 | 证据强度 | 对结果的影响 |
|---:|---|---|---|
| 1 | 策略进入 clearance 边界后持续输出不可接受动作 | 直接由 episode trace 支持 | 造成大量 rejection 和 governor exhaustion |
| 2 | demo 与 val 的速度、起点和局部动力学分布不匹配 | 由 source/val 统计支持 | 成功动作难以迁移到 val |
| 3 | critic/Q/temperature 在 40--60k 扩张 | 由 diagnostics 支持 | 60k 出现性能回落和干预增加 |
| 4 | full-horizon feasibility 未验证 | oracle 只覆盖前 100 步 | 仍无法判断后段是否存在可行序列 |

## 下一步

先做同后端、同任务分布的参考轨迹验证：

1. 对当前 val 轨迹生成完整 horizon 的 oracle 或在线 governor 参考轨迹；
2. 按 val 初始速度和任务方向分层重新采集 nominal demos。建议至少覆盖
   `0.015--0.05`、`0.05--0.15`、`0.15--0.30`、`0.30--0.45` 和
   `0.45--0.70` 五个速度区间，并在每个区间保留多个起点和几何条件；
3. 每条示范都必须由同一个 nominal endpoint/swept checker 接受到终点，不能只对旧轨迹做时间缩放，因为速度变化会改变有限步 clearance、beta 降速和 phase-rate 需求；
4. 将 source-task 和 held-out-val 分开报告，不把 source 成功率当作 val 泛化；
5. 在重新训练前冻结一个预先声明的稳定性对照，至少比较 50/50 与 25/75 demo/online 采样，并保留 Q dispersion、temperature、rejection 和 progress 曲线；
6. 只有在 oracle 能完成或明确证明后段不可行时，才继续解释 RL 的完成率。

当前不应通过降低 `d_safe`、放宽 `beta_stop` 或跳过 endpoint/swept checker 来制造完成率。
