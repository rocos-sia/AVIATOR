# GFR-1 不收敛根因分析（2026-10-13）

## 结论

本轮失败首先是示教语义与训练环境不一致，其次才是 SAC 后期的高方差。证据不支持“只是示教数量不够”或“只要增加探索就会收敛”。

## 观测结果

- 20k 有效评估：训练 split 0/10，平均 progress 0.1739，平均 endpoint rejection 108.6。
- 40k 有效评估：训练 split 0/10，平均 progress 0.1884，平均 endpoint rejection 232.0。
- 单条 val 轨迹跟踪：20k 在 progress=0.3776 时 governor exhausted；40k 在 progress=0.3967 时 governor exhausted。两次都在 clearance 约 5 mm 边界附近失败。
- 训练诊断中 Q 标准差从约 32 增加到 49--65，temperature 从约 0.116 增加到 0.23，critic loss 偶发尖峰（最高约 113），但没有 NaN 或完全发散。

## 根因 1：示教是短窗口，环境是长时域任务

`gfr1_make_success_demo.py` 将每条 100 步的成功窗口最后一步标成 terminal，并按窗口内 progress 增量构造奖励。示教中每步 progress 约为 0.01，非终止 reward 中位数约为 0.099，100 步后获得约 100 的 terminal reward。

实际 GFR 环境在 `dt=0.005`、轨迹采样间隔 `0.01` 下，每步只推进约半个轨迹索引；一个 val 任务需要约 900--1200 步。也就是说，demo 告诉 critic“100 步后任务结束”，环境实际要求“约 1800 步后任务结束”。这是明显的 horizon/reward-to-go mismatch，会产生过高 Q 值并不能教会策略长程提前避开边界。

## 根因 2：示教模型与训练模型不一致

100 条示教来自 10 个模型变体，其中约 65% 来自 base/fixture 安装误差模型；训练和评估只使用 `aviator_n16_base_rotation_0.xml`。重新用 nominal 模型检查 demo 状态时，闭合误差的 P95 约为 `5.2e-4`，最大约 `1.29e-3`，部分样本并不是训练环境中的同一可行状态。示教动作因此不是 nominal 环境的逐点标签。

## 根因 3：策略学到了边界附近动作，安全后端才终止

20k 和 40k 的单轨迹输出都能在早期通过检查，但在 progress 约 0.38--0.40 时反复触碰 `d_safe=0.005 m`：20k 在一次线性截面/endpoint rejection 后耗尽 governor，40k 在 endpoint validation 失败后耗尽 governor。说明问题不是起点完全不可行，而是策略没有提前把 phase 调到安全内侧。

## 次要因素：critic/temperature 方差上升

50/50 demo/online 比例本身保持正确，demo buffer 也有 10,000 条 transition。但 online 轨迹几乎全是失败，回放中长期缺少“成功通过边界后的长程后续”。同时短 demo 的高 terminal return 与在线的 `-100` failure return 混在一起，导致 Q 分布变宽、temperature 上升，进一步放大了边界动作不稳定。

## 评估流程问题

原异步 evaluator 少了 `hil-serl/examples` 的 Python 路径，第一次 20k/40k 结果是 instrumentation failure，已丢弃。修复后训练 split 的 20k/40k 均为 0/10。评估脚本默认读取 `rl_train`，未显式设置 `AVIATOR_GFR_SPLIT=val`；因此此前的 20k/40k 汇总不能当作 held-out 泛化证据。补跑的 20k val 评估同样为 0/10，平均 progress 0.2370。

## 论文解释边界

本轮只能说明当前 demo 构造和长时域训练接口没有形成可执行策略。不能据此说 RL 算法本身不能收敛，也不能把失败归因于泛化性已经被证明不足。

## 下一次最小验证顺序

1. 用同一 nominal 模型采集 demo。
2. 让 demo 沿完整 val 轨迹生成，保持真实 progress 时间尺度；不能把 100 步窗口末尾标为全任务 terminal。
3. 在训练前做一条 teacher replay sanity check：把 demo 动作逐步送入 GFR 环境，确认长轨迹能够完成且 endpoint/swept 通过。
4. 重新训练时固定 50/50 比例，20k 和 40k 显式使用 val split 独立评估。
5. 只有 teacher replay 和 20k/40k 评估接口都通过后，才讨论 SAC 的探索比例或 critic 正则化。
