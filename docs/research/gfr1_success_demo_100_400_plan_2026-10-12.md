# GFR-1 成功示范与在线探索配额（2026-10-12）

## 目标

用已有 `online_endpoint_governor` 的成功执行作为 demonstration anchor，再让 RL 在同一个固定流形和安全后端上探索。上一轮 GFR demo 是由零/随机 phase 动作生成的提前停止片段，不能继续作为示范。

## 数据配额

- `demo_buffer`：100 条完整成功轨迹。每条轨迹必须到达任务终点，所有接受步通过 endpoint 和 swept checker。
- `online_replay_buffer`：400 个在线 episode。允许包含成功、governor 降速、恢复和安全停止，用于表示探索分布和失败边界。
- 计数以 episode 为主，同时报告 transition 数量。不能用较长的失败轨迹人为放大探索比例。

现有可复用证据：N17 的 `online_endpoint_governor` 有 48/48 完整 trial、4,800 个接受步；N16 有 475/480 完整 trial。现有 JSON 主要保存几何诊断和调度信息，正式 GFR demo 仍需补齐每步的 GFR 观测、二维 `r`、`beta_g`、next observation 和动作执行字段。

## 训练采样比例

采集比例和 batch 比例分开冻结：

1. warm-up：前 10k learner updates 使用 50% demo + 50% online，防止策略在成功轨迹尚未学会前漂移；
2. exploration：后续使用 25% demo + 75% online，让 400 条在线 episode 主导探索；
3. 论文中保留固定 50/50 和固定 25/75 两个采样比例作为预先声明的消融，不根据结果挑选唯一配置。

demo buffer 只放完整成功轨迹的 accepted transitions。安全停止和失败 transition 保留在 online buffer；如果要学习恢复，应作为单独 recovery 集记录，不能伪装成成功示范。

## 轨迹转换验收

每条 demo transition 必须保存：

- `observations.state` 和 `next_observations.state`；
- `actions=[r_L,r_R]`；
- `beta_g`、`beta_executed`、`tau_requested`、`tau_executed`；
- endpoint/swept validation；
- closure residual、clearance、task progress；
- `masks` 和 `dones`。

转换后必须检查：

- demo completion rate = 100%；
- 所有 demo 的终止为 `end`；
- endpoint/swept 通过率为 100%；
- 没有 `governor_exhausted` 被标记为成功；
- demo 与 GFR 当前 observation/action space 的 shape 一致。

## 证据边界

成功示范可以验证“在线非强化控制器提供可学习的可行参考”，但不能单独证明 RL 泛化或 RL 优于在线 governor。400 条在线 episode 的作用是检验策略在成功参考附近的探索、干预成本和 held-out 执行质量。
