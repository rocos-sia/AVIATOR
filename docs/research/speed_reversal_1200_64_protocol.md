# 固定二维表与 RLPD：1200 轨迹、64 环境对照

2026-09-30；分支 `grasp-conditioned`。本次全部使用旧流形 `hil-serl/data/aviator/manifold_phi_stale_pitch-10deg`、已冻结二维表 `outputs/static_field_audit/old_fixed_q_field_00_10.npz`；新 SAC 模型从随机初始化开始（seed=42）。该表曾在旧高速 val/test 的探索中被选择；新建的 `speed_reversal_grid_v3_1200/test` 在最终模型和评测协议冻结之前不打开。

## 数据和硬约束

- 训练 1200 条：任务峰值转角速度 0.6、1.0、1.2、1.5 rad/s × 每条 2、4、6 次换向，每格 100 条；另有 val 每格 10 条、test 每格 100 条。
- 训练、val、test 的任务起点网格互不重叠；每条路径随机任务起点，在线训练还把初始相位随机化到安全区间的 25%–75%。该相位抽样在训练集 1200 个起点的一次独立 MuJoCo 检查中未见接触或净空低于 5 mm，结果保存在 `random_start_audit.json`；这不保证后续每次随机抽样都安全。
- 每条路径在任务位置、速度、加速度和 jerk 上限内。这里的“频次”是**每条路径的换向次数**；由于几何振幅固定，实际每秒换向率与峰值速度耦合。比较时必须在相同速度格内看换向次数，不能把它解释成完全独立的 Hz 轴。
- 两法同一起始构型、关节速度不超过 1.5 rad/s、加速度不超过 10 rad/s²、关节限位内、墙面净空至少 5 mm、无 MuJoCo 穿透接触。固定表相位在网格对齐的任务起点处可让 RL 首点关节角与表首点完全一致。评测检查每个采样点及相邻关节空间中点；这不是连续时间碰撞证明。
- **初始速度边界条件：**现有环境与离线评测从第二次关节位移开始计算二阶差分；第一步默认关节已有匹配轨迹的初始速度。它尚未证明从静止 `qdot0=0` 进入任务的加速度可行性。真机结论必须追加同一起始关节速度和接入段。
- 数据目录 `outputs/speed_reversal_grid_v3_1200`，生成脚本 `hil-serl/tools/generate_speed_reversal_grid.py`。最大时长 15.71 s，训练 `max_traj_length=2000`。

## 先验筛选与现有诊断

- 旧 DP 示范 99 条中，93 条在新加速度上限下不合格；重新检查动力学和 MuJoCo 后，保留 6 条、5992 个转移，存于 `dp_safe_prior.pkl`。
- 固定表直接插值的关节角与 RL 通过 `Q(x, phi)` 实际执行的关节角略有差异。因此从训练集抽取**实际相位动作**，逐段检查相位速率、关节速度/加速度、关节限位、流形分支、MuJoCo 接触和点/中点净空。最终 12 格各保留 25 段，共 300 段、35557 个转移，存于 `static_phase_prior.pkl`。这些是安全局部片段；不能声称整条高速固定表轨迹可执行。
- 两个新训练臂以相同随机种子 42 **重新初始化 actor、critic、温度和优化器**，使用相同 1200 条在线轨迹和同一 6 条安全 DP 基础先验。每个更新批次的在线样本数及基础 DP 样本数相同，另有固定 64 个“额外先验”样本：`dp_extra` 再抽安全 DP，`dp_static` 抽静态相位安全片段。两组每次更新都是 320 个样本；两者都启用关节加速度 10 的硬终止。
- 用 v3 中与 v2 字节一致的 120 条新 val 路径对旧 60k RL 和静态表做共同起点诊断：表 10/120，旧 RL 0/120。表有 110 条、旧 RL 有 120 条加速度超限；表无接触或净空失败，旧 RL 有 27 条净空失败。新训练结果尚未产生。

## 用户运行的最终训练命令

在仓库根目录执行，依次运行两个独立实验。脚本会预检输入、固定 `64` 环境、`1200` 在线轨迹、随机初始相位及加速度硬限制；新运行目录已存在时会拒绝覆盖。

```bash
cd /home/rocos/sia/AVIATOR/hil-serl
bash examples/experiments/aviator_manifold/launch_speed_reversal_rlpd.sh dp_extra --check-only
bash examples/experiments/aviator_manifold/launch_speed_reversal_rlpd.sh dp_static --check-only
bash examples/experiments/aviator_manifold/launch_speed_reversal_rlpd.sh dp_extra
bash examples/experiments/aviator_manifold/launch_speed_reversal_rlpd.sh dp_static
```

日志和 checkpoint 分别落在 `route_a_speed_reversal_1200_64_scratch_steps1m_dp_extra` 与 `route_a_speed_reversal_1200_64_scratch_steps1m_dp_static`。每个训练臂独立运行；每臂累计 **1,000,000 条在线转移**后结束，1200 条训练轨迹每轮打乱后重复采样，失败后重置继续。64 环境满批时共 15,625 个向量步；learner 按每条在线转移一次更新迭代同步推进，两臂预算一致。100 万是本轮设定的训练预算，不代表保证收敛。训练时的 MuJoCo 约束为查表近似；最终结果必须经过独立 MuJoCo 复核。

首次 `scratch_dp_static` 运行采用“一条轨迹只尝试一次”的停止条件，从头训练时大多数回合过早失败，仅产生约 2920 条在线转移并保存 `checkpoint_2919` 后退出。这是训练预算设计不足，不能用于判断 RL 能力。该目录保留；修正后的 `steps1m` 运行仍从随机初始化开始。日志进度显示累计 `transitions`；达到向量步硬上限却未达到在线预算会报错，避免误报训练完成。

## 最终评测顺序

1. 只用 val 挑选每个训练臂的 checkpoint、确认样本数和失败原因，并冻结两个 checkpoint 与固定表。
2. 对 test 的相同 1200 条任务路径，使用同一冻结表初始相位导出当前旧 RL、`dp_extra`、`dp_static` 的确定性轨迹；固定表直接插值。对四种轨迹均用 `tools.audit_static_collision --qddot-max 10` 独立复核。
3. 按速度和换向次数报告完整成功率、配对独有成功数、速度/加速度/净空/碰撞失败及运行时延。重点看 `dp_static` 是否救回 `dp_extra` 独有的失败，而不是只看总体回报。
4. 第二轮专门评价不同冗余姿态起点时，需要为固定表和 RL 共同规划从实测 `q0` 到工作相位的接入轨迹；本轮仅严格比较同一表起点的任务路径。

固定场可作为自动专家给在线训练提供示范或候选动作；只有经过当前状态到候选下一点的动力学与碰撞检查后，才能把它当作安全干预。人工操作者可处理自动专家未覆盖的失效片段，但本协议并未接入真人遥操作，不能把静态示范称作真实 human-in-the-loop 数据。
