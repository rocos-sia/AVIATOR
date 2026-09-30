# 固定二维关节角表与 RLPD：旧流形验证

> 本文是初步 LUT 代理检查。2026-09-30 已完成实际 MuJoCo 碰撞检查和严格速度扫描，结论以 [后续报告](static_q_vs_rl_speed_sweep_2026-09-30.md) 为准。

日期：2026-09-29；分支：`grasp-conditioned`；工作树中已有其他未跟踪文件，未改动。

## 实验对象

- 旧流形：`hil-serl/data/aviator/manifold_phi_stale_pitch-10deg/`，生成于 2026-09-23；当前 `manifold_phi/` 于 2026-09-28 重建，与旧 checkpoint 不匹配。
- RLPD：`route_a_lut_native_rlpd_800x64/checkpoint_60000`，确定性动作，无安全投影。
- 固定表：只在 `dp_train` 100 条轨迹上搜索两臂安全相位区间的常数比例，选出 `(0.9, 0.1)`，随后冻结 101×65×14 的 `q(theta,s)` 表；在线只对 `q` 做二维双线性插值。选择后没有使用 val/test 轨迹调整表。
- 初步通过标准：每个采样点 `d_min >= 5 mm`、关节在限位内、每 10 ms 的相邻关节差分速度不超过 `1.5 rad/s`，且没有无效/损坏的流形分支。此处未执行加速度硬检查；后续核对发现项目 URDF 和 C++ 检查器使用 `10 rad/s²` 上限。

| 数据 | 固定二维表 | RLPD 60k |
|---|---:|---:|
| val，原速，50 条 | 47/50 | 49/50 |
| test，原速，100 条 | 98/100 | 95/100 |
| test，同路径两倍速，100 条 | 58/100 | 50/100 |

固定表在原速 test 的失败包括 1 条净空不足和 2 条关节速度超限，其中至少一条重叠。两倍速有 42 条关节速度超限。RLPD 原速 test 有 4 条净空失败和 1 条速度失败；两倍速有 19 条净空失败、26 条速度失败、5 条相位网格退出。

## 解释与局限

这组证据表明：**现有轨迹范围内，时序 RL 还没有显示出超越固定二维关节角表的必要性。** 这是一个强基线，但还不是“RL 无用”的证明。固定表的 val 为 47/50，低于 RL 的 49/50；不同轨迹分布与初始状态会影响对比。

固定表是实际保存的 `q` 网格。其碰撞净空借用了 `Q(theta,s,phi)` 在相应相位的存储值，而不是对插值后 `q` 重新执行 MuJoCo 碰撞检测。test 上插值 `q` 与该碰撞代理的对应 `Q` 最大相差 0.0256 rad，因此 **98/100 只是 LUT 代理下的结果，不是已认证的真实碰撞成功率**。仅 6547/6565 个表节点满足代理的相位、分支和净空条件；目前也不能声称整个任务矩形都有可行固定构型。

固定表直接从其表内关节角启动；RLPD 使用环境默认初始相位。test 集上两者起始构型的最大单关节差异中位数为 0.125 rad，最大为 0.324 rad。真正在线切换到固定表前必须规划这一段接入运动。两倍速实验保留 10 ms 控制周期并每隔一个原始采样点取一次，因此相同路径运行约两倍快；21/100 条的任务速度超过轨迹生成器原定上限，属于压力测试，不属于原定工作包线。

当前 `manifold_phi/` 与旧 checkpoint 混用时，RLPD 60k 在 val 为 0/50；该结果不能用于固定表与 RL 的方法比较。旧流形重跑后得到 val 49/50，与仓库历史报告相符。

## 复现

在 `hil-serl/` 下使用 `/home/rocos/miniconda3/envs/serl_clean/bin/python`：

```bash
python -m tools.audit_static_field --manifold-dir data/aviator/manifold_phi_stale_pitch-10deg --split val --fraction 0.9 0.1 --field-output ../outputs/static_field_audit/old_fixed_q_field_09_01.npz
python -m tools.audit_static_field --manifold-dir data/aviator/manifold_phi_stale_pitch-10deg --split test --field-input ../outputs/static_field_audit/old_fixed_q_field_09_01.npz
python -m tools.audit_static_field --manifold-dir data/aviator/manifold_phi_stale_pitch-10deg --split test --stride 2 --field-input ../outputs/static_field_audit/old_fixed_q_field_09_01.npz
python -m tools.eval_rlpd_policy --manifold-dir data/aviator/manifold_phi_stale_pitch-10deg --split test --n-eps 100 --ckpt-step 60000 --sac-checkpoint examples/experiments/aviator_manifold/route_a_lut_native_rlpd_800x64
```

明细 JSON 和冻结表位于 `outputs/static_field_audit/`。两倍速 RLPD 输入轨迹位于该目录的 `fast2/trajs/test/`，由 test 原轨迹隔点采样并按二倍速度重算导数。

后续报告已对冻结表插值后的 `q` 逐点及点间中点做独立碰撞检查。初始姿态到表内首点的可达连接仍未验证。
