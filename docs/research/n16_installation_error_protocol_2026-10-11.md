# N16：安装误差下的在线方案与冻结查表对比

## 目的

本节点检验工件/夹具和双臂基座安装误差对安全、任务进度、完成率和控制延迟的影响。它不检验 RL 泛化性，也不声称全局 IK 分支保持或连续时间安全保证。

## 冻结方法和数值条件

- Python：`/home/rocos/miniconda3/envs/mujo/bin/python`
- MuJoCo model：`AviatorRobot/build/bin/model/aviator.xml`
- 流形：`hil-serl/data/aviator/manifold_phi_stale_pitch-10deg`
- `dt=0.005 s`，每条轨迹 100 个候选步
- `d_safe=0.005 m`，`beta_stop=0.125`
- `m_curv=0.50 mm`，swept checker subdivisions 为 5
- 每个 cell 的同一初始状态同时运行两个方法：
  1. 最终 endpoint-aware online governor；
  2. N6 冻结的 `stale_lut_safe_scaled`。
- 两个方法使用同一个实际扰动 XML、同一个 MuJoCo checker、同一个初始 `(x,q,phi)`、同一个任务速度和步数。
- 初始状态筛选为独立 checker clearance ≥7 mm 且无接触。无法达到目标样本数的 cell 记录为 `sampling_incomplete`，不改变阈值。

## 误差矩阵

误差均在世界坐标系定义，双臂基座误差采用左臂 `+half`、右臂 `-half` 的相对安装误差。

| 家族 | 幅值 | 方向/轴 | XML 注入方式 |
|---|---:|---|---|
| fixture translation | 0, 1, 2, 5 mm | +x | 两个夹具壁共同平移 |
| fixture rotation | 0, 0.25, 0.5, 1 deg | +z | 两个夹具壁共同绕 z 轴旋转其法向 |
| base translation | 0, 0.5, 1, 2 mm | +x | 左右基座相向平移 |
| base rotation | 0, 0.1, 0.25, 0.5 deg | +z | 左右基座相反旋转 |

首轮 pilot 使用 nominal task speed、每个 cell 10 个 paired trials。若运行正常，再在同一冻结误差矩阵上加入 gentle/stress 两个速度，不能事后挑选误差等级。

## 输出和判定

每一步保留方法级停止原因、任务进度、endpoint/swept clearance、闭合误差和延迟。`complete` 只表示 100 个候选步都被接受；任务进度单独报告。

主要比较：

- completion rate 与 task progress；
- endpoint/swept 最小 clearance 和安全停止原因；
- closure residual 与 joint-limit 失败；
- control-cycle P95/P99 latency；
- 误差幅值的性能退化曲线。

本节点主结果是“在测试误差包络内的经验鲁棒性”；不把在线方案获得更高完成率解释为全局安全定理，也不把 stale LUT 的失败解释为 branch jump。

## 结果分层

1. pilot：nominal speed，验证模型变体、配对状态、日志和停止分类完整。
2. formal：通过 pilot 数据完整性检查后，运行全部速度和预声明 cell。
3. rebuilt LUT：只有在明确实现同等校准信息后作为独立附加条件；不与 stale LUT 或 online 结果混为同一输入信息条件。
