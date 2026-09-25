# MuJoCo 关节阶跃响应（2026-09-24）

在独立总线上用同一个仿真模型、1 ms 物理周期、双臂抓住方向盘且方向盘初始
`(0,-80 mm)`。控制器约每 10 ms 读取反馈。记录控制器写入关节目标后的
`CLOCK_MONOTONIC` 时间，以及仿真器首次读到该目标的时间和其后每 1 ms 的关节角。
被测关节是左臂关节 1。以下百分比是该关节相对本次目标变化量的完成比例。

| 阶跃类型 | 控制器到仿真器 | 10 ms | 50 ms | 100 ms | 200 ms | 500 ms | 首次到 90% |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 单独改变左臂关节 1，+0.005 rad | 0.586 ms | 3.5% | 13.7% | 19.3% | 23.7% | 25.3% | 500 ms 内未到 |
| LUT 协调双臂，方向盘 +0.005 rad；该关节目标变化 +0.000672 rad | 0.875 ms | 21.6% | 42.4% | 70.1% | 92.3% | 101.2% | 185 ms |

结论：目标在一个 1 ms 仿真周期内进入执行器，关节没有在下一个 10 ms 控制反馈
周期到位。单独改变一个关节时，已锁定的抓握约束与其余关节不动相冲突，不能用
该行的 25.3% 推断无约束执行器速度。协调双臂的第二行更接近 T6 的真实下发方式，
其 90% 响应时间约 185 ms。这一小阶跃结果说明 10 ms 是命令与反馈周期，并非
到位时间；不同关节、幅度、方向盘负载和控制增益下响应时间可能不同。

原始数据：

- 单关节：[控制反馈](joint_step_feedback.csv)、[下发时间](joint_step_send.csv)、[1 ms 仿真轨迹](joint_step_sim_1ms.csv)。
- 双臂协调：[控制反馈](coordinated_joint_step/joint_step_feedback.csv)、[下发时间](coordinated_joint_step/joint_step_send.csv)、[1 ms 仿真轨迹](coordinated_joint_step/joint_step_sim_1ms.csv)。

复现：`python3 AviatorRobot/tests/run_joint_step_probe.py SIMULATOR STEP_PROBE CONFIG OUTPUT_DIR`
测单关节；末尾再加 `hil-serl/data/aviator/manifold_phi` 测双臂协调。仿真器设置
`AVIATOR_STEP_TRACE_FILE` 和 `AVIATOR_STEP_TRACE_JOINT` 时才记录 1 ms 轨迹，
控制器设置 `AVIATOR_JOINT_SEND_TRACE_FILE` 时才记录命令时间。
