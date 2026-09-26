# `kp=20000, kv=500` MuJoCo 位置执行器试验

`aviator.xml` 原来没有 arm actuator；仿真器对每个驱动关节使用力矩比例控制和
被动阻尼。`mujoco_simulator.cpp` 中提到 `kp=20000, kv=500` 的旧注释与这个
模型不符，已修正。现在另有可选模型
`reference/rocos-mujoco/model/aviator_position_20k500.xml`，给左右臂各 7 个
实际全名关节都加了 `<position kp="20000" kv="500">`。关节 2、4、6、7 的限位
窄于 `[-3.14,3.14]`，故每个 `ctrlrange` 使用对应关节的真实限位。原默认
`aviator.xml` 保留原有控制方式。
该实验模型有内置 actuator，仿真器会使用 `d->ctrl` 路径；针对原回退控制器的
`AVIATOR_SIM_POSITION_KP/KD` 环境变量不会改动这些 XML actuator 的增益。

在锁定双臂抓握、方向盘目标 +0.005 rad 的 LUT 协调阶跃中：

| 指标 | 结果 |
| --- | ---: |
| 下发至仿真器接收 | 0.694 ms |
| 左臂关节 1 首次越过目标变化的 90% | 3.17 ms |
| 10 ms 时关节完成比例 | 209.4% |
| 关节峰值完成比例 | 211.2% |
| 随后反向谷值 | −33.0% |
| 方向盘目标 / 短时峰值 | 0.005 / 0.00916 rad |
| 500 ms 时关节完成比例 | 100.9% |

因此这组增益在此负载和抓握约束下明显欠阻尼。首次到 90% 不等于稳定到位；
这一阶跃出现了显著正反振荡。不能把它直接用于正常摇杆测试并仅根据
“3.17 ms”宣称跟踪更好。原始[1 ms 轨迹](coordinated_joint_step_actuator_20k500/joint_step_sim_1ms.csv)、
[控制反馈](coordinated_joint_step_actuator_20k500/joint_step_feedback.csv)和
[命令时间](coordinated_joint_step_actuator_20k500/joint_step_send.csv)已保存。

## 缩小方向盘参考阶跃

保持 `kp=20000, kv=500`、初始方向盘约为 `(θ=0, s=-80 mm)`、双臂抓握和
LUT 关节目标生成方式不变，只改变一次性施加的方向盘 θ 参考步长。
控制反馈约每 10 ms 一帧。表中“10 ms 完成比例”使用下发后最接近 10 ms 的
一帧（约 10.3–10.5 ms）；“稳定进入 ±5%”要求在剩余 500 ms 观察窗口内
一直保持在目标的 ±5% 步长范围内。

| θ 参考步长 | 约 10 ms 时 θ 完成比例 | θ 峰值比例 | θ 稳定进入 ±5% |
| ---: | ---: | ---: | ---: |
| 0.00020 rad | 28.2% | 104.8% | 147 ms |
| 0.00100 rad | 84.0% | 108.6% | 137 ms |
| 0.00110 rad | 99.3% | 108.1% | 137 ms |
| 0.00125 rad | 117.5% | 117.5% | 137 ms |
| 0.00200 rad | 150.0% | 150.0% | 137 ms |
| 0.00500 rad | 183.2% | 183.2% | 148 ms |

0.00110 rad 特别容易误判：10.3 ms 时恰好在目标 ±5% 内，约 21 ms 时却
回落到目标步长的 23.5%，约 42 ms 时又到 108.1%。因此**减小一次参考阶跃
并没有让方向盘在 10 ms 内稳定到位**。对于 50 Hz 连续下发，新参考每 20 ms
变化一次；这种约 137 ms 的振荡尚未结束，就会收到多条新指令。单次阶跃的
“10 ms 瞬时碰到目标”不能作为连续轨迹跟踪能力。

这些数据只覆盖这里的初始抓握姿态和一次正向 θ 阶跃。0.00020 rad 已接近
该试验中的反馈波动量级，不宜用它推断微小动作的绝对精度。每组完整的
下发时间、逐毫秒关节响应和 10 ms 方向盘反馈保存在对应
`coordinated_joint_step_actuator_20k500[_步长]/` 目录中。

复现实验时，最后一个参数指定方向盘步长，例如：

```bash
AVIATOR_STEP_MODEL=reference/rocos-mujoco/model/aviator_position_20k500.xml \
python3 AviatorRobot/tests/run_joint_step_probe.py \
  reference/rocos-mujoco/build/aviator-visual/bin/rocos_mujoco_sim \
  AviatorRobot/build/bin/aviator_joint_step_probe \
  AviatorRobot/config/aviator.yaml \
  methods/t6/results/coordinated_joint_step_actuator_20k500_0p0011 \
  hil-serl/data/aviator/manifold_phi 0.0011
```

使用可选模型时，在仿真器命令末尾追加：

```bash
--urdf /home/rocos/sia/AVIATOR/reference/rocos-mujoco/model/aviator_position_20k500.xml
```

普通 `--model aviator` 仍使用原默认模型。生成脚本的
`--position-actuators` 选项可重新生成这个实验模型。下一步若继续调这套位置执行器，
应在独立仿真中提高阻尼或降低比例增益，并同时检查方向盘超调、抓握误差和
控制器安全拒绝，不能只比较首次到达时间。
