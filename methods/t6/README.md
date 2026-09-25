# t6：下午 60k actor + transported-phase 查表

这个目录是可单独编译的 C++17 方法库。当前权重来自下午的 800×64 训练
`hil-serl/examples/experiments/aviator_manifold/route_a_lut_native_rlpd_800x64/checkpoint_60000`；
固定 50 条验证轨迹完成 49 条。`actor_60k.onnx` 是从该 Flax 检查点导出的
确定性 actor 均值网络。旧的 `actor_160k.onnx` 保留作历史对照。C++ 使用
ONNX Runtime 在进程启动时加载模型路径；同一 `float32 [1,40] → [1,2]` 接口的其他
`.onnx` 权重可直接替换，**不需要重新编译 C++**。进程运行期间不切换权重。
替换模型必须保留相同的 40 维观测顺序、输出动作含义和训练尺度；形状正确但语义
不同的模型不能安全互换。
该训练的 `checkpoint_160000` 属于同一轮训练的后续步骤；当前选择的是
验证成绩更高的 `checkpoint_60000`。
这 49/50 是训练环境的固定验证轨迹成绩，不代表整个 LUT 矩形域可闭环执行；
MuJoCo 大范围实测结果见[60k 全范围测试报告](results/actor_60k_full_range_report.md)。

## 数据流

调用方每 10 ms 提供**同一时刻**的实际任务状态 `x=(theta [rad], s [m])`、实际
`xdot`、实测 14 关节角和单调时钟时间，并提供下一周期任务目标 `x_target_next`。
`T6` 从实测关节角反查当前相位，检查左右臂重建残差，构造与训练一致的 40 维观测；
ONNX actor 输出两臂归一化相位速度 `action∈[-1,1]^2`，再计算
`phi_command = phi_estimate + action * 1.5 * 0.01`。最后在 `manifold_phi` 中查表，
返回 14 个关节参考值。查表器读取 `qL/qR`、`dL/dR`、`safe`、`branch` 和 `phi`。

首次调用 `T6::operator()` 全局反查相位并填充历史，不输出关节指令。后续调用将反查
限制在上一周期相位附近，使用可配置的连续性权重，并检查每臂 RMS/最大单关节重建
残差、反馈周期、相位安全区间、分支、查表间隙、关节限位以及指令相对实测关节角的速度。
失败会锁存，必须在上层确认状态后调用同一个函数并传 `rearm=true` 才能重新定位。
`has_command == true` 是发送 `q` 的必要条件。

`StepResult` 还保留安全诊断：`stage == policy_command` 表示 RL 候选动作已生成且在
执行前被拒绝，`stage == feedback` 表示实测状态本身没有通过检查，`target` 表示
下一任务目标非法，`reset` 表示初始化定位失败。`actor_error` 表示推理失败或输出非法，
此时 `has_action` 为假。`action` 是原始 RL 输出，`candidate_x/candidate_phi` 是尝试查询的
下一状态，`evaluated_sample` 包含查表后的安全相位裕度、间隙、关节限位裕度及分支。
只有 `has_evaluated_sample` 为真时才读取它；候选点超出 LUT 网格时该标记为假。
在线执行会先检查原始动作；若不可行，沿同一动作方向依次尝试 1/2、1/4 等比例，
直至零动作。`action` 保留原始 RL 输出，`executed_action` 是实际通过检查的动作，
`intervened` 标记这次缩小。若这些候选都不可行，则拒绝并锁存原始动作的失败原因。
这是一维缩放回退，尚非训练版二维可行域的最近点投影。
速度检查会给出 `max_joint_speed` 和 `limiting_joint`（0–13）。
速度门只检查新关节目标相对本周期实测关节角的变化量除以 `10 ms`；
`max_joint_speed` 是这一候选步长的诊断值，不是由连续反馈求出的实际关节速度。
`new_failure` 只在新故障发生的那次 `T6` 调用为真；锁存期间重复调用返回同一故障，
但 `new_failure` 为假。记录 RL 不可行事件可使用
`new_failure && stage == CheckStage::policy_command`，并用 `status_name(status)`
取得稳定的原因字符串。建议上层将结果拷贝到日志队列，再由非控制线程写 CSV，
不要在 10 ms 控制回调里同步写磁盘。至少记录时间、模型与 LUT 版本、实测 `x/q`、
估计相位及残差、原始动作、候选 `x/phi`、失败原因和对应安全裕度。

没有隐式逆解回退。查表间隙是构表时的近似值，接入实体或动力学仿真时仍需上层
做实际碰撞、跟踪误差和急停检查。

这套方法适用于训练用的二维任务流形，不是任意末端位姿的通用逆解。接口与旧逆解
的替换边界应放在“任务状态 → 14 关节参考值”一层，旧规划器需要提供 `theta/s/xdot`。

## 流形文件与查表

原始表位于 `hil-serl/data/aviator/manifold_phi/`，由
`AviatorRobot/tools/clearance_trajectory.cpp` 的 `build_manifold_phi` 离线生成。
`manifest.json` 规定 `n_theta=101, n_s=65, n_phi=128`，即 6,565 个任务网格点、
840,320 个 `(x,phi)` 样本。主索引为
`((i_s * n_theta + i_theta) * n_phi + i_phi)`，每个样本再按关节 `0..6` 排列。
`phi.bin` 是两臂共享的 128 个 float32 相位节点；`qL/qR.bin` 各存
`(840320,7)` float32 关节角，`dL/dR.bin` 各存 840,320 个 float32 间隙值。
`safe.bin` 对每个 `(theta,s)` 点存 `[左下界,左上界,右下界,右上界]` 四个 float32；
`branch.bin` 对每个点存一个 uint8 分支码。目录还存有 `QxL/QxR/QphiL/QphiR.bin`
导数表；当前 `t6` 推理不读取这些导数文件。

正向查询对 `qL/dL` 使用 `(theta,s,phiL)` 的八角点三线性插值，对 `qR/dR` 使用
`(theta,s,phiR)`；安全区间只对 `(theta,s)` 四角点双线性插值，分支码取四角点最大值。
因此 `d_min=min(dL,dR)` 是离线表值的插值，不能当作实时碰撞测量。
`verify_lut.py` 在 600 个随机点和 27 个边界/网格节点上对照 Python 训练查表实现。

## 从实测关节角反查相位

`Lookup::estimate_phase(x, measured_q)` 对左右臂各自求解
`argmin_phi ||q_measured - Q_arm(x, phi)||²`。它先用任务状态 `x` 找安全相位区间，
再逐段扫描查表的相位轴；每段的 7 关节配置对相位是线性的，因此把实测关节向量
投影到这条 7 维线段上即可找到该段最佳相位，最终取残差最小的段。返回两臂相位和
各自 7 关节 RMS 和最大单关节残差。在线反查再与上一周期相位窗口相交，并在优化
目标中加入连续性惩罚。该函数需要 **实际任务状态 x 和实际 14 关节角**；仅凭关节角
不能唯一确定相位。阈值需要用实机正常抓握和跟踪数据标定。当前未实现多解歧义
判别，首次全局定位时尤其需要上层确认抓握状态。

当前 `AviatorRobot` 的 `Feedback.angle/displacement/velocity` 来自 MuJoCo 共享内存，
不能直接当作真机传感器。真机还需要提供方向盘角度/位移的测量或经过验证的估计。

## C++ 调用

```cpp
aviator::t6::FeedbackLimits limits;
// 以下仅为离线测试值，真机阈值必须由实测数据标定：
limits.max_rms_joint_error = 0.02; // rad
limits.max_joint_error = 0.05;     // rad
limits.max_phase_change = 0.025;   // rad per 10 ms
limits.continuity_weight = 0.001;
limits.max_joint_speed = 1.5;      // rad/s
limits.min_clearance = 0.005;      // m, LUT predicted
limits.time_tolerance = 0.002;     // s

aviator::t6::T6 t6(model_onnx_path, manifold_directory, limits);
// 每个 10 ms 控制周期只调用这一处：
auto result = t6({measured_x, measured_xdot, measured_q, monotonic_time}, next_x);
if (result.new_failure && result.stage == aviator::t6::CheckStage::policy_command)
    enqueue_safety_event(result); // 由调用方实现：拷贝到非控制线程的日志队列
if (result.has_command) send_joint_target(result.q);
else hold_current_position();
// 若故障已确认并需要重新定位：t6(feedback, next_x, true);
```

以上代码中的 `send_joint_target` 和 `hold_current_position` 由机器人上层实现。
`FeedbackLimits` 不提供真机默认阈值；单元测试中的数值只用于验证逻辑。

## 构建、导出与验证

构建需要 ONNX Runtime C++ 发布包的 `include/` 与 `lib/`。Linux x64 可运行
`setup_deps.sh` 从官方发布包下载到被 Git 忽略的 `.deps/`；其他平台或已有安装可用
CMake 参数 `ONNXRUNTIME_ROOT` 指向相应目录。运行时动态链接器也必须能找到其共享库。
导出模型时需要 Flax 训练环境和 Python `onnx` 包；运行 C++ 不需要 Python。

在仓库根目录运行：

```bash
JAX_PLATFORMS=cpu /home/rocos/miniconda3/envs/serl_clean/bin/python methods/t6/export_actor.py
/home/rocos/miniconda3/envs/env_isaaclab/bin/python methods/t6/actor_to_onnx.py
bash methods/t6/setup_deps.sh
cmake -S methods/t6 -B /tmp/aviator_t6_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/aviator_t6_build -j4
ctest --test-dir /tmp/aviator_t6_build --output-on-failure
/tmp/aviator_t6_build/t6_smoke methods/t6/actor_60k.onnx hil-serl/data/aviator/manifold_phi 0 -0.08 0 -0.08 0 0
JAX_PLATFORMS=cpu /home/rocos/miniconda3/envs/serl_clean/bin/python methods/t6/verify.py /tmp/aviator_t6_build/t6_smoke
JAX_PLATFORMS=cpu /home/rocos/miniconda3/envs/serl_clean/bin/python methods/t6/verify_lut.py /tmp/aviator_t6_build/t6_lut_probe
```

`verify.py` 对照原 Flax actor 的确定性输出和 Python `ManifoldLookup` 的 14 关节
查表结果；`ctest` 覆盖反馈闭环和故障门控。C++ 项目链接 `aviator_t6` 并包含
`t6.hpp`；模型及 LUT 路径在启动时传入。部署需要 C++ 可执行文件、ONNX Runtime
共享库、选定的 `.onnx` 模型和查表文件。

## MuJoCo 反馈正弦闭环测试

`AviatorRobot/build/bin/aviatorT6Sine` 先启用双臂、锁住手柄，移动到
`(theta,s)≈(0,-0.08)`；随后每 10 ms 从 MuJoCo 的同一步快照读取方向盘状态、
速度与 14 个实际关节角，运行 `T6`，并把安全检查通过的关节目标写回驱动。
方向盘参考每 20 ms 生成一个含位置和速度的节点（50 Hz），节点间使用三次
Hermite 插值，保证参考位置与速度在节点处连续；`T6` 仍以 10 ms（100 Hz）
运行。正弦轨迹默认两轴同相、幅值 `(0.1 rad,0.01 m)`、频率 `0.1 Hz`、持续
`10 s`，覆盖一个完整周期。
默认关节速度检查为 `1.5 rad/s`，与训练环境的速度门槛一致；命令行最后一个可选
参数可设置其他测试门槛。
起点的 `T6` 调用只做相位定位，不发送动作。

在两个终端中分别执行：

```bash
cd reference/rocos-mujoco/build/aviator-visual
./bin/rocos_mujoco_sim --model aviator --headless --duration 90
```

```bash
AviatorRobot/build/bin/aviatorT6Sine methods/t6/actor_60k.onnx \
  hil-serl/data/aviator/manifold_phi /tmp/mujoco_sine.csv 10 0.1 0.01 0.1
```

CSV 包含期望/实际任务状态、实际/指令关节角、原始/执行动作、相位反查残差、查表间隙、
抓握误差、控制状态和 `T6` 求解耗时。发生安全拒绝时测试保持当前关节位置、停止并
保存已有数据。MuJoCo 与 AviatorRobot 必须同时重新构建：反馈协议已升级到 v2，
同一快照新增 `joints[14]`。`ctest --test-dir AviatorRobot/build -R
aviator_t6_wheel50_integration --output-on-failure` 会在独立总线上执行完整周期
闭环测试，并检查方向盘参考轨迹及关节目标相对实测角的步长。

当前保证的是方向盘参考的 `C1` 连续和每周期关节目标相对实测角的步长上限。
这不约束相邻关节指令的变化率，也不直接测量实际关节速度。实体机器人部署前
还要验证实测速度、底层执行器限速以及跟踪误差。

当前 60k 模型的闭环测试见
[大范围结果](results/actor_60k_full_range_report.md)。原 160k 模型的 10 秒运行见
[结果摘要](results/mujoco_sine_10s_50hz_feedback_gate.md) 与
[原始 CSV](results/mujoco_sine_10s_50hz_feedback_gate.csv)。此前增加相邻指令约束的
[历史对照](results/mujoco_sine_10s_50hz_rate_guard.md)仅供比较。此前的 4 秒
[1.5 rad/s 运行](results/mujoco_sine_4s_1p5.md) 和 `0.7 rad/s` 的
[对照结果](results/mujoco_sine_4s.md)也保留。这些是 MuJoCo 动力学反馈结果，
不代表实体机器人精度。

## USB 摇杆闭环测试

终端 1 从仿真器所在目录启动 headless MuJoCo（省略 `-v` 即为 headless）：

```bash
cd /home/rocos/sia/AVIATOR/reference/rocos-mujoco/build/aviator-visual/bin
./rocos_mujoco_sim --model aviator --duration 90
```

终端 2 从仓库根目录启动 60k 模型和摇杆控制器：

```bash
cd /home/rocos/sia/AVIATOR
./AviatorRobot/build/bin/aviatorT6Joystick \
  methods/t6/actor_60k.onnx hil-serl/data/aviator/manifold_phi \
  joystick.csv 60 /dev/input/js0 0 1.5 0.04
```

命令中的 `60` 是仿真控制持续时间（秒），到时会正常保持、松开并退出；如需更长
时间，将这个参数改大，当前程序允许最多 120 秒。控制器先将方向盘移动到
`(θ,s)=(0,-0.08 m)`，再开始读取摇杆。轴 0 的原始值 `-32767/0/+32767`
分别对应 `−50°/0/+50°`；轴 1 分别对应 `−159/−80/−1 mm`。
两端各留 1 mm 的 LUT 边界余量。零点附近使用 5% 死区，其余范围重新归一化；
还经过每 20 ms 的 EMA 和任务参考斜率限制，因此映射结果不会在一个周期内跳到
满量程。CSV 的 `axis0_raw/axis1_raw` 记录内核返回的原始整数，`theta_ref/s_ref`
记录平滑及限速后的请求，`theta_cmd/s_cmd` 记录可行性缩小后的目标。
本机 PXN-F16 的两轴居中原始值已实测为 `(0,0)`。

方向盘参考每 20 ms 更新一次，T6 每 10 ms 运行一次。PXN-F16 不提供可用
震动接口时，T6 控制器关闭摇杆力反馈探测，避免每 2 秒的重试阻塞控制回调。
该故障的复现与修复验证见[摇杆超时报告](results/joystick_deadline_fix.md)。

摇杆的 `theta_rate=1.5 rad/s` 是**方向盘角速度**上限，不保证 14 个关节的
目标步长也低于 `1.5 rad/s`。控制器现在每个 10 ms 周期先用实测关节角反查相位，
再把摇杆请求的下一任务目标沿“实测任务状态 → 请求目标”缩小，要求在当前相位下
查表安全且目标关节角相对实测角的步长不超过关节限值的 80%。RL 动作改变相位后，
T6 仍执行最终完整安全检查；若仍不可行则保持当前位置并重新定位。CSV 中
`theta_ref/s_ref` 是摇杆请求，`theta_cmd/s_cmd` 是缩小后的目标，`task_scale` 是
本周期采用的比例。`compute_ms` 和终端 `control_compute_*` 包含预筛选与 T6 推理。
这项限制防止请求目标在被保护停住时继续作为一步目标直接跳过去；真实摇杆持续运动
时仍应监视拒绝次数和跟踪误差。

关节阶跃诊断使用 `AviatorRobot/build/bin/aviator_joint_step_probe` 与
`AviatorRobot/tests/run_joint_step_probe.py` 在独立总线上记录控制器下发时刻、
仿真器接收时刻和 1 ms 关节轨迹。结果见
[阶跃响应报告](results/joint_step_response.md)。
仿真位置环增益的阶跃与闭环对照见[位置环提速试验](results/position_gain_sweep.md)；
可用 `AVIATOR_SIM_POSITION_KP/KD` 在启动 MuJoCo 时临时覆盖默认值，不改
控制器的 100 Hz 频率，也不影响真机参数。
另有单独的 [20k/500 位置执行器实验模型](results/position_actuator_20k500.md)，
通过 `--model aviator --urdf /home/rocos/sia/AVIATOR/reference/rocos-mujoco/model/aviator_position_20k500.xml`
选择。该模型小阶跃明显振荡，普通 `--model aviator` 不会加载它。
