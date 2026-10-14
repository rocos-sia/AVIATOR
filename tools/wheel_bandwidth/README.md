# 无相机轮盘带宽 demo

独立入口 `aviator_core_wheel_bandwidth`，通过现有 `servoWheel()` 发送正弦目标。
读取 `arm.state` 的左右臂**实测关节角**，用现有 URDF 正运动学及 `grasp.json`
反算轮盘角度；不启动相机，也不使用相机观测。

## 构建与一键启动

```bash
cmake --build build/release --target aviator_core_wheel_bandwidth aviator_bus manipulator aviator_monitor -j2

# 预览所有命令，不启动节点、不生成文件
./scripts/start_wheel_bandwidth.sh --dry-run

# 真机与真实手；脚本在需要时通过 sudo 提示认证
./scripts/start_wheel_bandwidth.sh

# 指定单频，或自定义递增频点
./scripts/start_wheel_bandwidth.sh --frequencies 0.2
./scripts/start_wheel_bandwidth.sh --frequencies 0.1,0.2,0.3,0.5

# 真机机械臂 + 模拟手，或额外记录 MCAP
./scripts/start_wheel_bandwidth.sh --fake-hand
./scripts/start_wheel_bandwidth.sh --logger

# 无相机仿真验证；首次需构建 simulation
cmake --build build/release --target simulation -j2
./scripts/start_wheel_bandwidth.sh --simulation --headless --no-monitor \
    --frequencies 0.5 --settle-cycles 1 --measure-cycles 2
```

默认二进制目录 `build/release/bin`，可用 `AVIATOR_BIN` 覆盖。脚本默认用
`/usr/bin/python3 -s`，依赖 PyYAML、NumPy、Matplotlib；`-s` 避免用户 site-packages
影响系统科学计算库。其他 Python 环境可通过 `BANDWIDTH_PYTHON` 指定。
真实手由独立 xCoreSDK_hand 的 SDK RS485 连接提供，Monitor 可用 `MONITOR_BIN` 指定。
脚本在启动设备前检查分析依赖。`--no-monitor` 关闭 Monitor 和浏览器。

先退出其他控制节点。脚本与现有一键启动共用互斥锁，并检查控制端口占用。
请直接以普通用户运行，不要在整个脚本前加 `sudo`；脚本会单独为 manipulator 请求认证。
输出目录会先完成可写检查和实际创建，再请求认证或启动节点。
Ctrl+C 先通知 Core 减速、解锁和下使能，最多等待 15 秒，再停止本次启动的其他节点。
正常结束也会自动退出节点、生成分析结果。不会自动执行真实硬件测试来验证安装。

## 输入与抓取

- 启动时读取原 `system.yaml` 和 `robot.yaml`，生成本次输出目录下的配置副本。
  副本仅将 `grasp_wheel.source` 改为 `wheel_initial`，并解析相对资源路径。
  抓握几何与 posture 另存快照；不改写原配置，不改变既有 Core 或启动脚本。
- 抓取角度、推拉位移均取 `wheel_initial`。真机轮盘须预先放在该声明位形；
  配置不包含实际位形测量，也不会自动识别摆放误差。
- 自动使能、接近抓握、锁定后，先平滑移动到角度 `0`，推拉位移保持
  `wheel_initial.displacement`，再开始正弦测试。
- 默认幅值 **10°**、频率 **0.1、0.2、0.3、0.5、0.7、1 Hz**。
  每个频点依次为：1 周期平滑增幅、2 周期等待、5 周期测量、1 周期平滑减幅。
  默认扫频约 205 秒，另加抓握、回中及清理时间。
- 幅值可通过 `--amplitude-deg` 降低，上限 10°；频率范围 0.01..5 Hz，严格递增。
  目标发送沿用 `servo_period`，还要求每个正弦周期至少 20 个目标采样。
  `--settle-cycles`、`--measure-cycles`、`--ramp-cycles` 分别控制上述周期数。
- 正常结束先回中再解锁、下使能；中断或故障时减速停止，不强行回中。
  `--fake-hand` 仅模拟手部节点，不证明真实轮盘受到可靠约束。

## 关节角如何变成轮盘角度

沿用现有 Core 的几何定义：

```text
F = O · Rz(angle)Tz(displacement) · H · inverse(T)
W_est = inverse(O) · FK(q_measured) · T · inverse(H)
```

`O` 为 wheel_origin，`H` 为各侧抓握位姿，`T` 为各侧法兰到工具变换。
将 `W_est` 的旋转投影到轮盘 Z 轴，得到每侧转角；Z 平移得到每侧推拉估计。
左右角度按圆周平均融合，同时保存差值、横向平移残差和轴外旋转残差。
不把 `wheel_reference` 或指令关节角当作实测反馈。

两侧须为同机时钟的新鲜有效反馈，每侧采样时间严格递增，时间差不超过 5 ms。
使用两侧采样时间的中点作为融合估计时刻。超过 500 ms 没有合格测量则结束测试；
原有更短的设备通信超时和故障保护仍生效。

这是**关节反馈估计角度**，依赖抓握无滑动、工具与机器人标定准确及刚性假设。
双臂一起滑动等误差可能不会体现在左右差值中，不能据此宣称测到了实体轮盘真值。
它测量的是现有 Servo / Ruckig / IK / 机械臂跟踪链路的整体响应，包含轨迹整形影响。

## 结果

默认保存在 `logs/wheel_bandwidth_<北京时间>/`。若项目的 `logs/` 不可写（例如属于 root），
自动转到 `~/aviator_logs/wheel_bandwidth_<北京时间>/`，并在终端打印实际路径；不修改已有目录权限。
可通过 `--output` 指定一个尚不存在的目录，显式指定的目录不可写时会直接报错，不自动换路径：

| 文件 | 内容 |
|---|---|
| `commands.csv` | 实际发送目标及单调时间 |
| `samples.csv` | 原始关节角、左右采样时间、正弦目标、执行参考、估计角度和几何残差 |
| `run.json` / `launch.json` | 完成状态、参数和配置来源 |
| `frequency_response.csv` / `summary.json` | 每频点增益、相位、拟合残差、有效性及带宽估计 |
| `tracking.png` / `frequency_response.png` | 时序及幅频、相频曲线 |
| `config/`、`*.log` | 本次配置副本和节点日志 |

仅在恒幅测量区间，按原始采样时间对输入、输出分别进行带常数偏置的正弦最小二乘拟合。
输入使用该采样时刻之前最近一次发送的目标；相位为输出减输入，负值表示滞后。
`wheel_reference` 的采样时刻与关节反馈并非严格一致，仅用于诊断绘图。
不做会引入额外相位的平滑滤波。

频点不完整、采样不足/中断、非正弦残差超过幅值 25%，或几何一致性超阈值
（左右差 3°、横向残差 20 mm、轴外旋转 5°）时标为无效，不据此报告带宽。
这些阈值是分析质量标记，不是硬件保护阈值，也不能独立判断是否滑动。

完整有效的多频点测试，以第一个频点增益为低频基准，寻找首次下降 3 dB 的区间，
在对数频率轴插值估计带宽。首频点并非经过独立验证的直流增益，结果为本次扫频估计。
单频、未覆盖 -3 dB、无效或中断测试不会输出虚构的带宽数值。

重新分析已有数据：

```bash
/usr/bin/python3 -s tools/wheel_bandwidth/analyze.py logs/wheel_bandwidth_实际目录
```

## 离线验证

```bash
cmake --build build/release --target aviator_wheel_bandwidth_test -j2
ctest --test-dir build/release -R 'wheel_bandwidth_(estimate|help)' --output-on-failure
/usr/bin/python3 -s -m unittest discover -s tools/wheel_bandwidth -p 'test_*.py' -v
```

覆盖真实 URDF 的 IK→FK 角度反算、工具变换与推拉位移、圆周角融合、正弦幅值与过渡、
不规则采样下的已知增益/相位、无效数据处理、-3 dB 区间插值及原配置不变。

手控制由独立 xCoreSDK_hand 提供；`--fake-hand` 传给该节点。生成运行配置时会将手配置路径一并转为绝对路径。
