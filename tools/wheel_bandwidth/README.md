# 无相机轮盘带宽 demo

独立入口 `aviator_core_wheel_bandwidth`，通过现有 `servoWheel()` 发送正弦目标。
读取 `arm.state` 的左右臂**实测关节角**，用现有 URDF 正运动学及 `grasp.json`
反算轮盘角度和推拉位移；不启动相机，也不使用相机观测。

## 构建与一键启动

```bash
cmake --build build --target aviator_core_wheel_bandwidth aviator_bus manipulator xCoreSDK_hand aviator_monitor -j2

# 预览所有命令，不启动节点、不生成文件
./scripts/start_wheel_bandwidth.sh --dry-run

# 真机与真实手；脚本在需要时通过 sudo 提示认证
./scripts/start_wheel_bandwidth.sh

# ±3° 转动 / ±20 mm 推拉，分别运行；真实机械臂搭配模拟手
./scripts/start_wheel_bandwidth.sh --axis rotation --amplitude-deg 3 --fake-hand
./scripts/start_wheel_bandwidth.sh --axis translation --amplitude-mm 20 --fake-hand

# 指定单频，或自定义递增频点
./scripts/start_wheel_bandwidth.sh --frequencies 0.2
./scripts/start_wheel_bandwidth.sh --frequencies 0.1,0.2,0.3,0.5

# 真机机械臂 + 模拟手，或额外记录 MCAP
./scripts/start_wheel_bandwidth.sh --fake-hand
./scripts/start_wheel_bandwidth.sh --logger

# 无相机仿真验证；首次需构建 simulation
cmake --build build --target simulation -j2
./scripts/start_wheel_bandwidth.sh --simulation --headless --no-monitor \
    --frequencies 0.5 --settle-cycles 1 --measure-cycles 2
```

默认二进制目录 `build/bin`，与 `scripts/start_aviator.sh` 的源码运行默认值一致，
可用 `AVIATOR_BIN` 覆盖。若使用 `release` preset 构建，请先执行
`cmake --preset release`，再将上述构建目录改为 `build/release`，启动时使用
`AVIATOR_BIN="$PWD/build/release/bin" ./scripts/start_wheel_bandwidth.sh`。
脚本默认用
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

测试参数统一放在 [`config/wheel_bandwidth.yaml`](../../config/wheel_bandwidth.yaml)，
脚本从任意工作目录启动都会默认读取它。`--config` 仍用于选择系统配置，
`--sweep-config /path/to/test.yaml` 用于选择其他测试配置；命令行参数优先于 YAML。

```yaml
axis: rotation          # rotation 转动；translation 推拉，每次只测一个轴
amplitude_deg: 3.0       # ±3°，只用于 rotation
amplitude_mm: 20.0       # ±20 mm，只用于 translation
frequencies_hz: [0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
settle_cycles: 2
measure_cycles: 5
ramp_cycles: 1
fake_hand: false         # true：真实机械臂 + 模拟手
ignore_geometry: false
```

例如改为 `axis: translation`、`fake_hand: true`，直接运行脚本即可测试推拉。
`--fake-hand` / `--no-fake-hand` 可临时覆盖手模式；
`--ignore-geometry` / `--no-ignore-geometry` 可临时覆盖分析策略。
`fake_hand: true` 不可与 `--simulation` 同用；仿真时可加 `--no-fake-hand` 覆盖。
配置文件须保留以上全部字段，拼错字段、无效数值及越界范围会在启动节点前报错。

- 启动时读取原 `system.yaml` 和 `robot.yaml`，生成本次输出目录下的配置副本。
  副本仅将 `grasp_wheel.source` 改为 `wheel_initial`，并解析相对资源路径。
  抓握几何与 posture 另存快照；不改写原配置，不改变既有 Core 或启动脚本。
- 抓取角度、推拉位移均取 `wheel_initial`。真机轮盘须预先放在该声明位形；
  配置不包含实际位形测量，也不会自动识别摆放误差。
- 自动使能、接近抓握、锁定后，先平滑移动到角度 `0`、位移
  `wheel_initial.displacement`，再开始正弦测试。
- 转动以 **0°** 为中心，推拉保持初始位移；推拉以 **初始位移** 为中心，角度保持 0°。
  当前初始位移为 −85 mm，±20 mm 推拉对应 **−105～−65 mm**。
- 默认转动幅值 **3°**、推拉幅值 **20 mm**，默认选择转动；
  频率 **0.1、0.2、0.3、0.5、0.7、1 Hz**。
  每个频点依次为：1 周期平滑增幅、2 周期等待、5 周期测量、1 周期平滑减幅。
  默认扫频约 205 秒，另加抓握、回中及清理时间。
- 转动幅值通过 `amplitude_deg` / `--amplitude-deg` 设置，须在 (0,10]°；
  推拉幅值通过 `amplitude_mm` / `--amplitude-mm` 设置，须为正数，
  且中心 ± 幅值须全部位于 [-170,0] mm 内，超出时拒绝启动，不截断轨迹。
  命令行幅值单位须与所选轴匹配。频率范围 0.01..5 Hz，严格递增。
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
左右角度按圆周平均融合，推拉位移按算术平均融合，
同时保存左右差值、横向平移残差和轴外旋转残差。
不把 `wheel_reference` 或指令关节角当作实测反馈。

两侧须为同机时钟的新鲜有效反馈，每侧采样时间严格递增，时间差不超过 5 ms。
使用两侧采样时间的中点作为融合估计时刻。超过 500 ms 没有合格测量则结束测试；
原有更短的设备通信超时和故障保护仍生效。

这是**关节反馈估计位形**，依赖抓握无滑动、工具与机器人标定准确及刚性假设。
双臂一起滑动等误差可能不会体现在左右差值中，不能据此宣称测到了实体轮盘真值。
它测量的是现有 Servo / Ruckig / IK / 机械臂跟踪链路的整体响应，包含轨迹整形影响。

## 结果

默认保存在 `logs/wheel_bandwidth_<北京时间>/`。若项目的 `logs/` 不可写（例如属于 root），
自动转到 `~/aviator_logs/wheel_bandwidth_<北京时间>/`，并在终端打印实际路径；不修改已有目录权限。
可通过 `--output` 指定一个尚不存在的目录，显式指定的目录不可写时会直接报错，不自动换路径：

| 文件 | 内容 |
|---|---|
| `commands.csv` | 实际发送目标及单调时间 |
| `samples.csv` | 原始关节角、左右采样时间、正弦目标、执行参考、估计角度/位移和几何残差 |
| `run.json` / `launch.json` | 完成状态、参数和配置来源 |
| `frequency_response.csv` / `summary.json` | 每频点增益、原始及展开相位、拟合残差、有效性，以及独立的幅值带宽和相位交叉估计 |
| `crossings.csv` | −3 dB、−30°、−45°、−90°四项指标的交叉频率和状态，每项一行 |
| `tracking.png` / `frequency_response.png` | 时序及幅频、相频曲线 |
| `config/`、`*.log` | 本次配置副本（含测试 YAML）和节点日志；命令行覆盖后的实际参数见 `launch.json` |

`run.json` / `summary.json` 的 `axis` 标明测试轴。转动分析幅值、残差单位为 deg，
推拉分析为 mm，相位均为 deg；原始位移 CSV 为 m。
推拉新增 `ideal_displacement_m`、`command_displacement_m`、`reference_displacement_m`、
`estimated_displacement_m`、`arm_difference_m` 列，保留原有角度及左右位移列。
旧运行数据没有 `axis` 时仍按转动分析；中断后缺少 `run.json` 时从 `launch.json` 读取测试轴。

仅在恒幅测量区间，按原始采样时间对输入、输出分别进行带常数偏置的正弦最小二乘拟合。
输入使用该采样时刻之前最近一次发送的目标；相位为输出减输入，负值表示滞后。
`wheel_reference` 的采样时刻与关节反馈并非严格一致，仅用于诊断绘图。
不做会引入额外相位的平滑滤波。

频点不完整、采样不足/中断、非正弦残差超过幅值 25%，或几何一致性超阈值
（左右角度差 3°、横向残差 20 mm、轴外旋转 5°；推拉另检查左右位移差 20 mm）
时标为无效，不据此报告带宽或相位交叉频率。
这些阈值是分析质量标记，不是硬件保护阈值，也不能独立判断是否滑动。

完整有效的多频点测试，以第一个频点增益为低频基准，寻找首次下降 3 dB 的区间，
在对数频率轴插值估计带宽。首频点并非经过独立验证的直流增益，结果为本次扫频估计。
单频、未覆盖 -3 dB、无效或中断测试不会输出虚构的带宽数值。

同时独立计算相位首次向下穿过 **−30°、−45°、−90°** 的频率：

- 按频率顺序对相位连续展开，再对阈值两侧频点进行对数频率插值。
  展开依赖首频点接近低频零相位、相邻频点真实相位变化小于 180°的假设；
  若频点过稀或首频点延迟已很大，应补充低频点并加密扫频，避免相位混叠。
- 三个相位指标和 −3 dB 带宽各自独立。仅测到任意一项，就输出该项数值；
  其他项显示“未观测到交叉”，JSON 中频率为 `null`，CSV 中频率留空。
  四项都未测到仍生成报告和曲线，不据此判定性能合格或不合格。
- 如果首频点已达到某个相位阈值，标记
  `phase_threshold_already_reached_at_first_frequency`，不外推、不把后续再次穿越当成首次交叉。
  相位阈值相对于输入零相位，不减去首频点相位。
- 四项沿用相同的完整性和质量检查：测试中断、单频或存在不被当前策略接纳的无效频点时，
  不估计交叉频率。相位展开不跨越无效频点；`ignore_geometry` 仍只豁免几何一致性告警。

`summary.json` 保留 `bandwidth_hz` / `bandwidth_status`，新增 `phase_crossings`：
其中 `minus_30_deg`、`minus_45_deg`、`minus_90_deg` 分别包含
`threshold_deg`、`frequency_hz`、`status`。
`has_estimated_crossing` 表示是否至少估计出一项指标，不表示性能验收通过。
`frequency_response.csv` 增加 `phase_unwrapped_deg`，无法用于展开的点留空；
`crossings.csv` 汇总四项阈值、单位、频率和状态。
终端分别列出四项结果，频响图标出 −3 dB 交叉频率和三个相位阈值及已观测到的交叉点。
以上相位交叉频率是跟踪同步性指标，**不是开环相位裕度**。

重新分析已有数据：

```bash
/usr/bin/python3 -s tools/wheel_bandwidth/analyze.py logs/wheel_bandwidth_实际目录
```

### 忽略几何超限继续估计

如需在几何一致性未通过时观察关节反馈链路，可显式使用 `--ignore-geometry`。
该选项只允许带 `inconsistent_grasp_geometry` 告警的频点参与带宽和相位交叉计算，覆盖上述
左右角度差、横向残差、轴外旋转及推拉时的左右位移差检查；不改变 Core、运动限制或设备保护。
周期不足、采样中断、严重非正弦残差等其他问题仍会使结果无效。
原有 `valid=false` 和告警保留，`usable_for_bandwidth` 表示当前分析策略是否接纳该频点用于这四项指标。
输出文件另加 `_geometry_ignored` 后缀，不覆盖已有严格分析结果；JSON 中记录
`ignore_geometry=true`，频响图和终端也会标注。此结果只能表述为
**忽略几何一致性检查的关节反馈链路带宽估计**，不能视为实体轮盘的实测带宽。

```bash
# 已有数据无需重新运动，即可按该策略分析
/usr/bin/python3 -s tools/wheel_bandwidth/analyze.py \
    ~/aviator_logs/wheel_bandwidth_实际目录 --ignore-geometry

# 下一轮：保持 ±10° 幅值，扩展到 1 Hz；先用 --dry-run 预览
./scripts/start_wheel_bandwidth.sh --ignore-geometry \
    --amplitude-deg 10 --frequencies 0.1,0.2,0.3,0.5,0.7,1 \
    --settle-cycles 3 --measure-cycles 6 --ramp-cycles 2
```

上述扫频本身约 296 秒，另加抓握和清理时间。以本轮首频点增益 `G0` 为基准，
寻找 `G(f) <= G0 * 10^(-3/20)`（约 `0.708 * G0`）的首次交叉。
若仍提示 `minus_3db_crossing_not_observed`，说明当前范围未覆盖幅值交叉点，不能给出数值带宽；
已经观测到的相位交叉频率仍会独立输出。
先检查运动、抓握及失真情况，再分轮向更高频率扩展，保留同幅值低频基准。
找到交叉区间后，在区间内加密频点重测，脚本按对数频率插值。
若降低幅值，应在新幅值下重新测低频基准与高频响应，不能将不同幅值的数据直接拼接；
Servo/Ruckig 的限速、限加速度等可能使结果随幅值变化。

## 离线验证

```bash
cmake --build build --target aviator_wheel_bandwidth_test -j2
ctest --test-dir build -R 'wheel_bandwidth_(estimate|help)' --output-on-failure
/usr/bin/python3 -s -m unittest discover -s tools/wheel_bandwidth -p 'test_*.py' -v
```

覆盖真实 URDF 的 IK→FK 角度反算、工具变换与推拉位移、圆周角融合、±3°/±20 mm
轨迹与越界拒绝、配置覆盖及 fake-hand 预览、不规则采样下两种轴的已知增益/相位、
推拉单位及绘图、中断运行、无效数据处理、-3 dB 区间插值、相位展开与首次交叉、
四项指标独立报告、几何豁免策略及原配置不变。

手控制由独立 xCoreSDK_hand 提供；`--fake-hand` 传给该节点。生成运行配置时会将手配置路径一并转为绝对路径。
