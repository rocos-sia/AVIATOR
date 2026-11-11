# start_wheel_bandwidth.sh 部署机使用说明

## 1. 登录部署机并找到入口

可以在工控机桌面打开终端，也可以从开发机登录：

```bash
ssh -t rocos@192.168.31.100
```

登录后进入运行资源目录：

```bash
cd /opt/aviator/share/aviator
./scripts/start_wheel_bandwidth.sh --help
./scripts/start_wheel_bandwidth.sh --dry-run
```

也可以从任意目录使用绝对路径预览：

```bash
/opt/aviator/share/aviator/scripts/start_wheel_bandwidth.sh --dry-run
```

`--dry-run` 只校验配置和运动参数并打印命令，不启动设备、不写运行文件、不请求 sudo。

## 2. 部署目录与运行环境

| 路径 | 用途 |
| --- | --- |
| `/opt/aviator/share/aviator/scripts/start_wheel_bandwidth.sh` | 启动入口 |
| `/opt/aviator/bin/` | 已编译的 Core、Bus、Manipulator、Monitor 等程序 |
| `/opt/aviator/cache/python/bin/python` | 部署包 Python 环境，供启动、手节点及分析使用 |
| `/opt/aviator/share/aviator/config/wheel_bandwidth.yaml` | 扫频轴、幅值、频点及周期数 |
| `/opt/aviator/share/aviator/config/system.yaml` | 真机系统配置 |
| `/opt/aviator/share/aviator/config/robot.yaml` | 默认机器人配置，包含初始轮盘位形 |
| `/opt/aviator/share/aviator/config/rh56ftp_hand.yaml` | 手部连接与控制配置 |
| `/opt/aviator/share/aviator/tools/wheel_bandwidth/analyze.py` | 离线分析入口 |
| `/opt/aviator-data/logs/` | 持久化运行日志目录 |

启动脚本通过同目录的 `runtime_env.sh` 自动选择 `/opt/aviator/bin` 和部署包 Python，正常使用不需要手动激活环境。Python 依赖随部署安装，包含 PyYAML、NumPy、Matplotlib，以及真实手所需的 pyzmq、pymodbus。

若当前终端曾设置开发环境变量，先检查是否误指向开发机路径：

```bash
printenv AVIATOR_BIN BANDWIDTH_PYTHON HAND_PYTHON MONITOR_BIN
```

只有发现这些变量残留了错误路径时，才在当前终端清除，让脚本使用部署默认值：

```bash
unset AVIATOR_BIN BANDWIDTH_PYTHON HAND_PYTHON MONITOR_BIN
```

`--dry-run` 输出中的节点路径应指向 `/opt/aviator/bin/`，手节点解释器应指向 `/opt/aviator/cache/python/bin/python`。若缺少脚本、程序或依赖，应在开发机更新部署包；部署机不执行 CMake 构建。部署流程见 [部署说明](../deploy/README.md)。

## 3. 核对现场配置和设备

### 3.1 现场配置优先

部署更新会保留工控机已有的配置文件。因此，开发机 YAML 中的值不一定等于部署机实际使用值。先在部署机查看：

```bash
cat /opt/aviator/share/aviator/config/wheel_bandwidth.yaml
```

重点核对：

| 字段 | 含义 |
| --- | --- |
| `axis` | `rotation` 转动或 `translation` 推拉 |
| `amplitude_deg` / `amplitude_mm` | 对应轴的单边幅值 |
| `frequencies_hz` | 严格递增的扫频列表 |
| `settle_cycles` / `measure_cycles` / `ramp_cycles` | 等待、测量、增幅和减幅周期 |
| `fake_hand` | `true` 为真实机械臂 + 模拟手，`false` 为真实手 |
| `ignore_geometry` | 是否允许几何一致性超限频点参与分析 |

不带参数启动会直接采用这份现场配置。下方常用命令显式指定测试轴、幅值、频点、周期、手模式和分析策略，避免继承不符合本轮要求的设置。

命令行参数只覆盖本次运行，不改写原 YAML。若需保存专用测试配置，可复制到用户目录，再用 `--sweep-config` 指定，见第 4.6 节。

### 3.2 真机准备与测量边界

1. 退出其他 AVIATOR 控制实例和正在运行的测试；如现场有自动重启服务，应先停止对应服务。
2. 确认机械臂和真实手网络可达，现场配置中的地址正确。
3. 核对机器人配置的 `wheel_initial`，并把实际轮盘放到该声明位形；程序不启动相机，也不会测量或修正摆放误差。
4. 使用普通用户运行，**不要在整个脚本前加 `sudo`**。本带宽脚本会调用 `sudo -v`，需要时在终端输入当前账号密码，为 Manipulator 单独认证。

程序自动使能、接近抓握、锁定，回到测试中心后发送正弦目标；每次只测试转动或推拉一个轴。转动中心为 0°，推拉中心取 `wheel_initial.displacement`。例如该值为 `-0.085 m` 时，±20 mm 推拉对应 −105～−65 mm；现场值不同时应重新计算。

`--fake-hand` 只模拟手，机械臂仍是真机；完整仿真使用 `--simulation`。反馈来自双臂实测关节角及正运动学估计，反映 Servo / Ruckig / IK / 机械臂跟踪链路响应，依赖抓握无滑动及标定准确等假设。它不是相机测得的实体轮盘真值，相位交叉频率也不是开环相位裕度。

## 4. 部署机常用命令

以下相对路径命令均在 `/opt/aviator/share/aviator` 执行。前两组真机命令先预览，核对参数后移除末尾的 `--dry-run` 才会启动运动。

### 4.1 查看帮助与预览

```bash
./scripts/start_wheel_bandwidth.sh --help
./scripts/start_wheel_bandwidth.sh --dry-run
```

`--dry-run` 校验配置和运动参数，打印节点命令、测试范围和输出目录，不写配置、不启动节点、不请求 sudo。它在检查可执行文件、NumPy/Matplotlib、手节点配置、互斥锁及端口之前返回，因此预览成功不代表运行环境已全部就绪。

### 4.2 真机 + 真实手：转动扫频

先预览，核对后执行同一条命令并移除 `--dry-run`：

```bash
./scripts/start_wheel_bandwidth.sh \
  --axis rotation --amplitude-deg 3 \
  --frequencies 0.1,0.2,0.3,0.5,0.7,1.0 \
  --settle-cycles 2 --measure-cycles 5 --ramp-cycles 1 \
  --no-fake-hand --no-ignore-geometry --dry-run
```

转动中心为 0°，推拉保持 `wheel_initial.displacement`。上述扫频本身约 205 秒，另加抓握、回中和退出时间。

默认启动 Monitor，并在可用时打开本机 Chrome。桌面终端可直接使用；SSH 会话推荐采用第 4.4 节的无界面命令。

### 4.3 真机 + 模拟手：推拉扫频

```bash
./scripts/start_wheel_bandwidth.sh \
  --axis translation --amplitude-mm 20 \
  --frequencies 0.1,0.2,0.3,0.5 \
  --settle-cycles 2 --measure-cycles 5 --ramp-cycles 1 \
  --fake-hand --no-ignore-geometry --dry-run
```

核对后移除 `--dry-run` 执行。推拉中心为初始位移，角度保持 0°；模拟手模式本身不能证明真实轮盘已被可靠抓住。

### 4.4 SSH 运行真机：不启动 Monitor

登录部署机后执行下面的预览命令，核对后移除 `--dry-run` 运行：

```bash
/opt/aviator/share/aviator/scripts/start_wheel_bandwidth.sh \
  --axis rotation --amplitude-deg 3 \
  --frequencies 0.1,0.2,0.3,0.5,0.7,1.0 \
  --settle-cycles 2 --measure-cycles 5 --ramp-cycles 1 \
  --no-fake-hand --no-ignore-geometry --no-monitor --dry-run
```

保持 SSH 会话和终端打开，等待测试完成；需要中止时按 `Ctrl+C`。无需通过 SSH 获取桌面显示权限，也不需要启动主程序或相机随动脚本，本脚本自行启动所需节点。

### 4.5 无窗口仿真：单频检查

部署包包含 `simulation` 时，可以执行：

```bash
/opt/aviator/share/aviator/scripts/start_wheel_bandwidth.sh \
  --simulation --headless --no-monitor --no-fake-hand \
  --axis rotation --amplitude-deg 3 --frequencies 0.5 \
  --settle-cycles 1 --measure-cycles 2 --ramp-cycles 1 \
  --no-ignore-geometry
```

显式添加 `--no-fake-hand` 可以覆盖现场 YAML 的 `fake_hand: true`，避免与仿真模式冲突。单频可以检查响应，但不能估计带宽或相位交叉频率。

### 4.6 保存专用测试参数和指定输出

在部署机上复制测试参数到用户目录并编辑：

```bash
mkdir -p "$HOME/aviator-tests"
cp /opt/aviator/share/aviator/config/wheel_bandwidth.yaml \
  "$HOME/aviator-tests/wheel_bandwidth.yaml"
nano "$HOME/aviator-tests/wheel_bandwidth.yaml"
```

使用该配置预览，核对后移除 `--dry-run`：

```bash
/opt/aviator/share/aviator/scripts/start_wheel_bandwidth.sh \
  --sweep-config "$HOME/aviator-tests/wheel_bandwidth.yaml" \
  --config /opt/aviator/share/aviator/config/system.yaml \
  --output /opt/aviator-data/logs/wheel_test_001 \
  --no-monitor --dry-run
```

`--output` 必须是尚不存在的新目录，每轮更换名称；省略时自动生成带时间戳的目录。要额外记录 MCAP，在启动命令中加 `--logger`；CSV 默认记录。

如直接修改安装目录内的现场配置，它会影响发布校验值。确认修改后，按 [部署说明](../deploy/README.md) 的校验清单刷新流程处理。使用用户目录中的测试配置或命令行覆盖，无需修改安装配置。

## 5. 参数速查

命令行参数优先于测试 YAML；未指定的测试参数继承 YAML。`--axis` 不会根据幅值单位自动切换。

| 参数 | 说明 / 限制 |
| --- | --- |
| `-h` / `--help` | 显示帮助并退出 |
| `--axis rotation` / `translation` | 转动 / 推拉，每次一个轴 |
| `--amplitude-deg N` | 转动单边幅值，`0 < N ≤ 10`°，只能用于转动 |
| `--amplitude-mm N` | 推拉单边幅值，须为有限正数，中心 ± 幅值均须在 `[-170, 0] mm` 内 |
| `--frequencies f1,f2,...` | 非空、严格递增，范围 `0.01～5 Hz`，用英文逗号连接 |
| `--settle-cycles N` | 每频点增幅后等待周期，整数 `0～100` |
| `--measure-cycles N` | 每频点恒幅测量周期，整数 `2～100` |
| `--ramp-cycles N` | 每频点增幅和减幅各自的周期数，整数 `1～100` |
| `--fake-hand` / `--no-fake-hand` | 真机模式下选择模拟手 / 真实手，覆盖 YAML |
| `--simulation` | 使用仿真设备，与有效的 `fake_hand: true` 冲突 |
| `--headless` | 仿真不打开窗口，只能与 `--simulation` 同用 |
| `--no-monitor` | 不启动 Monitor 和浏览器 |
| `--logger` | 额外启动 Logger，保存 `recording.mcap` |
| `--ignore-geometry` / `--no-ignore-geometry` | 忽略 / 严格检查分析中的几何一致性，覆盖 YAML |
| `--config PATH` | 系统配置；真机默认 `config/system.yaml`，仿真默认 `config/system-simulation.yaml` |
| `--sweep-config PATH` | 测试参数配置；默认项目中的 `config/wheel_bandwidth.yaml` |
| `--output PATH` | 指定新的输出目录 |
| `--dry-run` | 校验参数并预览命令 |

两种幅值参数互斥。测试 YAML 必须包含 `axis`、`amplitude_deg`、`amplitude_mm`、`frequencies_hz`、`settle_cycles`、`measure_cycles`、`ramp_cycles`、`fake_hand`、`ignore_geometry` 全部 9 个字段，不接受额外字段；其中两种幅值都必须合法，`amplitude_deg` 范围 `(0, 10]`，`amplitude_mm` 范围 `(0, 85]`，布尔字段使用 `true` / `false`。

系统、机器人和默认测试配置的定位不依赖当前工作目录；用户传入的相对路径按调用时的工作目录解析。机器人资源路径按机器人配置文件所在目录解析。

`servo_period` 必须在 `0.001～0.05 s` 内，并满足：

```text
最高频率 × max(servo_period, 0.01) ≤ 0.05
```

该条件保证每个正弦周期至少有 20 个目标采样点。当现场配置为 `servo_period = 0.01 s` 时，频率上限为 5 Hz。

每频点按“增幅 → 等待 → 测量 → 减幅”执行，扫频时间可估算为：

```text
总扫频时间 ≈ Σ [(2 × ramp_cycles + settle_cycles + measure_cycles) / 频率]
```

## 6. 运行监控与停止

启动时终端打印实际测试参数、运动范围、输出路径及各节点命令。运行进度写入输出目录的 `core.log`，可在另一个终端查看：

```bash
tail -f /实际输出目录/core.log
```

默认 Monitor 地址为 `http://127.0.0.1:8081`；检测到 `google-chrome-stable` 时会自动打开浏览器，否则可手动访问。

正常结束先回中，再解锁、下使能并清理节点。按 `Ctrl+C` 时先通知 Core 减速、解锁和下使能，最多等待 15 秒，再清理其他节点；中断或故障不会强行回中。正常完成通常退出码为 `0`，Ctrl+C 为 `130`，启动或运行异常返回非零。

输出目录中已有 `samples.csv` 时，脚本清理节点后会尝试自动分析。中断数据仍可能生成报告，但不报告完整有效的带宽估计。

## 7. 输出文件与结果解读

标准部署将 `/opt/aviator/share/aviator/logs` 链接到 `/opt/aviator-data/logs`，因此默认结果实际保存在 **`/opt/aviator-data/logs/wheel_bandwidth_<北京时间>/`**，更新部署后继续保留。默认日志目录不可写时，脚本改用 `~/aviator_logs/`；显式指定 `--output` 时不会自动换路径，也不会覆盖已有目录。始终以启动终端打印的实际路径为准。

| 文件 | 内容 |
| --- | --- |
| `commands.csv` | 实际发送目标和单调时间 |
| `samples.csv` | 实测关节角、目标、执行参考、估计角度/位移及几何残差 |
| `launch.json` | 配置来源及命令行覆盖后的实际参数 |
| `run.json` | 运行参数和完成状态 |
| `summary.json` | 带宽、相位交叉频率、状态及各频点结果 |
| `frequency_response.csv` | 各频点增益、相位、展开相位、残差和有效性 |
| `crossings.csv` | −3 dB、−30°、−45°、−90°四项交叉指标 |
| `tracking.png` | 目标与反馈时序图 |
| `frequency_response.png` | 幅频、相频曲线 |
| `config/` | 本次系统、机器人和测试配置，以及抓握和姿态快照 |
| `*.log` | 各节点日志 |
| `recording.mcap` | 仅 `--logger` 启用时生成 |

原始配置不会被改写。运行副本将抓取来源设为 `wheel_initial`、关闭 `xcore_sdk_hand.enabled`，并解析资源路径。保存的测试 YAML 是源文件副本，**最终生效参数以 `launch.json` 为准**。

启用 `ignore_geometry` 时，三个分析报告和两张图的文件名都加 `_geometry_ignored` 后缀，例如 `summary_geometry_ignored.json`、`frequency_response_geometry_ignored.png`；原始数据文件名不变。是否启用以现场 YAML 和本轮命令行参数为准。

结果阅读要点：

- 转动分析幅值与残差单位为 deg，推拉为 mm；原始 CSV 位移单位为 m，相位单位为 deg，负相位表示输出滞后。
- −3 dB 带宽以本轮第一个频点的增益为基准，寻找首次下降 3 dB 的区间，并在对数频率轴插值；首频点不是独立验证过的直流增益。
- −30°、−45°、−90°相位交叉频率分别计算，与幅值带宽独立。未覆盖某项交叉时，该项频率为 `null` / 空值，不表示测试失败或性能合格。
- 单频、测试中断、采样不足或存在不被分析策略接纳的无效频点时，不估计交叉频率。首频点已达到相位阈值时也不会外推首次交叉。
- `--ignore-geometry` 仅豁免几何一致性超限，不豁免测量不完整或严重非正弦残差，不修改设备保护；告警和原始 `valid=false` 仍保留。这类结果只能表述为忽略几何一致性检查的关节反馈链路估计。


## 8. 常见问题

| 现象 | 处理方法 |
| --- | --- |
| 提示去掉 sudo | 使用普通用户直接运行，等待脚本为 Manipulator 单独认证 |
| `--fake-hand` 与 `--simulation` 冲突 | 仿真命令补充 `--no-fake-hand`，覆盖现场 YAML |
| `--amplitude-deg` 与推拉轴冲突 | 同时指定 `--axis rotation`；推拉使用 `--amplitude-mm` |
| 缺少可执行文件 | 核对 `/opt/aviator/bin` 和环境变量；缺少程序时从开发机更新部署包 |
| Python 模块找不到 | 确认解释器为 `/opt/aviator/cache/python/bin/python`；部署环境缺依赖时从开发机修复并更新部署包 |
| 输出目录已存在 / 不可写 | 换用当前用户可写、尚不存在的 `--output` 目录 |
| 互斥锁或端口占用 | 退出其他 AVIATOR 控制实例；Monitor 端口冲突可关闭占用程序或加 `--no-monitor` |
| 频率或周期参数不合法 | 检查严格递增频率、周期整数范围，以及 `servo_period` 采样约束 |
| 推拉范围越界 | 按初始位移重新计算中心 ± 幅值，完整范围必须在 `[-170, 0] mm` 内 |
| 节点启动后退出 | 根据终端提示查看输出目录中的对应节点日志 |
| 未观测到 −3 dB 交叉 | 当前频率范围未覆盖该交叉；先检查抓握与失真，再保持同幅值低频基准扩展或加密频点 |
| 几何告警导致指标无效 | 检查抓握、滑动和标定；如只研究关节反馈链路，可离线显式使用 `--ignore-geometry` |

本文路径和部署行为依据仓库安装规则整理，未连接部署机进行现场运行验证。详细测量模型、分析质量阈值和离线测试入口见 [wheel_bandwidth README](../tools/wheel_bandwidth/README.md)。
