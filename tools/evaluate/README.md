# AVIATOR 独立性能评估工具

本工具在 AVIATOR 已运行时采集资源和新增时序日志，生成离线 HTML、CSV、JSON。
Python 3.9+、Linux，无第三方 Python 依赖。不会启动、停止或重编译业务节点。

默认是轻量模式：每 2 秒采集资源、每 5 秒重新发现进程；不运行 perf、Heaptrack、
PSS 或 GUI。仅将评估器自身 nice 调低到至少 10，尝试设置自身 I/O best-effort/7；
失败会记入 summary，不修改业务进程的优先级、亲和性、看门狗或内核参数。
每个节点日志每轮最多读 64 KiB，积压会明确标记；低优先级可能造成采样延后。
轻量采集也不是零开销，需与不运行评估器的相同场景比较。

## 独立部署

开发机执行：

```bash
./scripts/deploy_evaluate.sh --package-only
./scripts/deploy_evaluate.sh --target rocos@192.168.31.100
```

第一条仅语法编译、校验、打包，不联网。产物默认在
`build/evaluate-deployment/aviator-evaluate.tar.gz`，附 SHA256。
第二条通过 SSH/SCP 上传并校验，默认安装到目标用户
`~/.local/share/aviator-evaluate/`。SSH 使用已有密钥或终端认证，不读取或保存密码。
可用 `--port` 指定 SSH 端口，`--output-dir` 指定本地打包目录。
`--prefix /绝对路径/aviator-evaluate` 指定目标安装目录，需目标用户可写。

只部署评估脚本和本文档，不调用 CMake、不重编译 AVIATOR、不安装 apt/pip 依赖。
现有安装须通过 manifest 校验才允许替换，拒绝覆盖未知目录、安装目录符号链接或已改动的包文件。
若目录内有额外文件（例如 Python 缓存、报告或手动添加的文件），升级会将旧安装完整保留到
同级 `.aviator-evaluate.saved-<唯一标识>/aviator-evaluate/`，并打印新增文件名和保留路径；
不会删除这些内容，也不会覆盖以前保留的旧安装。新安装目录仅包含已校验的包文件。
确认保留内容不再需要后，可自行清理打印出的旧安装路径；评估结果默认仍存放在另一个目录。
更新采用同文件系统目录重命名和备份恢复；中断后再次部署可恢复。

评估入口和部署入口已从主工程 CMake 安装及 `deploy_aviator.sh` 包中排除。
`--skip-build` 使用旧安装规则时，主部署打包阶段仍会剔除这两个入口。

## 使用

终端 A 保持原来的 AVIATOR 启动方式，例如：

```bash
/opt/aviator/share/aviator/scripts/start_aviator.sh --logger
```

等全部节点启动并进入要测量的场景，在终端 B 执行：

```bash
python3 ~/.local/share/aviator-evaluate/evaluate_aviator.py \
  --open
```

这是一条命令完成 300 秒轻量采集、生成 HTML 并尝试打开报告；无需 sudo。
省略 `--open` 则只生成报告路径，不启动浏览器。
在源码目录也可直接执行 `python3 scripts/evaluate_aviator.py ...`。
默认输出至 `${XDG_STATE_HOME:-~/.local/state}/aviator-evaluate/runs/<时间-唯一标识>/`。
用 `--output /可写路径/新目录` 覆盖输出位置，已有目录不会被覆盖。
`--open` 只在结束后打开报告，无桌面会话时打印路径；可复制整个结果目录到开发机查看。

```bash
# 明确指定实例和日志目录；日志须与该实例 stdout 路径相符。
python3 scripts/evaluate_aviator.py \
  --launcher-pid 12345 --log-dir /tmp/aviator-start.XXXXXXXX --duration 300

# 只采集指定节点，并每 5 秒尝试读取 PSS。
python3 scripts/evaluate_aviator.py --nodes core,logger --pss --duration 120

# 可选 CPU 热点：所有节点仍做基础监控，仅 Core 做 19 Hz、30 秒无调用栈采样。
# 在受控测试中使用；会增加运行时开销。
python3 scripts/evaluate_aviator.py --cpu --perf-sudo --open

# 明确需要调用链时，再单独启用较小的 DWARF 栈。
python3 scripts/evaluate_aviator.py --cpu --perf-sudo --perf-nodes core \
  --frequency 19 --perf-duration 20 --call-graph dwarf --stack-bytes 2048
```

`--perf-sudo` 会在采集开始前验证 sudo，需要时在同一终端输入一次密码；
只有 perf 子任务提权，评估器、结果和 GUI 由原用户管理。无人值守时仅使用已有授权，失败即报错。
不要将整个 Python 评估器放在 sudo 下运行。

CPU 模式的默认值为 `--perf-nodes core --frequency 19 --perf-duration 30 --call-graph none`。
`--duration` 控制基础采集总时长；CPU 采集时间是它与 `--perf-duration` 中较小值。
`--nodes` 限制基础监控范围，`--perf-nodes` 必须在该范围内；缺少所选节点直接报错，
不自动扩大到全体节点。perf 只附加最初选中的 PID；新增进程的继承覆盖不作保证。
默认 `--perf-max-mb 128`：每个资源采样周期检查文件软上限，达到后停止 perf，
基础采集继续；缓冲和收尾可能超过该大小。提前结束、文件限额和失败都写入报告。
不采调用栈时只能看函数自身采样热点，无法还原完整调用链。
`--call-graph fp` 需相关二进制保留帧指针；DWARF 小栈可能截断调用链。
这些设置减少采样开销，不能保证真实控制时序不受影响。

### 完全离线生成报告

部署机一条命令只保存 CSV/JSON，结束时不生成 HTML 或启动浏览器：

```bash
python3 ~/.local/share/aviator-evaluate/evaluate_aviator.py --collect-only
```

采集结束后，把终端输出的整个结果目录复制到开发机（将示例 `<本次目录>` 替换为实际名称）：

```bash
run_name=替换为本次目录名
scp -r "rocos@192.168.31.100:~/.local/state/aviator-evaluate/runs/$run_name" ./
python3 scripts/evaluate_aviator.py --report "./$run_name" --open
```

离线模式不发现进程、不连接设备、不启动 perf、不要求 sudo；只读取已有 CSV/JSON，
生成或重新生成该目录的 `report.html`，支持旧版 schema_version=1 的结果。
默认模式也只在采集结束后生成 HTML。离线处理移走解析和可视化开销，不能移走采样中断、
栈复制和写盘开销；Hotspot/Heaptrack GUI 应在开发机运行。

示例 PID 需替换。每次运行至少一个已识别业务节点；没有启动器或存在多个实例时明确报错。
脚本按启动器后代关系、实际可执行入口和 stdout 日志识别，校验 Manipulator PID 文件，
跟踪 PID 启动时间防止复用；不会把所有同名 Python 进程都加入。
启动浏览器产生的后台 Bash 会继承启动器命令行；同一祖先链、用户、进程组和命令行的
这类子 shell 自动归并到主启动器。遗留但不再拥有业务节点的浏览器 shell 不会阻挡当前实例；
确有多个独立活动实例时仍列出 PID 并要求用 `--launcher-pid` 指定。
业务节点的后代进程也纳入对应节点，但包装进程不会冒充实际业务节点。
默认不纳入复用的浏览器；CPU 图允许按进程勾选。

评估期间按 Ctrl+C，保存部分报告并停止本工具的采集任务，业务节点继续运行。
perf 使用独立采集会话附加指定 PID；提权模式另有 timeout 限时，异常收尾会标记不完整。
不要在运行中修改工具文件或把结果输出到安装目录内。

## 函数热点分析（1.2，离线执行）

basic 只统计进程资源，不能从 CPU 曲线反推出函数。先在受控场景做单节点 CPU 采样；
不自动在部署机分析。推荐先测 Monitor，再测 Core，最后按需要测试 Manipulator，分开运行：

```bash
# 部署机：一条命令采集；只附加 Monitor，19 Hz，无调用栈。
python3 ~/.local/share/aviator-evaluate/evaluate_aviator.py \
  --cpu --perf-sudo --perf-nodes monitor --duration 90 --perf-duration 90 \
  --frequency 19 --label monitor_functions --collect-only
```

采集结束后把整个结果目录复制到开发机，执行：

```bash
# 在开发机仓库根目录执行，run_name 替换为实际目录名。
run_name=替换为本次目录名
scp -r "rocos@192.168.31.100:/home/rocos/.local/state/aviator-evaluate/runs/$run_name" ./
python3 scripts/evaluate_aviator.py --report "./$run_name" --functions \
  --symbol-file build/bin/aviator_monitor --open
```

`--symbol-file` 可重复指定二进制或独立调试文件。脚本用 `readelf` 检查 `.symtab`，
并与 `perf buildid-list` 中的 build ID 比较；不匹配会拒绝加载，不能用重建后的错误版本替代。
符号缓存位于结果目录 `.perf-symbol-cache/`，不替换部署程序、不修改全局符号缓存。
只含 `.symtab` 可以恢复函数名，但没有 `.debug_line` 就不能可靠定位源码行；报告记录该区别。
系统/第三方库缺少匹配符号时保留未知项，不能把缺失部分当作零开销。

开发机 `/usr/bin/perf` 可能因本机 realtime 内核工具缺失而无法启动。离线解析可显式选择
已安装、兼容的工具；本次验证可用路径为：

```bash
python3 scripts/evaluate_aviator.py --report "./$run_name" --functions \
  --perf-bin /usr/lib/linux-tools/6.8.0-149-generic/perf \
  --symbol-file build/bin/aviator_monitor --open
```

该路径是开发机环境，不要把它写死到部署机采集命令。`--perf-bin` 也可明确选择采集工具。
`--function-comms aviator_core_ma` 可筛选旧多节点数据；它按 perf 记录的线程 comm 匹配，
不是节点名或 PID，不能保证包含被单独命名的全部线程。新测试优先只采一个节点后不加筛选。

新增文件：`functions.csv`、`functions.json`、`perf.report.txt`、`perf.report.stderr.log`、
`perf.buildids.txt`；HTML 增加事件选择、函数/模块搜索和前 100 项排名。
原始 CSV/JSON 与 perf.data 不变。重复分析会更新函数结果，失败时清除旧排名，避免误读历史成功结果。
`--functions` 必须配合 `--report`，与在线采集分离；`--symbol-file`、`--function-comms` 需配合 `--functions`。

函数 Self % 按同一事件、当前 comm 范围内的 period 加权，分母包含未知符号。
它不是整机 CPU%、调用次数、单次耗时或含子调用的累计耗时。混合 CPU 的 `cpu_atom` / `cpu_core`
事件分别统计，不直接合并。显示总样本数、未知符号占比和丢失计数；不足 100 个样本给出提示，
但超过该阈值也不等于结果已达到统计稳定。无调用栈时不能据此确定业务调用者。

函数分析退出码 `0` 表示解析完成且无已识别质量问题；`2` 表示原始采集不完整、缺失符号、
样本不足、诊断信息或分析失败，应结合 `functions.json` 查看。已有失败状态的 perf 可以用于
排查线索，但不会因解析成功被改标为完整采集。不要将带干扰的旧采样占比直接外推为正常负载。

## 报告与数据解释

- `report.html`：无公网依赖；指标选择、节点勾选、时间范围、悬停数值、摘要、事件和质量信息。
- `metrics.csv`：进程 CPU、RSS、可选 PSS、线程数、I/O、缺页及缺失原因。
- `host.csv`：主机 CPU busy、iowait、可用内存。
- `timing.csv`：采集开始后新增的 Motion/Servo timing 字段与原始时间戳、文件偏移。
- `processes.json`：节点、实际 PID、启动 ticks、可执行文件及采集覆盖。
- `events.json` / `summary.json`：模式、事件、缺失、持续时长和限制。
- CPU 模式额外保存 `perf.data` 和 `perf.stderr.log`，可用 `hotspot /路径/perf.data` 打开。

CPU 100% 表示一个逻辑核，多线程进程可大于 100%。主机 CPU 是整机口径，busy 含 steal，
不含 idle 和 iowait。RSS 不能直接相加当作实际总内存。I/O 指进程存储字节计数器差值，
不等于持久化完成；没有物理盘时延统计。PSS、root 进程 I/O 等无权限时留空并标记。
CPU 均值按有效采样时间加权。第一条计数器样本没有差分 CPU/I/O，不能算作零。

图表按时间桶保留首尾、极值和缺口，完整 CSV 不降采样。缺失数据断线显示。
时序图横轴是日志被读取的时刻，不能用它推导微秒级因果关系。
`rt_max_gap_ms/rt_max_work_us` 是使能以来累计最大值，可能含评估开始前历史。
`max_target_error_rad` 是快照跨关节最大值；`plan_mean_us/plan_max_us` 只覆盖规划 step。
多进程共写一份节点日志时 PID 不作猜测。没有新时序日志时明确显示无样本。

本版没有逐周期 P99/P99.9、端到端消息延迟、泄漏判定、Heaptrack 自动附加、
off-CPU 自动化或基线自动比较，也不会自动收集业务配置和源码身份。
对比前自行固定并归档版本、构建参数、配置、输入、相机参数、编码器、调度和频率策略。
可先 basic 运行建立基线，再在相同负载的单独运行中采集 perf；不把 profiler 开销当作正常控制时延。

退出码：`0` 完整采集；`2` 有部分结果（中断、字段权限不足、profiler 失败等）；
`1` 无法开始或无法完成归档。参数错误由 argparse 返回 `2`，这时没有报告目录。
节点正常出现、无动作时没有时序样本本身不自动判为采集故障；报告仍显示相关限制。

## perf 和符号

本机原环境的 realtime 内核对应 perf 缺失。先在采集机检查 `perf --version`。
Ubuntu 可尝试安装匹配当前内核的 tools 包：

```bash
sudo apt install "linux-tools-$(uname -r)"
```

对应包是否可用取决于系统软件源，工具不会更改内核、perf_event_paranoid 或 ptrace 设置。
有 `/usr/bin/perf` 不代表它可以执行。CPU 模式使用 `cycles:u`，默认不采调用栈；事件不可用时
明确失败并保留基础结果，不静默改用不同计数口径。
开发机安装 Hotspot 后分析数据；HTML 不自动运行本地 shell 命令。

主部署会 strip 二进制。本工具不重编译/替换主程序，也不能恢复被剥离的调试信息。
要定位源码，需另备匹配 build ID 的未剥离二进制、库或独立调试符号，并在 Hotspot 配置路径。
缺少符号时 CPU、内存、I/O 仍可测量，但函数/行号分析受限。

## 开发验证

```bash
python3 -B tests/evaluate_test.py
python3 -B tests/evaluate_deploy_test.py
./scripts/deploy_evaluate.sh --package-only
```

测试使用临时启动器和纯软件负载，不连接机械臂、串口或相机，不执行远端部署。
