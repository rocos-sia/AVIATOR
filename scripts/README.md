# 项目配置脚本

## 工控机部署

在开发机执行 `./scripts/deploy_aviator.sh`，本机编译后通过 SSH 部署到
`rocos@192.168.11.100:/opt/aviator`。全部启动脚本及 485 示例随包安装，
部署不启动设备。固定目录、程序替换、现场配置保留和失败恢复见 [部署说明](../deploy/README.md)。

## MuJoCo STL 拆分

[split_stl.py](split_stl.py) 使用 Python 标准库将二进制 STL 按每件最多 200,000 面
无损拆分，以满足 MuJoCo 的 STL 加载限制。保留面顺序、坐标、法线和属性字节，
拒绝不完整文件及覆盖已有输出，不修改源文件。

```powershell
python scripts\split_stl.py models\meshes\Cessna\aircraft_visual.STL models\meshes\Cessna\mjcf
```

仓库已包含生成的两件显示网格，正常使用无需重复运行。更新源模型时，先将旧输出
移至备份目录，再重新生成，并运行 `python tests\mjcf_urdf_test.py` 核对一致性。

## 一键启动

按 [启动流程](../docs/启动流程.txt) 依次启动 Bus、光电节点、Manipulator（sudo）、
独立手节点、Gateway、Managed Core、Camera、Monitor，打开全屏 Chrome；指定 `--logger` 时最后启动 Logger：

```bash
./scripts/start_aviator.sh --dry-run
./scripts/start_aviator.sh
./scripts/start_aviator.sh --logger  # 开启日志记录
./scripts/start_aviator.sh --fake-hand  # 真实机械臂和相机 + 模拟 RH56FTP 双手
./scripts/start_aviator.sh --tts  # 已准备音频包时开启离线语音提醒
```

真机模式（含 `--fake-hand`、`--rs422`）自动启动 `aviator_photoelectric`，默认读取
`config/photoelectric.yaml`（`/dev/ttyUSB0`，38400，X1）。首次使用先构建
`cmake --build build --target aviator_photoelectric -j2`，也可用 `AVIATOR_BIN` 指定整套二进制目录。
`PHOTOELECTRIC_CONFIG` 可指定其他光电配置路径；相对路径以项目根目录（安装包中为
`share/aviator`）为基准。发布地址和发布者标识须与 system 配置一致。
启动前检查光电二进制、配置文件并执行 `--check-config`；校验失败时不启动设备。
Core 收到新鲜有效的 X1=1 才允许初始化；启动间隔不代替信号检查。
光电进程属于必需节点，退出时停止本次会话；正常清理先等待 Core，再停止光电节点和总线。
光电日志写入 `photoelectric.log`；重定向日志时不打印终端动态刷新行。
不要再另外启动采集进程。`--simulation` 使用关闭光电联锁的仿真配置，不启动或检查串口节点。

上述联锁仅接入 Managed Core。`start_camera_servo.sh`、`start_wheel_bandwidth.sh`
和 `start_grasp_calibration_ui.sh` 使用独立控制入口，没有该 X1 急停联锁，也不启动光电节点。

脚本可从任意目录调用。启动前检查文件，沿用脚本内置的 sudo 密码，通过标准输入提交，无需手动输入；
Manipulator 在 sudo 内另建独立会话，将实际进程组号记录到本次日志目录的 `manipulator.pid`；停止时同时清理实际节点组和 sudo 包装组，不依赖 sudo 缓存，
各节点日志写入输出的
`/tmp/aviator-start.*` 目录；保持终端打开，Ctrl+C 停止本次启动的所有节点。
控制节点退出会触发其余节点停止，浏览器保留打开；可选 TTS 节点退出只报告日志，不停止控制节点。
锁阻止该脚本重复运行；使用前应先停止手动启动的节点。
关闭终端（SIGHUP）也会触发节点清理。清理期间重复 Ctrl+C、TERM 或 HUP 不会打断清理：先等待 Core（最多 15 秒），再等待手节点（最多 8 秒），最后停止其余节点；超时执行 SIGKILL，并回收已退出的直接子进程。清理后若仍有活动进程，会列出本次进程的 PID、进程组和状态，不无限等待。如果提示启动锁被占用，可能是启动脚本或上次异常退出后
残留的节点仍在运行；先停止这些进程，不要删除锁文件绕过检查。进程退出后锁自动释放。
启动顺序之间默认等待 2 秒（不代表设备已就绪），Monitor HTTP 就绪后打开浏览器。
Manipulator（或 Simulation）启动后额外等待本次日志中的服务 `READY` 标记，之后才启动后续节点；
真机手节点也在 Manipulator 就绪后启动。默认最多等待 60 秒，可通过 `DEVICE_READY_TIMEOUT`
设为 1..3600 的整数秒数。超时会显示设备日志末尾并清理本次进程，不启动 Core。
若设备一直未就绪，应检查对应日志及控制器 SDK 通信；ping 通只代表 IP 可达。
相机仍使用 `--show`，因此需要桌面显示环境。
默认不开启 Logger，也不启用相机图像录制通道；各节点的运行诊断日志仍写入临时日志目录。
真机或 `--simulation` 模式均可添加 `--logger` 开启记录。
启用时 Logger 使用 `config/recording.yaml`，录制输出为仓库根目录的
`recording_年月日_时分秒_纳秒.mcap`（北京时间，按 Logger 启动时间命名）；
其运行日志保存在上述日志目录的 `logger.log`。
可设置 `AVIATOR_RECORDING_DIR=/绝对路径` 将 MCAP 写到用户可写目录；脚本在启动任何节点前
检查并创建录制目录。未设置时保持原有 `$ROOT/logs`。`--dry-run` 不创建录制目录。

### GNOME 桌面启动器

`/usr/bin/python3 -B tools/aviator_launcher/main.py --show` 打开原生配置面板；
安装包使用 `<prefix>/bin/aviator_launcher`，通过同包脚本启动。
支持托盘驻留、选项保存、命令预览、运行日志和有序停止。打开工具不自动启动节点。
依赖、安装与运行说明见 [桌面启动器](../tools/aviator_launcher/README.md)。

相机默认 Python 环境为 `~/miniconda3/envs/apriltag_realsense/bin/python`，Monitor 使用
`build/bin/aviator_monitor`。可通过 `CONDA_ROOT`、`CAMERA_PYTHON`、`MONITOR_BIN`、
`START_DELAY` 环境变量覆盖。手控制由独立 xCoreSDK_hand 节点提供，单独写入 hand.log；使用构建时选择的 Python，需要 PyYAML、pyzmq，不需要 pymodbus。
`--fake-hand` 传给 xCoreSDK_hand，在独立进程内模拟双手，其余机械臂和相机流程照常执行；
也可在 system.yaml 的 `xcore_sdk_hand.fake_hand` 中设置。不能与 `--simulation` 同时使用，
因为整机仿真已经提供手反馈。
`/tmp/aviator_session.uuid` 不存在或为空时生成 UUID，否则沿用现有值。
`--dry-run` 只打印启动命令，不启动节点、不请求 sudo、不写文件。

`--tts` 在 Core 前启动 `aviator_tts`，真机、模拟手与整机仿真均支持，默认关闭。
先按 [节点说明](../nodes/aviator_tts/README.md) 离线生成音频包，构建所选 `AVIATOR_BIN`
中的 `aviator_tts`。脚本在启动设备前检查 `aplay`、配置和音频，日志写入 `tts.log`。
`TTS_CONFIG` 默认指向运行目录的 `config/tts.yaml`；总线地址使用所选 `system.yaml`
或 `system-simulation.yaml`。TTS 有独立的首次状态宽限时间，音频设备故障不阻塞控制。
相机随动入口是 Direct Core，不能使用这一整机状态提醒，因此相机随动脚本不增加 `--tts`。

## 预生成语音

[generate_tts.sh](generate_tts.sh) 使用项目 `models/tts/` 内的 Matcha Baker 和 Vocos，
按 `config/tts.yaml` 批量生成音频并自动检查文案、WAV 格式及 SHA256。
默认输出到 `assets/tts/zh_CN/`，不启动机器人或播放声音。

```bash
# 首次准备 Python 环境；脚本自身不安装依赖或下载模型
python3 -m venv .cache/tts-venv
.cache/tts-venv/bin/pip install -r tools/tts/requirements.txt
./scripts/generate_tts.sh --dry-run
./scripts/generate_tts.sh
./scripts/generate_tts.sh --check
./scripts/generate_tts.sh --speed 0.9 --output /tmp/aviator-tts-preview
```

默认优先使用项目 `.cache/tts-venv/bin/python`，否则使用 `python3`；`TTS_PYTHON` 可指定
其他解释器。支持 `--config`、`--output`、`--threads`、`--speed`，以及用于更换模型的
`--model-dir`、`--vocoder`、`--model-source`。模型依赖准备好后可以完全离线生成。
可从任意目录调用，默认路径定位到项目；显式相对路径以调用目录为准。
全部配置与播放命令见 [aviator_tts README](../nodes/aviator_tts/README.md)。

## 相机随动测试

独立脚本 `start_camera_servo.sh` 启动相机随动测试，原有启动脚本保持不变：

```bash
./scripts/start_camera_servo.sh --dry-run       # 预览启动命令
./scripts/start_camera_servo.sh --fake-hand     # 真实机械臂、相机 + 模拟手
./scripts/start_camera_servo.sh                 # 真实机械臂、相机和手
./scripts/start_camera_servo.sh --fake-hand --logger  # 同时记录 MCAP
```

脚本从任意目录调用均可，使用 `config/system.yaml`，依次启动 Bus、Manipulator
（sudo，含 RH56FTP）、相机、Monitor 和 Chrome，最后启动 `aviator_core_camera_servo`；
不需要 Gateway。Core 会自动使能、接近并锁定把手，再根据 `cockpit` 相机的
`steering_wheel` 在关节阻抗模式下随动。角度取 `-theta_rad`，位移取
`-translation_along_axis_m - 0.085` 并限幅到 `[-0.170, 0]` m。
启动前先停止手动运行的节点；此脚本与 `start_aviator.sh` 使用同一互斥锁。

默认二进制目录为 `build/release/bin`（可通过 `AVIATOR_BIN` 覆盖），Monitor 默认
也使用此目录。Python 环境同上，可设置 `CONDA_ROOT`、
`CAMERA_PYTHON`、`MONITOR_BIN` 和 `START_DELAY`（默认 2 秒）。
`CAMERA_TIMEOUT_MS` 控制相机观测有效期，默认 200 ms，范围为 20–1000 ms。
相机带 `--show --print-pose`，需要桌面显示环境。sudo 认证沿用原脚本的标准输入方式。

日志保存在输出的 `/tmp/aviator-camera-servo.*` 目录，每次生成独立 session 并保存在
该目录，不改写 `/tmp/aviator_session.uuid`。默认不录制图像；`--logger` 会在相机前
启动 Logger，并将记录保存到 `logs/camera_servo_时间.mcap`。
保持终端打开；Ctrl+C 先停止 Core，最多等待 15 秒完成减速、解锁和下使能，再停止
其他节点；与主启动脚本相同，清理期间重复信号不会中断退出，sudo 内的实际机械臂进程组也会被清理。`--dry-run` 仅预览，不启动节点、不请求 sudo、不写文件；`--help` 查看完整用法。

## 轮盘转动 / 推拉带宽测试

`start_wheel_bandwidth.sh` 从 [`config/wheel_bandwidth.yaml`](../config/wheel_bandwidth.yaml)
读取测试轴、幅值、扫频频点、等待/测量/过渡周期、`fake_hand` 和分析选项。
默认转动 ±3°；将 `axis` 改为 `translation` 可测试 ±20 mm 推拉。
推拉中心取 `robot.yaml` 的 `wheel_initial.displacement`，当前范围为 −105～−65 mm。
设 `fake_hand: true` 可使用真实机械臂搭配模拟手。

```bash
./scripts/start_wheel_bandwidth.sh --dry-run  # 检查配置并预览命令
./scripts/start_wheel_bandwidth.sh --fake-hand
./scripts/start_wheel_bandwidth.sh --axis translation --fake-hand
# 临时覆盖配置中的幅值或频点
./scripts/start_wheel_bandwidth.sh --amplitude-deg 3 --frequencies 0.1,0.2 --fake-hand
```

命令行参数优先；`--sweep-config` 选择其他测试 YAML，`--config` 选择系统 YAML。
`--no-fake-hand` 可覆盖配置中的模拟手设置；`--fake-hand` 不可与 `--simulation` 同用。
测试结束后独立报告 −3 dB 带宽及 −30°、−45°、−90°相位交叉频率；
任意一项测到交叉即可输出该项，其余标注未观测到，四项均未观测到也保留报告。
结果见 `summary.json`、`crossings.csv` 和频响图；启用 `ignore_geometry` 时文件名带
`_geometry_ignored` 后缀。已有数据可重新分析得到新增指标。
修改测试程序后需重新构建 `aviator_core_wheel_bandwidth`。
构建、输出及分析说明见 [wheel_bandwidth README](../tools/wheel_bandwidth/README.md)。

## 方向盘两点标定界面

```bash
./scripts/start_steering_wheel_calibration_ui.sh --dry-run  # 只预览启动命令
./scripts/start_steering_wheel_calibration_ui.sh            # 启动 Bus、相机和标定窗口
```

先停止已有 Bus 和 Camera。保持方向盘零位，点击“采集第一个点（零位）”；转动到第二个
稳定位置后点击“采集第二个点并保存”。每点默认平均 20 帧，保存前自动备份已有标定。
界面实时显示相对零位的角度（度）和沿轴纵深（mm），无效或过期数据清空读数。
关闭窗口 / Ctrl+C 清理本次进程。UI 使用系统 `python3`（Tkinter、NumPy、PyYAML、pyzmq），
相机使用 `apriltag_realsense` 环境；支持 `UI_PYTHON`、`CAMERA_PYTHON`、`AVIATOR_BIN`。
完整参数、采样规则和运行日志见 [方向盘标定工具](../tools/steering_wheel_calibration/README.md#一键图形界面)。

## 静态抓握校准界面

```bash
cmake --build build --target aviator_grasp_tool_session aviator_bus -j2
./scripts/start_grasp_calibration_ui.sh --dry-run  # 打开模拟设备界面，禁止写回
./scripts/start_grasp_calibration_ui.sh            # 打开真机界面，点击准备后才连接
```

界面管理自己的 Bus、机械手和相机节点，支持左右手开合、双臂关节空间自由拖动、
相机或手动轮盘输入，以及静态校准预览和备份写回。默认打开 `http://127.0.0.1:8766`。
准备前退出已有控制节点；准备会张开双手，结束会话也会请求结束拖动并张开双手。
解释器、配置与操作说明见 [抓握校准工具](../tools/grasp_tool_calibration/README.md)。
此脚本的 `--dry-run` 会运行可交互模拟界面，与其他启动脚本的命令预览不同。

## 安装 apt 依赖（Ubuntu 22.04）

```bash
# 一键安装主工程全部 apt 依赖和开发工具，普通用户执行时会调用 sudo。
./scripts/install_dependencies.sh

# 预览命令，不运行 sudo、不下载、不修改系统。
./scripts/install_dependencies.sh --dry-run

# 加装独立 Foxglove 示例的 OpenCV、Protobuf 和 protoc。
./scripts/install_dependencies.sh --with-examples
```

默认包含 GCC/G++、CMake、Ninja、pkg-config、Python 3、clangd、clang-format、
clang-tidy、GDB，以及 Eigen、Boost、NLopt C/C++、URDF、Assimp、OctoMap、YAML、
XML、ZeroMQ、JSON、GLFW/OpenGL/EGL 和 MuJoCo 所需系统库。
另外包含离线提醒播放所需 `alsa-utils` 和配置/音频生成工具所需 `python3-yaml`；
sherpa-onnx、numpy 和模型在独立生成环境中准备，不自动安装到系统 Python。
`libnlopt-cxx-dev` 提供 PIN-IK 必需的 `nlopt.hpp`，仅安装 `libnlopt-dev` 不够。

脚本执行 `apt-get update` 和依赖安装，允许重复运行；不会卸载包、添加 PPA 或修改软件源。
需启用 Ubuntu jammy universe。当前仅支持项目已验证的 Ubuntu 22.04。
仓库中的 Coal、Pinocchio、PIN-IK、MuJoCo、Ruckig 等仍由 CMake 构建，cppzmq/MCAP
和 xCoreSDK 继续使用仓库版本。脚本不安装 VS Code 扩展、ONNX Runtime、Python/pip
环境或设备 udev 规则；独立 T6 示例及 hil-serl 的额外环境需按各自文档配置。
旧 `examples/AviatorRobot_simple/scripts/install_dependencies.sh` 转调同一入口。

Ubuntu 22.04 apt 提供的 clangd 14 在本机解析机器人依赖时仍会报告 `_mm_getcsr`
内建函数兼容诊断；VS Code 已安装的 clangd 22 检查通过。脚本不更改 VS Code 的
`clangd.path`，当前工作区继续使用已配置的 clangd 22。

安装后从仓库根目录执行：

```bash
cmake --preset debug
cmake --build --preset debug
```

## 摇杆设备权限

`setup_joystick_udev.sh` 为 USB 摇杆配置普通用户读取权限，供 flight_gateway 使用。脚本的 `--device` 与网关 `config/flight.yaml` 中的 `device` 均默认 `/dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick`；其他摇杆需向脚本传入 `--device`，并同步修改网关 YAML 的 `device` 路径。

```bash
# 找到实际摇杆的稳定 event 路径（不是 js 路径）。
ls -l /dev/input/by-id/*event-joystick

# 先预览，省略 --device 使用默认 PXN-F16。
./scripts/setup_joystick_udev.sh --dry-run

# 正式安装；默认授权运行 sudo 的用户。
sudo ./scripts/setup_joystick_udev.sh

# 直接以 root 登录时，显式指定普通用户。
sudo ./scripts/setup_joystick_udev.sh --device /dev/input/eventN --user YOUR_USER
```

脚本检查 evdev 字符设备和 `ID_INPUT_JOYSTICK=1`，从同一 USB 父设备读取 VID/PID，创建 `aviator` 组、将用户追加到该组，再写入 `/etc/udev/rules.d/99-aviator-joystick-VID-PID.rules`。规则仅匹配该型号的 joystick event 设备，权限为 `0640`，不授予写设备或力反馈权限。连接多个同型号摇杆时规则会同时适用。

脚本重新加载规则并只触发选定设备的 change 事件。**注销后重新登录**使新组生效；如设备权限尚未变化，再拔插摇杆。使用 `id` 和 `ls -l /dev/input/eventN` 检查，然后以普通用户启动网关。

重复执行更新同一规则文件，不重复追加规则；不会覆盖不带脚本标记的管理员文件。配置失败可能已完成部分步骤，修正错误后可重新运行。`--dry-run` 不执行系统修改。依赖 Linux、udevadm 和标准账户管理工具，不自动安装软件包。

撤销时删除对应规则文件并重新加载规则、拔插设备；若用户不再需要任何 AVIATOR 设备权限，再执行 `sudo gpasswd -d USER aviator` 并重新登录。删除规则不会自动移除用户组成员。

## 脚本回归

`python3 tests/scripts_test.py`（或 CTest 的 `scripts`）覆盖本目录全部入口：语法、帮助、
非法参数、真实/fake/simulation 启动命令、任意工作目录、含空格路径、模拟包管理器安装流程、
udev 只读预览、STL 临时文件处理、TTS 音频包校验、发布元数据及标定 UI HTTP 启停。
替身进程还验证两个整机启动入口的 Core→Manipulator 清理顺序和子进程回收。
`tests/release_test.py` 另验证发布包完整性与安装布局下的启动脚本。
测试不会启动机器人、修改系统包或写入 udev 规则。
