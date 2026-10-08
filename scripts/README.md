# 项目配置脚本

## 一键启动

按 [启动流程](../docs/启动流程.txt) 启动 Bus、Manipulator（sudo）、Gateway、
RH56FTP、Core、Camera、Monitor，打开全屏 Chrome；指定 `--logger` 时最后启动 Logger：

```bash
./scripts/start_aviator.sh --dry-run
./scripts/start_aviator.sh
./scripts/start_aviator.sh --logger  # 开启日志记录
./scripts/start_aviator.sh --fake-hand  # 真实机械臂和相机 + 模拟 RH56FTP 双手
```

脚本可从任意目录调用。启动前检查文件，通过标准输入自动提交脚本内配置的 sudo 密码；
Manipulator 在独立会话内直接认证，停止时也直接认证，不依赖 sudo 缓存，
各节点日志写入输出的
`/tmp/aviator-start.*` 目录；保持终端打开，Ctrl+C 停止本次启动的所有节点。
节点退出会触发其余节点停止，浏览器保留打开。锁阻止该脚本重复运行；使用前应先停止手动启动的节点。
启动顺序之间默认等待 2 秒（不代表设备已就绪），Monitor HTTP 就绪后打开浏览器。
相机仍使用 `--show`，因此需要桌面显示环境。
默认不开启 Logger，也不启用相机图像录制通道；各节点的运行诊断日志仍写入临时日志目录。
真机或 `--simulation` 模式均可添加 `--logger` 开启记录。
启用时 Logger 使用 `config/recording.yaml`，录制输出为仓库根目录的
`recording_年月日_时分秒_纳秒.mcap`（北京时间，按 Logger 启动时间命名）；
其运行日志保存在上述日志目录的 `logger.log`。

默认 Python 环境为 `~/miniconda3/envs/rh56-pendant/bin/python3` 和
`~/miniconda3/envs/apriltag_realsense/bin/python`，Monitor 使用
`build/bin/aviator_monitor`。可通过 `CONDA_ROOT`、`HAND_PYTHON`、
`CAMERA_PYTHON`、`MONITOR_BIN`、`START_DELAY` 环境变量覆盖。
脚本直接使用环境 Python 并设置 PATH，不执行自定义 Conda 激活钩子。
`--fake-hand` 使用同一个 `HAND_PYTHON`，仅需 `pyzmq` 和 `PyYAML`，不需要
`pymodbus` 或手设备连接。它保留 `config/system.yaml` 的 Core 配置，
手节点替换为 [fake_rh56ftp_hand.py](../nodes/rh56ftp_hand/fake_rh56ftp_hand.py)，
其余启动流程照常执行。不能与 `--simulation` 同时使用，因为整机仿真已经提供手反馈。
`/tmp/aviator_session.uuid` 不存在或为空时生成 UUID，否则沿用现有值。
`--dry-run` 只打印启动命令，不启动节点、不请求 sudo、不写文件。

## 相机随动测试

独立脚本 `start_camera_servo.sh` 启动相机随动测试，原有启动脚本保持不变：

```bash
./scripts/start_camera_servo.sh --dry-run       # 预览启动命令
./scripts/start_camera_servo.sh --fake-hand     # 真实机械臂、相机 + 模拟手
./scripts/start_camera_servo.sh                 # 真实机械臂、相机和手
./scripts/start_camera_servo.sh --fake-hand --logger  # 同时记录 MCAP
```

脚本从任意目录调用均可，使用 `config/system.yaml`，依次启动 Bus、Manipulator
（sudo）、RH56FTP、相机、Monitor 和 Chrome，最后启动 `aviator_core_camera_servo`；
不需要 Gateway。Core 会自动使能、接近并锁定把手，再根据 `cockpit` 相机的
`steering_wheel` 在关节阻抗模式下随动。角度取 `-theta_rad`，位移取
`-translation_along_axis_m - 0.085` 并限幅到 `[-0.170, 0]` m。
启动前先停止手动运行的节点；此脚本与 `start_aviator.sh` 使用同一互斥锁。

默认二进制目录为 `build/release/bin`（可通过 `AVIATOR_BIN` 覆盖），Monitor 默认
也使用此目录。Python 环境同上，可设置 `CONDA_ROOT`、`HAND_PYTHON`、
`CAMERA_PYTHON`、`MONITOR_BIN` 和 `START_DELAY`（默认 2 秒）。
`CAMERA_TIMEOUT_MS` 控制相机观测有效期，默认 200 ms，范围为 20–1000 ms。
相机带 `--show --print-pose`，需要桌面显示环境。sudo 认证沿用原脚本的标准输入方式。

日志保存在输出的 `/tmp/aviator-camera-servo.*` 目录，每次生成独立 session 并保存在
该目录，不改写 `/tmp/aviator_session.uuid`。默认不录制图像；`--logger` 会在相机前
启动 Logger，并将记录保存到 `logs/camera_servo_时间.mcap`。
保持终端打开；Ctrl+C 先停止 Core，最多等待 15 秒完成减速、解锁和下使能，再停止
其他节点。`--dry-run` 仅预览，不启动节点、不请求 sudo、不写文件；`--help` 查看完整用法。

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
