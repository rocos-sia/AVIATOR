# 项目配置脚本

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
