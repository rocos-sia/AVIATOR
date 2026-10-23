# 工控机一键部署

在开发机编译并运行以下命令，把运行包安装到 `rocos@192.168.11.100:/opt/aviator`：

```bash
cd /home/sia/Documents/GitHub/AVIATOR
./scripts/deploy_aviator.sh
```

部署连接的 SSH 和 sudo 如需认证，会在当前终端提示输入密码；部署工具不保存这些输入。硬件启动脚本保留原有内置 sudo 密码，启动时无需手动输入。只在开发机执行 CMake/C++ 编译和 Python wheel 构建。目标工控机只解包、安装系统运行库与预编译 wheel、检查和替换安装，不安装项目的编译工具，不接收 C++ 源码。

## 前提

- 开发机与目标机均为 Ubuntu 22.04 x86_64；目标使用 Python 3.10。实时内核无需修改。
- 已能执行 `ssh rocos@192.168.11.100`，该账号能使用 sudo。
- 开发机已有主工程构建依赖，以及 `qtbase5-dev python3-pip python3-dev python3-venv`；首次可以在开发机执行 `scripts/install_dependencies.sh` 准备主工程依赖。
- 开发机能访问 PyPI 和 Google 的 Chrome 下载地址。目标机首次安装系统运行库时能访问 Ubuntu 软件源。网线直连本身不提供互联网；必要时另外接入网络。
- RealSense udev 规则随仓库提供并打包，无需从开发机系统复制；规则与部署用 SDK 配套，包含 D436（`8086:1156`）。
- 更新前停止已部署的 AVIATOR 程序及桌面启动器。如果配置了 systemd 自启/自动重启服务，先停止对应服务，部署完成后再启动。安装器检测到部署目录中的进程时会退出，不强制结束程序。

## 部署内容

包括主工程硬件/仿真程序、动态库、全部 `scripts/*.sh` 和 `scripts/*.py`、Python 相机与手节点、标定工具、轮盘工具、配置、机器人模型、TTS 模型和音频，以及独立的 `xCoreSDK_RH56FTP_485`、`rh56_485_probe`、`run.sh` 和 `run_grip_demo.sh`。

`include/`、静态库、C/C++ 源文件、CMake 开发文件、Git 数据和构建目录不会进入发布压缩包。Shell、Python、网页和配置按原格式部署。`install_dependencies.sh` 保留供维护使用，但它是开发机依赖安装工具，不会在工控机部署流程中执行。

程序、库通过 CMake 安装后再打包，保留安装 RPATH。打包时检查 ELF 动态库依赖，并从开发机 dpkg 数据库生成目标系统所需的运行包清单；不属于系统包的动态库（例如开发机 `/usr/local/lib` 中的 NLopt）自动随包复制，并通过本机的 patchelf 修正 RPATH；无法解析或存在文件名冲突时明确报错。patchelf 缺失时仅在本机的打包虚拟环境安装。

Python 依赖在开发机生成 wheel，目标端在固定的 `/opt/aviator/cache/python` 虚拟环境中离线安装。相机、标定、TTS、手部演示等启动脚本会自动使用这个环境。Chrome 的官方 deb 随包提供，仅在目标机没有 `google-chrome-stable` 时安装。

RealSense 固定为 `pyrealsense2==2.58.1.10581`，支持 D436。`deploy/99-aviator-realsense.rules` 来自 [SDK v2.58.1 官方规则](https://github.com/realsenseai/librealsense/blob/v2.58.1/config/99-realsense-libusb.rules)。SDK 2.56.5 无法识别 D436，不能仅凭 Python 导入成功判断实机可用。

目标端安装 `python3.10-venv`，并明确使用 `/usr/bin/python3.10` 创建环境，不请求会严格绑定 `python3` 默认包版本的 `python3-venv` 元包。安装器也会转换旧发布包中的这一依赖名称。若 APT 仍报告 `python3.10-venv` 与 `python3.10` 版本不匹配，需要检查目标机软件源是否提供配套版本；安装器不会自动降级系统 Python。

## 固定目录与完整替换

```text
/opt/aviator/                 实体目录，没有日期层，也不是版本目录的符号链接
  bin/                       程序
  lib/                       动态库
  share/aviator/scripts/     启动与维护脚本
  share/aviator/config/      保留现场配置，补齐新增配置文件
  share/aviator/examples/xCoreSDK_RH56FTP_485/
  share/aviator/models/      模型
  share/aviator/deploy/      安装器和依赖包
  cache/python/             当前 Python 环境
  deployment.json           当前部署记录
  release-version.json
  SHA256SUMS
/opt/aviator-data/logs/      运行日志，独立于安装内容保留
```

每次部署完整替换上一套程序、库、Python 环境和资源，删除其中的旧文件，但保留工控机上的已有配置。保留范围（相对于 `/opt/aviator`）为：

- `share/aviator/config/` 下的全部文件，包括现场自行新增的配置和子目录。
- `share/aviator/monitor/monitor.yaml`。
- `share/aviator/examples/xCoreSDK_RH56FTP_485/config.ini`。

同名配置以工控机上的整个文件为准，不合并本机新增的字段；仅当工控机没有对应文件时，才安装发布包中的默认配置。首次部署使用包内默认配置。如果新版要求新增配置字段，需按新版要求修改现场配置；检查失败会保留原安装，不会用默认配置覆盖现场设置。配置中遇到符号链接或文件/目录类型冲突时会报错退出，保留原安装。

不会继续积累历史版本，也不提供部署后的 `--rollback` 命令。运行日志不属于部署文件，继续保留在固定的 `/opt/aviator-data/logs`。

安装器先在固定的隐藏暂存目录 `/opt/.aviator-install` 解包和检查，临时备份旧安装，再将新目录放到 `/opt/aviator`。Python 环境在最终路径重新创建，确保 pip 等入口不含暂存路径；最终检查通过后删除临时备份和暂存目录。最终检查失败会自动恢复原安装。中途中断后再次运行部署脚本，会根据事务记录恢复或完成清理。系统包、设备组和 udev 规则等系统级变更不自动撤销。

此前使用日期目录部署过的机器会自动迁移：把 `/opt/aviator` 改为实体目录，成功后删除 `/opt/aviator-releases` 内能确认属于旧部署的版本目录，目录为空时一并移除它。无法识别的其他文件不会删除。如果 `/opt/aviator` 是与本工具无关的目录或链接，仍拒绝覆盖。

开机自启始终引用 `/opt/aviator/...`，后续更新无需修改路径。当前部署只准备程序和桌面入口，不自动设置自启；硬件启动脚本用内置密码完成 sudo 认证，配置无人值守启动时仍需正确设置服务账号和图形会话。

## 常用命令（都在开发机执行）

```bash
# 默认：本机编译、打包、上传、安装；不启动设备
./scripts/deploy_aviator.sh

# 指定目标与本机编译并发数
./scripts/deploy_aviator.sh --target rocos@192.168.11.100 --jobs 2

# 只在本机准备运行包，不连接目标机
./scripts/deploy_aviator.sh --package-only

# 已完成本机 Release 编译时跳过编译；仍重新安装打包和检查
./scripts/deploy_aviator.sh --skip-build --skip-python-download
```

本机产物位于 `build/deployment/aviator-runtime.tar.gz`，附带 SHA256 文件。`--skip-python-download` 仍会检查缓存 wheel 是否能完整安装；缺包会报错。默认复用本机 `build/`，可用 `--build-dir` 指定其他构建目录。为这个部署配置，脚本会设置 Release、关闭测试和无关示例、启用全部运行节点及 485 示例。

## 启动与检查边界

部署检查包括校验清单、动态库解析、Python 模块导入、Shell 语法、启动命令预览、标定工具的 help 模式和 485 probe 的 dry-run。不会连接机器人、打开串口、执行电机上电，也不创建开机自动启动服务。

在工控机的桌面应用菜单打开 **AVIATOR 启动器**、**AVIATOR 双手 485 测试**、**AVIATOR 手部演示**，或在工控机桌面终端执行：

```bash
/opt/aviator/share/aviator/scripts/start_aviator.sh
/opt/aviator/share/aviator/scripts/start_camera_servo.sh
/opt/aviator/share/aviator/examples/xCoreSDK_RH56FTP_485/run.sh
```

普通 SSH 会话不会自动获得工控机桌面的显示权限；桌面入口是在当前登录的普通用户会话里启动。主程序和相机随动启动脚本沿用原有内置 sudo 密码，通过标准输入认证，不再弹出密码框或要求终端输入。

目标账号加入 `dialout`、`video`、`plugdev`；新组权限需重新登录，RealSense udev 规则需重新插拔设备或重启后生效。脚本不修改机械臂网卡、串口编号、相机序列号和机器人 IP，不重启系统。首次实际运行前仍需核对这些现场配置。例如仓库的 `flight-simulator.yaml` 串口路径含待替换占位符，不能据此保证实机通信。

标定工具可能修改配置，TTS 生成器也会修改音频及清单（更新部署会重新安装包内音频）。确认这些修改后，可从开发机重新生成当前安装的校验清单：

```bash
ssh -t rocos@192.168.11.100 \
  'sudo python3 /opt/aviator/share/aviator/deploy/remote.py --refresh-checksums'
```

下一次部署会继续保留上述范围内的现场配置，并根据新程序与实际保留的配置重新生成校验清单；部署前不要求旧配置与旧清单一致。
