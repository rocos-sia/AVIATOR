# 双手静态抓握工具校准

先手动示教，让双手握住轮盘把手并保持静止。工具读取双臂实测关节角，按 Core 使用的
URDF 做正运动学，再结合轮盘的实际角度、推拉位移，反算 `grasp.json` 中
`tool.left` 和 `tool.right` 的 `position`、`quaternion`。可使用下面的图形界面完成
节点管理、机械手控制和双臂拖动，也保留原有命令行方式。计算结果先预览，明确执行写回后
才会备份并更新文件。

## 图形界面

在仓库根目录构建：

```bash
cmake -S . -B build
cmake --build build --target aviator_grasp_tool_session aviator_bus -j2
```

界面服务需要 Python 3、NumPy、PyYAML、pyzmq；真实相机仍使用已有的
`apriltag_realsense` 环境及相机依赖。真机连接需要构建时启用 `AVIATOR_BUILD_XCORE_SDK`。
使用新脚本启动，原有启动脚本不需要修改：

```bash
./scripts/start_grasp_calibration_ui.sh
```

默认界面地址为 `http://127.0.0.1:8766`，启动后自动打开本机浏览器。界面服务只监听
localhost，打开页面不会立即连接硬件。`--no-browser` 可关闭自动打开，`--port` 可修改
本地端口；完整选项使用 `./scripts/start_grasp_calibration_ui.sh --help` 查看。

启动脚本默认使用 `python3` 运行 UI；可通过 `UI_PYTHON` 指向安装了上述 Python 依赖
的解释器。机械手和相机的默认解释器分别是
`~/miniconda3/envs/rh56-pendant/bin/python3` 和
`~/miniconda3/envs/apriltag_realsense/bin/python`，对应文件不存在时回退到 UI 解释器。
可分别用 `--hand-python`、`--camera-python` 覆盖。示例：

```bash
UI_PYTHON=~/miniconda3/envs/apriltag_realsense/bin/python \
  ./scripts/start_grasp_calibration_ui.sh \
  --hand-python ~/miniconda3/envs/rh56-pendant/bin/python3 \
  --camera-python ~/miniconda3/envs/apriltag_realsense/bin/python
```

配置文件可通过 `--robot-config`、`--system-config`、`--hand-config`、`--camera-config`
指定；页面会显示实际路径和机械臂地址。原生程序默认从 `build/bin` 查找，可通过
`AVIATOR_BIN` 指定整个目录，或分别设置 `--session-helper`、`--bus-helper`。
节点与 SDK 日志优先保存到 `logs/grasp_calibration_ui/时间戳-PID/`；若仓库日志目录不可写，
自动改用系统临时目录下独立的 `aviator-grasp-calibration-ui-*` 目录（通常在 `/tmp`）。
启动时会打印实际日志目录，采样报告也写入同一目录。可用 `--log-dir` 指定持久保存位置；
显式指定的目录不可写时会报错，不会自动换位置。临时目录中的日志可能被系统清理。

使用普通用户启动即可。不要为日志权限问题改用 `sudo`：它可能切换 Python 环境，
导致普通用户已安装的 NumPy、PyYAML、pyzmq 无法导入。也可直接指定可写目录：

```bash
./scripts/start_grasp_calibration_ui.sh --log-dir /tmp/aviator-grasp-logs
```

可先检查模拟流程：

```bash
./scripts/start_grasp_calibration_ui.sh --dry-run
```

模拟模式不连接真实机械臂、机械手或相机，也不启动这些设备节点；机械臂使用模拟关节
角执行相同的 FK，轮盘和机械手反馈为模拟数据。可以演示拖动状态、采样和预览，但禁止
写回 `grasp.json`，不能用模拟结果做真实标定。

### 界面操作顺序

1. 退出 Core、Manipulator、已有机械手节点和其他机械臂控制程序。选择“相机实时测量”
   或“手动填写”，点击“准备环境 · 双手张开”。工具创建自己的 Bus 和机械手节点；
   相机模式同时启动 Camera，手动模式不启动 Camera。已有节点或端口冲突会报错，
   工具不会接管或终止外部节点。
2. 根据需要开启左臂、右臂或双臂拖动，手动将双手放到轮盘把手处。拖动通过 SDK 切换
   **手动模式 → 下电 → 关节空间自由拖动**，使用
   `enableDrag(DragParameter::jointSpace, DragParameter::freely, ec, false)`。
   最后一个参数为 `false`，保留机械臂末端拖动按钮的要求。
3. 使用张开/握紧按钮调整左手、右手或双手，观察实际抓握状态。目标读取
   `config/system.yaml` 的 `core_hand.open` / `core_hand.close`；当前默认值如下。
4. 结束双臂拖动，让双臂和轮盘保持静止。结束拖动不会自动切回自动模式或上电。
   相机模式确认页面上换算后的角度、位移有效；手动模式直接输入 Core 坐标中的
   实际角度（度）和推拉位移（米）。
5. 点击“保持静止并采样预览”，查看两侧新旧工具位姿、变化量、静止波动和重建残差。
   采样与拖动使用同一组持续连接的机械臂会话，采样不会临时创建另一套 SDK 连接。
6. 点击“备份并写回 grasp.json”。界面保存刚才预览的结果，不重新采样；若配置在此期间
   已被修改，写回会被拒绝。后续手部操作、拖动或新采样会使旧预览失效。
7. 完成后点击“结束会话 · 结束拖动并张开双手”。工具先结束自己开启的拖动、释放
   SDK 连接，再张开双手并关闭自己启动的节点。

```yaml
core_hand:
  open:
    left: [0.5, 1, 1, 1, 1, 1]
    right: [0.5, 1, 1, 1, 1, 1]
  close:
    left: [0.5, 0, 0, 0, 0, 0]
    right: [0.5, 0, 0, 0, 0, 0]
```

六个通道依次为拇指旋转、拇指弯曲、食指、中指、无名指、小指。界面持续发送当前手部
目标，避免只发一次命令后触发机械手命令超时；单手按钮保持另一只手的目标不变。
“指令已确认”仅表示机械手节点接收了本会话的命令，不能证明实际已握住轮盘。

页面关闭会请求结束会话；若页面心跳失联约 10 秒，仍在运行的界面服务也会结束自有
拖动并张开双手。终端 Ctrl+C 同样执行退出清理。**退出会张开双手**，应先确保轮盘
及机械臂无需依靠手指抓握保持位置。SDK 断连操作本身会停止机械臂运动。
此清理依赖服务和设备通信仍可工作；进程被强制杀死、断电或网络故障时不能保证执行。
进程冲突检查仅覆盖本机同一用户；SDK 必须建立连接后才能查询机械臂运行状态，
拒绝外部控制后断开 SDK 也可能停止那台机械臂。因此准备前仍须退出所有其他控制与示教程序，
不能把准备按钮当作对其他控制器无影响的状态探测。

相机仍使用下面介绍的转换：`angle = -theta`、`displacement = -translation - 0.085`，
合法位移范围为 `[-0.170, 0] m`。相机数据超范围时拒绝，不自动限幅。界面采样复用
原工具的同主机时钟检查、数据时效检查、相机时间配对和静止性检查。手动输入不再转换。

## 命令行工具：准备

在仓库根目录构建采样程序：

```bash
cmake -S . -B build
cmake --build build --target aviator_grasp_tool_state -j2
```

Python 入口使用相机的 Conda 环境，需要 NumPy、PyYAML；相机模式还需要 pyzmq。默认从仓库
`build/bin`、`build/release/bin` 依次寻找 `aviator_grasp_tool_state`，也可以用
`--state-helper /绝对路径/aviator_grasp_tool_state` 指定其他构建目录。
真机采样需要该程序构建时启用 `AVIATOR_BUILD_XCORE_SDK`。

运行前退出 Core 和 Manipulator，不要启动 `start_aviator.sh` 或相机随动 Core。
完成拖动后结束示教运动，确认双臂和轮盘保持静止。工具不使能、不切换自动/实时模式、
不修改 SDK 工具参数、不发送轨迹；但 **SDK 断连操作本身会停止机械臂运动**，因此不能
在其他控制程序或运动仍运行时使用。机械手应已保持合适的抓握状态，**命令行采样工具**
不会控制手指；前面介绍的 UI 则包含明确的手部与拖动控制。

默认读取仓库 `config/robot.yaml`，可用 `--robot-config PATH` 覆盖；URDF、姿态配置
和待更新的抓取文件路径均按该 YAML 所在目录解析。真机地址来自其中的
`rokae.left_ip`、`rokae.right_ip`。

## 使用相机测量轮盘

只启动 Bus 和相机，不启动 Core、Manipulator 或 Gateway。在不同终端执行：

```bash
./build/bin/aviator_bus --config config/system.yaml
```

相机继续使用已有命令；需保证 `/tmp/aviator_session.uuid` 中有有效 session UUID。
如果是独立校准的新会话且文件不存在，可先运行
`~/miniconda3/envs/apriltag_realsense/bin/python -c 'import uuid; print(uuid.uuid4())' > /tmp/aviator_session.uuid`。

```bash
~/miniconda3/envs/apriltag_realsense/bin/python nodes/camera/main.py \
  --config config/camera.yaml --recording-config config/recording.yaml \
  --camera-id cockpit --session "$(cat /tmp/aviator_session.uuid)" \
  --show --print-pose
```

预览校准结果，同时保存采样和结果报告：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python tools/grasp_tool_calibration/calibrate.py \
  --wheel-source camera --camera-id cockpit \
  --report /tmp/grasp-tool-preview.json
```

确认预览结果后，保持当前姿态，再执行一次采样并写回：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python tools/grasp_tool_calibration/calibrate.py \
  --wheel-source camera --camera-id cockpit --write \
  --report /tmp/grasp-tool-written.json
```

相机输入使用 `camera.detection.steering_wheel`，按 Monitor 的约定转换：

```text
angle_rad      = -theta_rad
displacement_m = -translation_along_axis_m - 0.085
```

相机轴向平移 `-0.085 / 0 / +0.085 m` 分别对应 Core 位移 `0 / -0.085 / -0.170 m`。
相机与工具须运行在同一主机，使用相同的单调时钟。默认连接 `tcp://127.0.0.1:5556`，
可用 `--endpoint` 覆盖。相机输入会检查有效性、时效、时钟、session 和标定标识连续性，
并与关节采样按时间配对；采样期间不要重启相机或更新相机标定。
超出模型位置范围时拒绝校准，不通过限幅隐藏错误。相机轮盘零位、轴向标定应事先完成。

## 手动指定轮盘位形

手动模式仍读取真实机械臂关节角，但不需要 Bus 和相机。角度、位移直接使用
**Core 坐标约定**，不会再次进行上述相机转换。角度可以用 `--angle-deg` 或
`--angle-rad` 指定，二者只选一个。角度范围为 `[-0.87266, 0.87266] rad`（约 ±50°）；
位移单位为米，范围为 `[-0.170, 0]`。输入超限会拒绝，不会自动限幅。

```bash
~/miniconda3/envs/apriltag_realsense/bin/python tools/grasp_tool_calibration/calibrate.py \
  --wheel-source manual --angle-deg 0 --displacement-m -0.085 \
  --report /tmp/grasp-tool-manual.json
```

需要写回时加 `--write`。手动值必须描述采样时的真实轮盘姿态；程序无法验证输入值
是否与轮盘实际位置相符。

## 输出与计算约定

默认采集 20 组关节数据，采样间隔为 50 ms，可用 `--samples` 和 `--interval-ms` 调整。
Python 校准入口接受 3～500 组样本。
`--timeout-s` 默认为 20 秒，必须大于总采样跨度。采样期间检查静止程度，相关选项如下：

| 参数 | 默认值 | 检查内容 |
| --- | ---: | --- |
| `--max-joint-span-deg` | 0.2 | 每个关节采样期间的最大值减最小值，度 |
| `--max-position-span-mm` | 2 | 同侧法兰任意两帧的位置距离，mm |
| `--max-rotation-span-deg` | 0.5 | 同侧法兰任意两帧的姿态差，度 |
| `--max-wheel-angle-span-deg` | 0.5 | 轮盘角度采样范围，度 |
| `--max-wheel-displacement-span-mm` | 3 | 轮盘推拉位移采样范围，mm |

默认 `--max-age-ms 250` 限制关节/相机样本接收年龄，`--max-sync-ms 100` 限制
相机采样与双臂读取起止时刻的最大差；相机最低置信度为 `--min-confidence 0.5`。
这些阈值可按静止噪声调整，增大阈值也会放宽对运动和测量误差的拒绝条件。

程序分别显示左右工具的新旧参数、平移变化量（mm）、旋转变化量（度）和同姿态重建残差。
`--report PATH` 保存采样、轮盘测量和计算结果，便于复查。报告在写回 `grasp.json`
之前生成，其中的 `write_requested` 仅表示请求了写回，不代表写回成功。是否写回成功及
原文件的实际备份路径，以控制台输出为准。

每侧计算使用同一坐标关系：

```text
T_aircraft_flange = FK(实测关节角)
T_aircraft_grasp  = wheel_origin × SE3(Rz(angle), [0, 0, displacement]) × grasp[side]
T_flange_tool     = inverse(T_aircraft_flange) × T_aircraft_grasp
```

`grasp[side]` 是 `grasp.json` 顶层的 `left` / `right` 抓取位姿。
正运动学计算法兰位姿，不依赖旧的 `tool`。保存的位置单位为米，四元数顺序为
**`[w, x, y, z]`**。写回仅更新两侧工具位姿；保留其他配置字段，先备份原文件字节，
再原子替换目标文件。控制程序下次启动时加载新值。

单个静态姿态会把 `wheel_origin`、把手模型、相机标定等误差一并吸收到 `tool` 中。
同姿态重建残差验证的是这次反算关系，不是独立的精度测量。建议校准后换一个轮盘
角度或位移，重新手动摆好双手，只运行预览，比较反算出的工具参数是否一致。

## 离线检查

`--joints-file` 可替代 SDK 采样。文件为 JSON 对象，左右各 7 个关节角，按 J1～J7
排列，单位为弧度。例如下面的零关节姿态仅用于检查计算流程，不代表正确抓握位置：

```json
{"left": [0, 0, 0, 0, 0, 0, 0], "right": [0, 0, 0, 0, 0, 0, 0]}
```

将其保存为 `/tmp/grasp-joints.json`，运行：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python tools/grasp_tool_calibration/calibrate.py \
  --joints-file /tmp/grasp-joints.json \
  --wheel-source manual --angle-rad 0 --displacement-m -0.085 \
  --report /tmp/grasp-tool-offline.json
```

这个组合不连接机械臂，也不需要 Bus 或相机。也可只检查原生程序的法兰 FK 输出：

```bash
./build/bin/aviator_grasp_tool_state --robot-config config/robot.yaml \
  --joints-file /tmp/grasp-joints.json --samples 3 --interval-ms 1
```

静态文件模式只能验证计算和文件格式，不能验证实机稳定性或轮盘测量误差。
