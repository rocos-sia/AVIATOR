# 双手静态抓握工具校准

先手动示教，让双手握住轮盘把手并保持静止。工具读取双臂实测关节角，按 Core 使用的
URDF 做正运动学，再结合轮盘的实际角度、推拉位移，反算 `grasp.json` 中
`tool.left` 和 `tool.right` 的 `position`、`quaternion`。默认只预览；加 `--write`
才会备份并写回。

## 准备

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
在其他控制程序或运动仍运行时使用。机械手应已保持合适的抓握状态，本工具不会控制手指。

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
