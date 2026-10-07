# RH56 灵巧手示教器环境

从 AVIATOR 仓库根目录创建并激活环境：

```bash
conda env create -f third_party/RH56FTP/python/pendant/environment.yml
conda activate rh56-pendant
```

如果已经创建过 `rh56-pendant`，直接激活即可。

## 启动界面

无硬件时使用模拟数据，验证状态表、触觉热力图和控制滑条：

```bash
python third_party/RH56FTP/run_pendant.py --mock
```

连接实机（默认 `192.168.11.210:6000`）：

```bash
python third_party/RH56FTP/run_pendant.py
```

指定地址和端口：

```bash
python third_party/RH56FTP/run_pendant.py --host 192.168.11.210 --port 6000
```

启动器也支持使用绝对路径，从任意目录启动。

## 力传感器校准

先停止 Core、手节点和示教器等控制端，将手移离物体，张开手掌并保持所有手指无接触、无外力。
在 `rh56-pendant` 环境中，指定需要校准的手 IP：

```bash
python third_party/RH56FTP/force_calibrate.py --host 192.168.11.210
```

默认端口为 `6000`，可用 `--port` 修改。脚本连接后显示六路位置、受力、电流和故障码，
确认条件满足后输入 `CALIBRATE`。脚本会再次检查位置、静止及无故障，然后向地址 `1009`
写入一次 `1`，默认观察反馈 10 秒并显示最后五次空载受力范围。
检查失败或取消时不写入校准指令；写入出错时不自动重试。
五个弯曲通道位置须至少为 `950`，这是脚本的张开检查条件，不是厂家规定的校准阈值；
是否真正无接触必须由操作者确认。脚本保持当前手指目标，不自动张开或修改运动参数。

`--timeout` 设置单次 Modbus 请求超时（默认 1 秒），`--observe-seconds` 设置观察时间
（默认 10 秒，不代表厂家规定的校准耗时）。受力应接近零并稳定；手册没有说明完成标志
及校准结果是否自动保存，脚本不写 Flash、不恢复出厂设置，断电后的保留情况需另行验证。

## 依赖与兼容性

环境使用 Python 3.11、PyQt5、pyqtgraph、NumPy 和 pymodbus。
固定 pymodbus 3.11.4，以匹配 `handlink.py` 中的 `device_id` 参数。
这套 UI 通过 Modbus TCP 连接，不需要 ROS 或串口依赖。

正常显示窗口需要桌面会话。仅在无显示器的自动检查中使用
`QT_QPA_PLATFORM=offscreen`，此模式不会显示可交互窗口。
