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

## 依赖与兼容性

环境使用 Python 3.11、PyQt5、pyqtgraph、NumPy 和 pymodbus。
固定 pymodbus 3.11.4，以匹配 `handlink.py` 中的 `device_id` 参数。
这套 UI 通过 Modbus TCP 连接，不需要 ROS 或串口依赖。

正常显示窗口需要桌面会话。仅在无显示器的自动检查中使用
`QT_QPA_PLATFORM=offscreen`，此模式不会显示可交互窗口。
