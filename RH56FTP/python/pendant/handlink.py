"""RH56E2 灵巧手 Modbus TCP 传输层。

手册《PRJ-02-TS-U-011 RH56E2系列用户手册》中的地址就是 Modbus 寄存器号；
一个寄存器含手册地址 N 和 N+1 两个字节，低字节为手册地址 N。
"""
from __future__ import annotations

import time

from pymodbus.client import ModbusTcpClient

# ---------------------------------------------------------------- 常量

HAND_IP = "192.168.11.210"
HAND_PORT = 6000
DEVICE_ID = 1

# 自由度顺序（手册 2.6.x 顺序），写 6 个寄存器时按此序
FINGERS = ["小指", "无名指", "中指", "食指", "拇指弯曲", "拇指侧摆"]
N_FINGERS = 6

# 触觉区（手册表 56 压阻式）。地址即寄存器号。
TOUCH_ZONES = [
    ("小指",   3000, 3369),
    ("无名指", 3370, 3739),
    ("中指",   3740, 4109),
    ("食指",   4110, 4479),
    ("拇指",   4480, 4899),
    ("掌心",   4900, 5123),
]
TOUCH_ZONE_NAMES = [z[0] for z in TOUCH_ZONES]

# 触觉点量程（手册：16 位整型，0-4095）。超出此范围视为无效帧。
TOUCH_MAX = 4095

MAX_REGS_PER_READ = 125  # Modbus 0x03 单次上限

REG = {
    "id":         1000,
    "clearErr":   1004,
    "saveFlash":  1005,
    "resetPara":  1006,
    "forceClb":   1009,
    "angleSet":   1486,
    "forceSet":   1498,
    "speedSet":   1522,
    "posAct":     1534,
    "angleAct":   1546,
    "forceAct":   1582,
    "currentAct": 1594,
    "errCode":    1606,  # 6 byte
    "statusCode": 1612,  # 6 byte
    "temp":       1618,  # 6 byte
    "mode":       1625,  # 6 byte
}


class HandError(RuntimeError):
    pass


def _as_signed(v: int) -> int:
    """寄存器 → 有符号 16 位（受力可能为负）。"""
    return v - 65536 if v > 32767 else v


def _split_bytes(regs: list[int], count: int) -> list[int]:
    """3 个寄存器 → 6 个字节，低字节在前（与手册字节地址一致）。"""
    out: list[int] = []
    for r in regs:
        out.append(r & 0xFF)
        out.append((r >> 8) & 0xFF)
    return out[:count]


class HandLink:
    """灵巧手连接。所有方法都可能抛 HandError。"""

    def __init__(self, host: str = HAND_IP, port: int = HAND_PORT, timeout: float = 1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._cli: ModbusTcpClient | None = None

    # ------------------------------------------------------------ 连接

    def connect(self) -> None:
        if self._cli is not None:
            return
        self._cli = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout)
        if not self._cli.connect():
            self._cli = None
            raise HandError(f"无法连接灵巧手 {self.host}:{self.port}")

    def close(self) -> None:
        if self._cli is not None:
            self._cli.close()
            self._cli = None

    @property
    def connected(self) -> bool:
        return self._cli is not None

    # ------------------------------------------------------------ 原语

    def _read(self, addr: int, count: int) -> list[int]:
        if self._cli is None:
            raise HandError("未连接")
        r = self._cli.read_holding_registers(address=addr, count=count, device_id=DEVICE_ID)
        if r.isError():
            raise HandError(f"读寄存器 {addr}×{count} 失败: {r}")
        return list(r.registers)

    def _write(self, addr: int, values: list[int]) -> None:
        if self._cli is None:
            raise HandError("未连接")
        vals = [v & 0xFFFF for v in values]
        r = self._cli.write_registers(address=addr, values=vals, device_id=DEVICE_ID)
        if r.isError():
            raise HandError(f"写寄存器 {addr} 失败: {r}")

    # ------------------------------------------------------------ 读状态

    def read_state(self) -> dict:
        """一次状态巡读：角度/受力/电流/故障码/状态码/温度。"""
        angle = [_as_signed(v) for v in self._read(REG["angleAct"], N_FINGERS)]
        force = [_as_signed(v) for v in self._read(REG["forceAct"], N_FINGERS)]
        current = self._read(REG["currentAct"], N_FINGERS)
        err = _split_bytes(self._read(REG["errCode"], 3), N_FINGERS)
        status = _split_bytes(self._read(REG["statusCode"], 3), N_FINGERS)
        temp = _split_bytes(self._read(REG["temp"], 3), N_FINGERS)
        return {
            "t": time.time(),
            "angle": angle,
            "force": force,
            "current": current,
            "err": err,
            "status": status,
            "temp": temp,
        }

    # ------------------------------------------------------------ 读触觉

    def read_touch(self) -> dict[str, list[int]]:
        """读整手触觉。返回 {部位名: [原始值...]}，长度见手册（185/210/112）。"""
        out: dict[str, list[int]] = {}
        for name, start, end in TOUCH_ZONES:
            out[name] = self._read_range(start, end)
        return out

    def _read_range(self, start: int, end: int) -> list[int]:
        vals: list[int] = []
        addr = start
        while addr <= end:
            n = min(MAX_REGS_PER_READ, end - addr + 1)
            vals.extend(self._read(addr, n))
            addr += n
        return vals

    # ------------------------------------------------------------ 写控制

    def write_angle_set(self, values: list[int]) -> None:
        """六个自由度角度设定，0-1000；-1 表示该指保持不动。"""
        self._write(REG["angleSet"], values)

    def write_force_set(self, values: list[int]) -> None:
        """六个自由度力控阈值，0-3000。"""
        self._write(REG["forceSet"], values)

    def write_speed_set(self, values: list[int]) -> None:
        """六个自由度速度，0-1000。"""
        self._write(REG["speedSet"], values)

    def clear_error(self) -> None:
        self._write(REG["clearErr"], [1])

    def save_flash(self) -> None:
        self._write(REG["saveFlash"], [1])

    def reset_para(self) -> None:
        self._write(REG["resetPara"], [1])

    def force_calibrate(self) -> None:
        self._write(REG["forceClb"], [1])
