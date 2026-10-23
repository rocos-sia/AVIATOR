"""Fault-only history. Input bytes use RH device order; records use AVIATOR order."""
from collections import deque
from datetime import datetime
from pathlib import Path
import threading
from zoneinfo import ZoneInfo

CHANNELS = ('拇指侧摆', '拇指弯曲', '食指', '中指', '无名指', '小指')
# RH56E2 manual §2.6.18, table 53 (same mapping as pendant/ui.py).
ERROR_BITS = ((1, '堵转'), (2, '过温'), (4, '过流'), (8, '电机异常'), (16, '通信故障'))
TZ = ZoneInfo('Asia/Shanghai')


def describe_error(code):
    names = [name for bit, name in ERROR_BITS if code & bit]
    if code & ~31:
        names.append(f'未知故障位 0x{code & ~31:02X}')
    return '、'.join(names)


class FaultJournal:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.lock = threading.Lock()
        self.history = deque(maxlen=1000)
        self.sequence = 0
        self.write_error = ''

    def observe(self, side, codes, error=''):
        # Normal polls create neither a history entry nor a log file.
        if not error and (codes is None or not any(codes)):
            return
        timestamp = datetime.now(TZ).isoformat(sep=' ', timespec='milliseconds')
        side_name = {'left': '左手', 'right': '右手'}[side]
        faults = [('通信读取', None, '故障码读取失败：' + error)] if error else [
            (CHANNELS[i], code, describe_error(code))
            for i, code in enumerate(reversed(codes)) if code]
        with self.lock:
            lines = []
            for channel, code, description in faults:
                self.sequence += 1
                entry = dict(id=self.sequence, time=timestamp, side=side_name,
                             channel=channel, code=code, description=description)
                self.history.append(entry)
                raw = f'{code} (0x{code:02X})' if code is not None else '未读到'
                lines.append(f'{timestamp} | {side_name} | {channel} | 故障码={raw} | {description}\n')
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                with self.path.open('a', encoding='utf-8') as log:
                    log.writelines(lines)
                self.write_error = ''
            except OSError as exc:
                # Keep the UI history and polling alive even if the disk is unavailable.
                self.write_error = f'故障日志写入失败：{exc}'

    def snapshot(self):
        with self.lock:
            return list(self.history), self.write_error
