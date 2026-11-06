#!/usr/bin/env python3
"""独立接收项目双路 RS422 下行，Ctrl+C 结束并统计；不发送任何数据。"""
import argparse
from collections import deque
from contextlib import ExitStack
import copy
import fcntl
import json
import os
from pathlib import Path
import select
import selectors
import signal
import stat
import struct
import sys
import termios
import time

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config/flight-rs422.yaml'
# docs/AVIATOR_RS422通信协议规范.md and flight_gateway/serial_port.cpp.
FRAME_SIZE = 32
HEADER = b'\xeb\x90'
PERIOD_S = 0.0125
# Linux termios.h; Python 3.10 does not expose this flag on Ubuntu 22.04.
CMSPAR = getattr(termios, 'CMSPAR', 0x40000000)


def load_config(path):
    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node):
        result = {}
        for key, value in node.value:
            name = loader.construct_object(key)
            if name in result:
                raise ValueError(f'配置重复字段：{name}')
            result[name] = loader.construct_object(value)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    with path.open(encoding='utf-8') as stream:
        root = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(root, dict) or root.get('role', 'robot') != 'robot' or root.get('source') != 'rs422':
        raise ValueError('需要项目的 robot / rs422 配置')
    rs = root.get('rs422')
    if not isinstance(rs, dict) or rs.get('profile') != 'aviator-bench-v1':
        raise ValueError('rs422.profile 必须为 aviator-bench-v1')
    ports = {name: rs.get('channel_' + name.lower()) for name in ('A', 'B')}
    if any(not isinstance(p, str) or not Path(p).is_absolute() for p in ports.values()):
        raise ValueError('channel_a / channel_b 必须为绝对串口路径')
    if ports['A'] == ports['B']:
        raise ValueError('A/B 不能使用同一串口')
    timeout = rs.get('receive_timeout_ms', 50)
    if type(timeout) is not int or not 25 <= timeout <= 100:
        raise ValueError('rs422.receive_timeout_ms 必须为 25..100 的整数')
    return ports, timeout


class Receiver:
    """Envelope counts are independent of gateway business/freshness filtering."""
    def __init__(self):
        self.buffer = bytearray()
        self.rx_bytes = self.valid_frames = self.semantic_invalid_frames = 0
        self.checksum_failures = self.discarded_bytes = 0
        self.counter_gap_events = self.counter_missing_estimate = 0
        self.same_counter_events = self.ambiguous_events = 0
        self.first_counter = self.last_counter = None
        self.first_time = self.last_time = None
        self.max_valid_gap_s = 0.0
        self.events = deque(maxlen=1000)
        self.event_count = 0

    def event(self, kind, now, **fields):
        self.event_count += 1
        self.events.append(dict(kind=kind, elapsed_s=round(now, 6), **fields))

    def accept(self, frame, now):
        self.valid_frames += 1
        counter = frame[2]
        roll, pitch = struct.unpack_from('<hh', frame, 4)
        if frame[3] not in (0, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60) or abs(roll) > 1000 or abs(pitch) > 1000:
            self.semantic_invalid_frames += 1
            self.event('invalid_business_fields', now, counter=counter)
        if self.last_counter is not None:
            gap = now - self.last_time
            self.max_valid_gap_s = max(self.max_valid_gap_s, gap)
            delta = (counter - self.last_counter) & 0xff
            if gap >= 256 * PERIOD_S:
                self.ambiguous_events += 1
                self.event('possible_full_wrap_or_restart', now, previous=self.last_counter,
                           current=counter, gap_s=gap, delta=delta)
            elif delta == 0:
                self.same_counter_events += 1
                self.event('same_counter', now, counter=counter)
            elif delta >= 128:
                self.ambiguous_events += 1
                self.event('reverse_or_large_gap_or_restart', now,
                           previous=self.last_counter, current=counter, delta=delta)
            elif delta > 1:
                self.counter_gap_events += 1
                self.counter_missing_estimate += delta - 1
                self.event('counter_gap', now, previous=self.last_counter,
                           current=counter, missing_estimate=delta - 1)
        else:
            self.first_counter, self.first_time = counter, now
        # After an ambiguous transition, start a new local comparison baseline.
        self.last_counter, self.last_time = counter, now

    def feed(self, data, now):
        self.rx_bytes += len(data)
        self.buffer.extend(data)
        while self.buffer:
            index = self.buffer.find(HEADER)
            if index < 0:
                # Keep a split first header byte for the next read.
                discard = len(self.buffer) - int(self.buffer[-1] == HEADER[0])
            else:
                discard = index
            if discard:
                self.discarded_bytes += discard
                del self.buffer[:discard]
            if index < 0 or len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            if sum(frame[2:31]) & 0xff != frame[31]:
                self.checksum_failures += 1
                self.event('checksum_failure_candidate', now, hex=frame.hex(' '))
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            self.accept(frame, now)
            del self.buffer[:FRAME_SIZE]

    def summary(self, expected, elapsed):
        values = {key: value for key, value in vars(self).items()
                  if key not in ('buffer', 'events')}
        values.update(expected_frames=expected, count_shortfall=max(0, expected - self.valid_frames),
                      count_excess=max(0, self.valid_frames - expected),
                      reception_percent=100 * self.valid_frames / expected,
                      trailing_partial_bytes=len(self.buffer),
                      observed_frames_per_s=self.valid_frames / elapsed if elapsed else 0,
                      events=list(self.events), omitted_events=max(0, self.event_count - len(self.events)))
        return values


class SerialPort:
    def __init__(self, path):
        self.path, self.fd, self.original = path, None, None

    def open(self):
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise OSError(f'串口已被占用：{self.path}；请停止 flight_gateway 或其他测试进程') from exc
            if not stat.S_ISCHR(os.fstat(self.fd).st_mode) or not os.isatty(self.fd):
                raise ValueError(f'不是串口设备：{self.path}')
        except BaseException:
            self.close()
            raise
        return self

    def configure(self):
        self.original = termios.tcgetattr(self.fd)
        attrs = copy.deepcopy(self.original)
        # Match cfmakeraw + the overrides in serial_port.cpp, including INPCK.
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
                      termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON |
                      termios.IXOFF | termios.IXANY | termios.IGNPAR)
        attrs[0] |= termios.INPCK
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.CSIZE | termios.PARODD | CMSPAR |
                      termios.CSTOPB | termios.CRTSCTS)
        attrs[2] |= termios.CS8 | termios.PARENB | termios.CLOCAL | termios.CREAD
        attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        if self.fd is not None:
            try:
                if self.original is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            except (OSError, termios.error):
                pass  # A physically disconnected device may no longer accept settings.
            finally:
                os.close(self.fd)
                self.fd = None


def drain(port, receiver, started):
    # Bound each pass for fairness and responsive Ctrl+C; never discard a backlog.
    for _ in range(16):
        try:
            data = os.read(port.fd, 4096)
        except BlockingIOError:
            return
        if not data:
            poller = select.poll()
            poller.register(port.fd, select.POLLHUP | select.POLLERR)
            if poller.poll(0):
                raise OSError(f'串口断开：{port.path}')
            return  # VMIN=0 may return an empty read once the queue is drained.
        receiver.feed(data, time.monotonic() - started)


def measure(ports, timeout, expected):
    receivers = {name: Receiver() for name in ports}
    started, error, stop_reason = None, None, 'interrupted'
    stopped = False

    def stop(signum, _frame):
        nonlocal stopped, stop_reason
        stopped, stop_reason = True, signal.Signals(signum).name

    with ExitStack() as stack:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, stop)
            stack.callback(signal.signal, sig, previous)
        try:
            opened = {}
            for name, path in ports.items():
                port = SerialPort(path)
                stack.callback(port.close)
                opened[name] = port.open()
            devices = [os.fstat(port.fd).st_rdev for port in opened.values()]
            if len(set(devices)) != len(devices):
                raise ValueError('A/B 路径指向同一串口设备')
            # Acquire both gateway-compatible locks before changing either port.
            for port in opened.values():
                port.configure()
            selector = stack.enter_context(selectors.DefaultSelector())
            for name, port in opened.items():
                selector.register(port.fd, selectors.EVENT_READ, name)
            started = time.monotonic()
            print('接收就绪：现在让飞控发送；发送结束后按 Ctrl+C 汇总。', flush=True)
            next_status = started + 1
            while not stopped:
                for key, _ in selector.select(timeout=0.1):
                    drain(opened[key.data], receivers[key.data], started)
                now = time.monotonic()
                if now >= next_status:
                    elapsed = now - started
                    for name, receiver in receivers.items():
                        live = receiver.last_time is not None and elapsed - receiver.last_time <= timeout / 1000
                        print(f'{name}: {"LIVE" if live else "STALE"} '
                              f'有效帧={receiver.valid_frames}/{expected} '
                              f'校验失败候选={receiver.checksum_failures} '
                              f'计数缺口推断={receiver.counter_missing_estimate} '
                              f'同计数={receiver.same_counter_events} 歧义={receiver.ambiguous_events}', flush=True)
                    next_status = now + 1
            # Capture bytes already queued when the operator ended the test.
            for key, _ in selector.select(timeout=0):
                drain(opened[key.data], receivers[key.data], started)
        except (OSError, ValueError, termios.error) as exc:
            error, stop_reason = str(exc), 'error'
        elapsed = time.monotonic() - started if started is not None else 0
    return dict(config=str(CONFIG), ports=ports, profile='aviator-bench-v1',
                serial='115200/8E1', receive_timeout_ms=timeout,
                expected_frames_per_channel=expected, elapsed_s=elapsed,
                stop_reason=stop_reason, error=error, ready=started is not None,
                counting_note='有效帧含重复及业务非法帧；数量缺口不是已确认的唯一帧丢失数。'
                '仅在发送总数准确、完整覆盖发送区间且无重复时可按缺口计算丢包。'
                '计数缺口仅为推断；8位计数无法可靠识别整圈丢失、重启或发送端跳拍。'
                '时间为主机读取时间，合帧时不是逐帧线上到达时间。',
                channels={name: receiver.summary(expected, elapsed) for name, receiver in receivers.items()})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected', type=int, default=10000, help='飞控每路声明发送总数，默认 10000；不触发自动结束')
    parser.add_argument('--report', type=Path, help='保存 JSON 汇总及最近 1000 条异常记录；文件必须不存在')
    parser.add_argument('--dry-run', action='store_true', help='检查项目配置及依赖，不打开串口、不写文件')
    args = parser.parse_args(argv)
    if args.expected <= 0:
        parser.error('--expected 必须为正整数')
    ports, timeout = load_config(CONFIG)
    print(f'配置：{CONFIG}\n串口：115200 / 8E1；下行 32 字节；EB 90；字节 2..30 累加低 8 位')
    for name, path in ports.items():
        print(f'{name}: {path}')
    print(f'通信有效期沿用项目 {timeout} ms，仅显示 LIVE/STALE，不停止统计。')
    if args.dry_run:
        return 0
    print('请先停止占用这两路串口的 flight_gateway；本工具不发送上行，也不连接 Core。')
    with ExitStack() as stack:
        report = stack.enter_context(args.report.open('x', encoding='utf-8')) if args.report else None
        result = measure(ports, timeout, args.expected)
        print(f'\n测试结束：{result["stop_reason"]}；时长 {result["elapsed_s"]:.2f} 秒')
        if not result['ready']:
            print('未进入接收就绪，本轮统计无效。')
        for name, values in (result['channels'].items() if result['ready'] else ()):
            print(f'{name}: 声明发送={args.expected} 有效接收={values["valid_frames"]} '
                  f'数量缺口={values["count_shortfall"]} 超额={values["count_excess"]} '
                  f'有效接收率={values["reception_percent"]:.3f}%\n'
                  f'   字节={values["rx_bytes"]} 校验失败候选={values["checksum_failures"]} '
                  f'重同步丢弃字节={values["discarded_bytes"]} 残帧字节={values["trailing_partial_bytes"]}\n'
                  f'   首/末计数={values["first_counter"]}/{values["last_counter"]} '
                  f'计数缺口推断={values["counter_missing_estimate"]} 同计数={values["same_counter_events"]} '
                  f'歧义={values["ambiguous_events"]} 业务非法帧={values["semantic_invalid_frames"]}')
        print(result['counting_note'])
        if report:
            json.dump(result, report, ensure_ascii=False, indent=2)
            report.write('\n')
            print(f'报告：{args.report.absolute()}')
        if result['error']:
            print(f'测试未完整完成：{result["error"]}', file=sys.stderr)
            return 1
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'RS422 测试：{exc}', file=sys.stderr)
        sys.exit(1)
