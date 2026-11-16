#!/usr/bin/env python3
"""Observe a running AVIATOR instance and write an offline report (Python 3.9+)."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import resource
import shlex
import shutil
import signal
import subprocess
import sys
import time
import uuid
import webbrowser

VERSION = '1.2.2'
PROC = Path('/proc')
NATIVE = {
    'aviator_bus': 'bus', 'aviator_photoelectric': 'photoelectric',
    'manipulator': 'manipulator', 'simulation': 'simulation',
    'flight_gateway': 'gateway', 'aviator_core_managed': 'core',
    'aviator_monitor': 'monitor', 'aviator_logger': 'logger', 'aviator_tts': 'tts',
}
NODES = set(NATIVE.values()) | {'camera', 'hand', 'wheel_test_ui'}
METRICS = ['elapsed_s', 'wall_time', 'node', 'pid', 'start_ticks', 'sample_dt_s',
           'cpu_pct', 'rss_bytes', 'pss_bytes', 'thread_count',
           'read_bytes_per_s', 'write_bytes_per_s', 'minor_faults_per_s',
           'major_faults_per_s', 'sample_valid', 'missing_reason']
TIMING = ['observed_elapsed_s', 'source_timestamp', 'node', 'pid', 'side',
          'record_type', 'metric_name', 'value', 'unit', 'aggregation_scope',
          'window_s', 'sample_count', 'source_file', 'source_offset']
SCOPES = {
    'plan_mean_us': ('us', 'window_mean'), 'plan_max_us': ('us', 'window_max'),
    'input_age_ms': ('ms', 'snapshot'), 'feedback_age_ms': ('ms', 'snapshot'),
    'command_hz': ('Hz', 'window_rate'), 'callback_hz': ('Hz', 'window_rate'),
    'feedback_hz': ('Hz', 'window_rate'),
    'rt_max_gap_ms': ('ms', 'since_enable_max'),
    'rt_max_work_us': ('us', 'since_enable_max'),
    'max_target_error_rad': ('rad', 'snapshot_joint_max'),
    'max_actual_speed_rad_s': ('rad/s', 'snapshot_joint_max'),
    'policy': ('', 'snapshot'), 'priority': ('', 'snapshot'),
}
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def parse_stat(text):
    # comm may contain spaces and parentheses; fields after its final ')' are stable.
    end = text.rfind(')')
    fields = text[end + 2:].split()
    return dict(pid=int(text.split('(', 1)[0]), comm=text[text.index('(') + 1:end],
                state=fields[0], ppid=int(fields[1]), pgid=int(fields[2]),
                start_ticks=int(fields[19]), ticks=int(fields[11]) + int(fields[12]),
                minflt=int(fields[7]), majflt=int(fields[9]))


def process_table(proc=PROC):
    result = {}
    for directory in proc.iterdir():
        if not directory.name.isdigit():
            continue
        try:
            item = parse_stat((directory / 'stat').read_text())
            item['argv'] = (directory / 'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
            item['uid'] = directory.stat().st_uid
            try:
                item['exe'] = os.readlink(directory / 'exe')
            except OSError:
                item['exe'] = ''
            if item['state'] != 'Z':
                result[item['pid']] = item
        except (OSError, ValueError, IndexError):
            continue
    return result


def descendants(table, root):
    owned = {root}
    while True:
        added = {p for p, item in table.items() if item['ppid'] in owned} - owned
        if not added:
            return owned - {root}
        owned.update(added)


def is_launcher(item):
    argv = item['argv']
    # Match script operands, never arbitrary shell -c source or editor file arguments.
    if not argv or Path(argv[0]).name not in ('bash', 'sh', 'start_aviator.sh'):
        return False
    return '-c' not in argv[:3] and any(Path(arg).name == 'start_aviator.sh' for arg in argv[:3])


def node_name(item):
    argv = item['argv']
    name = Path(item['exe'] or (argv[0] if argv else '')).name
    if name in NATIVE:
        return NATIVE[name]
    if name.startswith('python'):
        for arg in argv[1:4]:
            path = arg.replace('\\', '/')
            if path.endswith('nodes/camera/main.py'):
                return 'camera'
            if path.endswith('rh56ftp_hand/launch.py') or path.endswith('rh56ftp_hand/__main__.py'):
                return 'hand'
            if path.endswith('wheel_accuracy/ui.py'):
                return 'wheel_test_ui'
    return None


def launcher_families(table):
    """Collapse forked shell copies, not unrelated or separately executed launchers.

    start_aviator.sh's background browser subshell keeps bash's original argv.
    Its ancestors, UID, process group and argv identify the parent launch family.
    """
    candidates = {pid for pid, item in table.items() if is_launcher(item)
                  and (item['uid'] == os.getuid() or os.geteuid() == 0)}
    families = {}
    for pid in candidates:
        item = table[pid]
        root, parent, seen = pid, item['ppid'], {pid}
        while parent in table and parent not in seen:
            seen.add(parent)
            ancestor = table[parent]
            if (parent in candidates and ancestor['uid'] == item['uid']
                    and ancestor['pgid'] == item['pgid'] and ancestor['argv'] == item['argv']):
                root = parent
            parent = ancestor['ppid']
        families[pid] = root
    return families


def stdout_log(pid, proc=PROC):
    try:
        path = Path(os.readlink(proc / str(pid) / 'fd/1'))
        if path.suffix == '.log' and path.stem in NODES:
            return path
    except OSError:
        pass
    return None


class Instance:
    def __init__(self, launcher_pid=None, log_dir=None, proc=PROC):
        self.proc = proc
        table = process_table(proc)
        families = launcher_families(table)
        candidates = sorted(set(families.values()))
        if launcher_pid is not None:
            if launcher_pid not in families:
                raise ValueError('--launcher-pid 不是可访问的 start_aviator.sh 进程')
            candidates = [families[launcher_pid]]
        else:
            # An orphaned browser shell may outlive a previous launcher. It has
            # no business descendants and must not shadow an active instance.
            active = [pid for pid in candidates if any(
                node_name(table[child]) for child in descendants(table, pid))]
            if active:
                candidates = active
        if len(candidates) != 1:
            detail = '，PID=' + ','.join(map(str, candidates)) if candidates else ''
            raise ValueError(f'发现 {len(candidates)} 个独立启动器{detail}；请先启动 AVIATOR，或用 --launcher-pid 指定实例')
        self.pid = candidates[0]
        self.start = table[self.pid]['start_ticks']
        children = descendants(table, self.pid)
        logs = {p.parent.resolve() for pid in children if (p := stdout_log(pid, proc))}
        self.log_dir = Path(log_dir).resolve() if log_dir else None
        if self.log_dir:
            if self.log_dir not in logs:
                raise ValueError('--log-dir 无法通过本次实例的 stdout 日志路径核实')
        elif len(logs) == 1:
            self.log_dir = logs.pop()
        elif len(logs) > 1:
            raise ValueError('发现多个日志目录，请用 --log-dir 指定')
        self.known = {}
        self.issues = []

    def discover(self):
        self.issues = []
        table = process_table(self.proc)
        launcher = table.get(self.pid)
        owned = descendants(table, self.pid) if launcher and launcher['start_ticks'] == self.start else set()
        # Keep already verified nodes across reparenting, but never follow a reused PID.
        owned.update(pid for pid, start in self.known.items()
                     if pid in table and table[pid]['start_ticks'] == start)
        for pid in list(owned):
            if pid in self.known:
                owned.update(descendants(table, pid))
        if self.log_dir and launcher and launcher['start_ticks'] == self.start:
            try:
                pid = int((self.log_dir / 'manipulator.pid').read_text().strip())
                item = table.get(pid)
                if (item and node_name(item) == 'manipulator' and item['pgid'] == pid
                        and item['start_ticks'] >= self.start):
                    owned.add(pid)
                    owned.update(descendants(table, pid))
                else:
                    self.issues.append('manipulator.pid 无法核实为本次有效业务进程（退出、权限或身份不符）')
            except FileNotFoundError:
                pass  # Simulation has no manipulator.pid.
            except (OSError, ValueError) as error:
                self.issues.append('manipulator.pid 不可读或无效: ' + type(error).__name__)
        nodes = {}
        for pid in owned:
            item = table.get(pid)
            if not item:
                continue
            node = node_name(item)
            if node:
                nodes[pid] = dict(item, node=node)
        # Native/Python worker descendants belong to the closest identified business node.
        changed = True
        while changed:
            changed = False
            for pid in owned - nodes.keys():
                item = table.get(pid)
                if item and item['ppid'] in nodes:
                    nodes[pid] = dict(item, node=nodes[item['ppid']]['node'])
                    changed = True
        for pid, item in nodes.items():
            self.known[pid] = item['start_ticks']
        return nodes


def key_values(path):
    result = {}
    for line in path.read_text().splitlines():
        key, _, value = line.partition(':')
        result[key] = value.strip()
    return result


def sample_process(item, proc=PROC, pss=False):
    base = proc / str(item['pid'])
    before = parse_stat((base / 'stat').read_text())
    if before['start_ticks'] != item['start_ticks'] or before['state'] == 'Z':
        raise ProcessLookupError('进程退出或 PID 已复用')
    values = dict(before, sampled_at=time.monotonic(), errors=[])
    for name, source, factor in [('rss_bytes', 'VmRSS', 1024), ('thread_count', 'Threads', 1)]:
        values[name] = None
    try:
        status = key_values(base / 'status')
        for name, source, factor in [('rss_bytes', 'VmRSS', 1024), ('thread_count', 'Threads', 1)]:
            if source in status:
                values[name] = int(status[source].split()[0]) * factor
    except (OSError, ValueError) as error:
        values['errors'].append(f'status:{type(error).__name__}')
    try:
        io = key_values(base / 'io')
        values.update({name: int(io[name]) for name in ('read_bytes', 'write_bytes')})
    except (OSError, ValueError, KeyError) as error:
        values['errors'].append(f'io:{type(error).__name__}')
    values['pss_bytes'] = None
    if pss:
        try:
            values['pss_bytes'] = int(key_values(base / 'smaps_rollup')['Pss'].split()[0]) * 1024
        except (OSError, ValueError, KeyError) as error:
            values['errors'].append(f'pss:{type(error).__name__}')
    after = parse_stat((base / 'stat').read_text())
    if after['start_ticks'] != before['start_ticks']:
        raise ProcessLookupError('读取期间 PID 已复用')
    return values


def metric_row(item, current, previous, elapsed, ticks_per_second):
    row = dict.fromkeys(METRICS)
    row.update(elapsed_s=round(elapsed, 6), wall_time=utc_now(), node=item['node'],
               pid=item['pid'], start_ticks=item['start_ticks'], sample_valid=True,
               missing_reason=';'.join(current['errors']))
    for name in ('rss_bytes', 'pss_bytes', 'thread_count'):
        row[name] = current.get(name)
    if previous and previous['start_ticks'] == current['start_ticks']:
        dt = current['sampled_at'] - previous['sampled_at']
        if dt > 0:
            row['sample_dt_s'] = dt
            for source, target, scale in [('ticks', 'cpu_pct', 100 / ticks_per_second),
                                          ('read_bytes', 'read_bytes_per_s', 1),
                                          ('write_bytes', 'write_bytes_per_s', 1),
                                          ('minflt', 'minor_faults_per_s', 1),
                                          ('majflt', 'major_faults_per_s', 1)]:
                if source in current and source in previous:
                    delta = current[source] - previous[source]
                    if delta >= 0:
                        row[target] = delta * scale / dt
                    else:
                        row['missing_reason'] += ';counter_reset:' + source
    return row


class HostSampler:
    def __init__(self, proc=PROC):
        self.proc, self.previous = proc, None

    def read(self, elapsed):
        row = {'elapsed_s': elapsed, 'cpu_busy_pct': None, 'iowait_pct': None,
               'available_bytes': None, 'missing_reason': ''}
        try:
            counts = list(map(int, (self.proc / 'stat').read_text().splitlines()[0].split()[1:9]))
            if self.previous:
                delta = [a-b for a, b in zip(counts, self.previous)]
                total = sum(delta)
                if total > 0 and all(x >= 0 for x in delta):
                    row['cpu_busy_pct'] = 100 * (total - delta[3] - delta[4]) / total
                    row['iowait_pct'] = 100 * delta[4] / total
            self.previous = counts
            row['available_bytes'] = int(key_values(self.proc / 'meminfo')['MemAvailable'].split()[0]) * 1024
        except (OSError, ValueError, IndexError, KeyError) as error:
            row['missing_reason'] = type(error).__name__
        return row


def parse_timing(line, elapsed, node, pid, source, offset):
    clean = ANSI.sub('', line)
    match = re.search(r'(Motion|Servo) timing\s+(.*)', clean)
    if not match:
        return []
    fields = dict(re.findall(r'(\w+)=([^\s]+)', match[2]))
    timestamp = re.search(r'\[(\d{4}-\d\d-\d\d [\d:.]+)\]', clean)
    rows = []
    for name, (unit, scope) in SCOPES.items():
        try:
            value = float(fields[name])
            if not math.isfinite(value):
                continue
        except (KeyError, ValueError):
            continue
        rows.append(dict(observed_elapsed_s=elapsed,
                         source_timestamp=timestamp[1] if timestamp else '', node=node, pid=pid,
                         side=fields.get('side', ''), record_type=match[1], metric_name=name,
                         value=value, unit=unit, aggregation_scope=scope,
                         window_s=fields.get('window_s', ''),
                         sample_count=fields.get('steps', '') if name.startswith('plan_') else '',
                         source_file=str(source), source_offset=offset))
    return rows


class LogTail:
    """Incremental binary tail: no startup history; bounded per-tick work and line size."""
    def __init__(self, path, node, event):
        self.path, self.node, self.event = path, node, event
        self.offset, self.inode, self.pending = 0, None, b''
        try:
            stat = path.stat()
            self.offset, self.inode = stat.st_size, (stat.st_dev, stat.st_ino)
        except OSError:
            pass

    def read(self, elapsed, pid):
        rows = []
        try:
            with self.path.open('rb') as stream:
                stat = os.fstat(stream.fileno())
                inode = (stat.st_dev, stat.st_ino)
                if self.inode is not None and (inode != self.inode or stat.st_size < self.offset):
                    self.event('log_reset', self.node, '日志轮转/截断，时间线可能有缺口')
                    self.offset, self.pending = 0, b''
                self.inode = inode
                stream.seek(self.offset)
                chunk = stream.read(64 * 1024)
                start = self.offset - len(self.pending)
                self.offset += len(chunk)
                lines = (self.pending + chunk).split(b'\n')
                self.pending = lines.pop()
                for raw in lines:
                    line = raw.decode(errors='replace')
                    found = parse_timing(line, elapsed, self.node, pid, self.path, start)
                    rows.extend(found)
                    if ' timing ' in line and ('Motion timing' in line or 'Servo timing' in line) and not found:
                        self.event('timing_parse_failed', self.node, f'offset={start}')
                    if re.search(r'\[(?:error|critical)\]', ANSI.sub('', line), re.I):
                        self.event('log_error', self.node, f'{self.path.name}:offset={start}')
                    start += len(raw) + 1
                if len(self.pending) > 65536:
                    self.pending = b''
                    self.event('log_line_too_long', self.node, '超长未完成日志行已跳过')
                if self.offset < stat.st_size:
                    self.event('log_backlog', self.node, '本轮读取限额 64 KiB；日志存在积压，时间采用读取时刻')
        except OSError as error:
            self.event('log_unavailable', self.node, type(error).__name__)
        return rows


class PerfRecorder:
    def __init__(self, output, pids, duration, frequency, elevated, event,
                 call_graph='none', stack_bytes=2048, max_mb=128, perf_bin='perf'):
        self.process = None
        self.stream = None
        self.status = 'unavailable'
        self.command = []
        self.event, self.output = event, output
        self.returncode = None
        self.finished = False
        self.duration = duration
        self.max_bytes = max_mb * 1024 * 1024
        self.stop_reason = 'duration'
        self.started = None
        self.elapsed_s = 0
        self.elevated = elevated and os.geteuid() != 0
        try:
            check = subprocess.run([perf_bin, '--version'], capture_output=True, text=True, timeout=5)
            if check.returncode:
                raise RuntimeError((check.stderr or check.stdout).strip()[:1000])
            self.version = check.stdout.strip()
            (output / 'perf.data').touch(mode=0o600)
            graph = [] if call_graph == 'none' else ['--call-graph',
                f'dwarf,{stack_bytes}' if call_graph == 'dwarf' else 'fp']
            self.command = [perf_bin, 'record', '-F', str(frequency), '-e', 'cycles:u',
                            *graph, '-p', ','.join(map(str, sorted(pids))),
                            '-o', str(output / 'perf.data'), '--', 'sleep', str(duration)]
            if elevated and os.geteuid() != 0:
                # The privileged recorder is bounded even if the unprivileged parent dies.
                self.command = ['sudo', '-n', 'timeout', '--signal=INT', '--kill-after=15s',
                                str(duration + 30), *self.command]
            self.stream = (output / 'perf.stderr.log').open('w')
            # Preserve the controlling terminal/session for sudo's tty timestamp,
            # but isolate its process group. The collector is single-threaded here.
            self.process = subprocess.Popen(
                self.command, stdout=self.stream, stderr=self.stream, stdin=subprocess.DEVNULL,
                start_new_session=not self.elevated,
                preexec_fn=os.setpgrp if self.elevated else None)
            self.started = time.monotonic()
            self.status = 'recording'
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            event('perf_unavailable', '', str(error))

    def poll(self):
        if self.finished or not self.process:
            return
        if self.process.poll() is not None:
            if self.process.returncode == 0 and time.monotonic() - self.started < self.duration - .1:
                self.stop_reason = 'early_exit'
                self.event('perf_early_exit', '', 'perf 未到请求时长便结束；基础采集继续')
            self.stop()
        elif (self.output / 'perf.data').stat().st_size >= self.max_bytes:
            self.stop_reason = 'size_limit'
            self.event('perf_size_limit', '', '达到 perf 文件软上限，结束 CPU 采样；基础采集继续')
            self.stop()

    def stop(self):
        if self.finished:
            return
        self.finished = True
        if self.started is not None:
            self.elapsed_s = time.monotonic() - self.started
        if not self.process:
            if self.stream:
                self.stream.close()
            return
        if self.process.poll() is None:
            try:
                # The resource clock starts before perf is spawned. At the natural
                # deadline, let perf's own sleep finish and flush before signalling
                # sudo; premature SIGINT can make sudo kill its wrapper on exit.
                if self.stop_reason == 'duration' and self.started is not None:
                    remaining = self.duration - (time.monotonic() - self.started)
                    if remaining <= 5:
                        try:
                            self.process.wait(timeout=max(0, remaining) + 30)
                        except subprocess.TimeoutExpired:
                            pass
                if self.process.poll() is None:
                    self.process.send_signal(signal.SIGINT)
                    self.process.wait(timeout=18)
            except subprocess.TimeoutExpired:
                self.event('perf_stop_timeout', '', '采集器未及时收尾；提权模式仍受 timeout 限时，不向业务进程发信号')
                self.status = 'incomplete'
                # The entire session belongs to this recorder, never the attached nodes.
                # For sudo, the root timeout remains the final bound if signal forwarding fails.
                try:
                    self.process.terminate()
                    self.process.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    if not self.elevated:
                        try:
                            os.killpg(self.process.pid, signal.SIGKILL)
                            self.process.wait(timeout=2)
                        except (OSError, subprocess.TimeoutExpired):
                            self.event('perf_cleanup_pending', '', '采集子任务尚未确认退出；数据不能视为完整')
                    else:
                        self.event('perf_cleanup_pending', '', '提权采集子任务尚未确认退出；受 timeout 限时，数据不完整')
            except (OSError, PermissionError) as error:
                self.event('perf_stop_error', '', str(error))
                self.status = 'incomplete'
        self.returncode = self.process.poll()
        if self.stream:
            self.stream.close()
        if self.status == 'incomplete':
            return
        self.status = 'recorded' if self.returncode in (0, -signal.SIGINT, 130) and (self.output / 'perf.data').stat().st_size else 'failed'
        if self.status == 'failed':
            self.event('perf_failed', '', f'退出码 {self.returncode}；详见 perf.stderr.log')
        if self.status == 'recorded' and self.stop_reason == 'size_limit':
            self.status = 'limited'
        stderr = (self.output / 'perf.stderr.log').read_text(errors='replace')
        if re.search(r'lost|throttl|permission|not supported', stderr, re.I):
            self.event('perf_diagnostic', '', 'perf 报告丢失/限频/权限/不支持信息；请检查原始 stderr')


class Plot:
    """Time buckets retain first/last, extrema and a gap; original CSV is never reduced."""
    def __init__(self, duration):
        self.width = max(duration / 800, 0.001)
        self.buckets = {}

    def add(self, x, y):
        if not math.isfinite(x):
            return
        index = int(x / self.width)
        bucket = self.buckets.setdefault(index, {})
        point = [x, y]
        bucket.setdefault('first', point)
        bucket['last'] = point
        if y is None:
            bucket['gap'] = point
        else:
            if 'min' not in bucket or y < bucket['min'][1]:
                bucket['min'] = point
            if 'max' not in bucket or y > bucket['max'][1]:
                bucket['max'] = point

    def points(self):
        result = []
        for index in sorted(self.buckets):
            unique = {(p[0], p[1]) for p in self.buckets[index].values()}
            result.extend([list(p) for p in sorted(unique, key=lambda p: (p[0], p[1] is None))])
        return result


def numeric(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def parse_function_report(raw):
    """Parse perf's explicit five-column stdio format; keep hybrid PMUs separate."""
    rows, event, lost = [], None, None
    for line in raw.splitlines():
        match = re.match(r"# Samples: .* of event '(.*)'", line)
        if match:
            event = match[1]
        match = re.match(r'# Total Lost Samples:\s*(\d+)', line)
        if match:
            lost = int(match[1])
        if line.lstrip().startswith('#') or ';' not in line:
            continue
        parts = line.strip().split(';', 4)
        if len(parts) != 5 or not re.fullmatch(r'[\d.]+%\s*', parts[0]):
            continue
        if event is None:
            raise ValueError('perf 输出缺少事件名称，不能混合不同 PMU 的占比')
        samples, period = int(parts[1].strip()), int(parts[2].strip())
        if samples < 0 or period < 0:
            raise ValueError('perf 计数不能为负数')
        symbol = re.sub(r'^\[[^]]+\]\s*', '', parts[4].strip())
        resolved = bool(symbol) and not re.match(r'^(?:0x[0-9a-f]+|\[unknown\])(?:$|\s)', symbol, re.I)
        rows.append(dict(event=event, dso=parts[3].strip(), symbol=symbol,
                         samples=samples, period=period, resolved=resolved))
    totals = {}
    for row in rows:
        total = totals.setdefault(row['event'], dict(samples=0, period=0, unresolved_period=0))
        total['samples'] += row['samples']
        total['period'] += row['period']
        if not row['resolved']:
            total['unresolved_period'] += row['period']
    for row in rows:
        total = totals[row['event']]['period']
        row['self_pct'] = 100 * row['period'] / total if total else 0
    for total in totals.values():
        total['unresolved_pct'] = 100 * total['unresolved_period'] / total['period'] if total['period'] else 0
    rows.sort(key=lambda row: (row['event'], -row['period']))
    return dict(rows=rows, events=totals, lost_samples=lost)


def analyze_functions(output, args, summary):
    """Explicit offline operation. Never attach, sample, authenticate or rebuild targets."""
    result = dict(status='failed', rows=[], events={}, warnings=[], symbols=[],
                  source_perf_status=summary.get('perf', {}).get('status'),
                  comm_filter=args.function_comms,
                  denominator='同一事件中，选定 comm 范围内所有已解析行的 period 之和；包含未知符号',
                  metric='Self cycles 占比；不是整机 CPU%、调用次数、单次耗时或含子调用的耗时')
    # Invalidate stale analysis before any external command; raw capture is never changed.
    write_json(output / 'functions.json', result)
    (output / 'functions.csv').unlink(missing_ok=True)
    try:
        if not (output / 'perf.data').is_file() or not (output / 'perf.data').stat().st_size:
            raise ValueError('没有 perf.data；basic 数据只能定位进程，请重新做单节点低频 CPU 采样')
        env = dict(os.environ, LC_ALL='C', DEBUGINFOD_URLS='')

        def run(command, timeout=120):
            response = subprocess.run(command, capture_output=True, text=True, errors='replace',
                                      env=env, timeout=timeout)
            if response.returncode:
                raise RuntimeError((response.stderr or response.stdout)[-3000:] or f'命令退出 {response.returncode}')
            return response

        result['perf_version'] = run([args.perf_bin, '--version'], 5).stdout.strip()
        ids = run([args.perf_bin, 'buildid-list', '-f', '-i', str(output / 'perf.data')]).stdout
        (output / 'perf.buildids.txt').write_text(ids)
        captured = {line.split()[0].lower() for line in ids.splitlines()
                    if re.match(r'^[0-9a-fA-F]{8,}\s', line)}
        cache = output / '.perf-symbol-cache'
        cache.mkdir(exist_ok=True)
        command = [args.perf_bin, '--buildid-dir', str(cache)]
        for path in args.symbol_file:
            path = path.expanduser().resolve()
            notes = run(['readelf', '-n', str(path)], 10).stdout
            match = re.search(r'Build ID:\s*([0-9a-fA-F]+)', notes)
            if not match or match[1].lower() not in captured:
                raise ValueError(f'符号文件 build ID 不匹配本次采样，拒绝加载: {path}')
            sections = run(['readelf', '-SW', str(path)], 10).stdout
            if '.symtab' not in sections:
                raise ValueError(f'符号文件没有 .symtab，请提供未剥离二进制或独立调试文件: {path}')
            run([*command, 'buildid-cache', '--add', str(path)])
            result['symbols'].append(dict(path=str(path), build_id=match[1].lower(),
                                          source_lines_available='.debug_line' in sections))
        report_command = [*command, 'report', '--stdio', '-f', '--no-children', '--no-inline',
                          '-g', 'none', '--percent-limit', '0',
                          '-F', 'overhead,sample,period,dso,symbol', '-t', ';',
                          '-i', str(output / 'perf.data')]
        if args.function_comms:
            report_command += ['--comms', args.function_comms]
        result['command'] = shlex.join(report_command)
        # Keep raw output for audits and unsupported perf format diagnostics.
        with (output / 'perf.report.txt').open('w') as stdout, \
                (output / 'perf.report.stderr.log').open('w') as stderr:
            process = subprocess.run(report_command, stdout=stdout, stderr=stderr, env=env, timeout=120)
        diagnostic = (output / 'perf.report.stderr.log').read_text(errors='replace')
        if process.returncode:
            raise RuntimeError(f'perf report 失败 ({process.returncode}): {diagnostic[-2000:]}')
        result.update(parse_function_report((output / 'perf.report.txt').read_text(errors='replace')))
        if not result['rows']:
            raise ValueError('没有可解析函数样本；检查 comm 筛选、采样完整度和 perf.report.txt 格式')
        if result['source_perf_status'] != 'recorded':
            result['warnings'].append('原始 perf 采集未标为完整；只可作为排查线索')
        if diagnostic.strip():
            result['warnings'].append('perf 有诊断输出，请检查 perf.report.stderr.log')
        if result['lost_samples'] is None or result['lost_samples']:
            result['warnings'].append('存在丢失样本或丢失计数未知')
        for event, total in result['events'].items():
            if total['samples'] < 100:
                result['warnings'].append(f'{event}: 样本少于 100，排名不稳定')
            if total['unresolved_pct'] > 0:
                result['warnings'].append(f"{event}: 未解析符号占 {total['unresolved_pct']:.2f}%")
        result['status'] = 'partial' if result['warnings'] else 'analyzed'
        fields = ['event', 'dso', 'symbol', 'samples', 'period', 'self_pct', 'resolved']
        with (output / 'functions.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(result['rows'])
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        result['status'] = 'failed'
        result['error'] = str(error)
        print(f'函数分析未完成: {error}', file=sys.stderr)
    write_json(output / 'functions.json', result)
    return result


def report_data(output, summary):
    plots, stats = {}, {}
    duration = summary['duration_s']

    def point(metric, label, x, y):
        plots.setdefault(metric, {}).setdefault(label, Plot(duration)).add(x, y)

    with (output / 'metrics.csv').open() as stream:
        for row in csv.DictReader(stream):
            label = f"{row['node']} · PID {row['pid']} @{row['start_ticks']}"
            stat = stats.setdefault(label, dict(samples=0, cpu_samples=0, cpu_weighted=0, cpu_seconds=0,
                                                cpu_peak=None, rss_first=None, rss_last=None,
                                                rss_peak=None, missing_samples=0))
            stat['samples'] += 1
            stat['missing_samples'] += bool(row['missing_reason'])
            x = float(row['elapsed_s'])
            for name in ('cpu_pct', 'rss_bytes', 'pss_bytes', 'thread_count', 'read_bytes_per_s', 'write_bytes_per_s'):
                value = numeric(row[name])
                # PSS is deliberately sparse; do not invent intermediate samples.
                if name != 'pss_bytes' or value is not None or 'pss:' in row['missing_reason']:
                    point(name, label, x, value)
            cpu, dt, rss = numeric(row['cpu_pct']), numeric(row['sample_dt_s']), numeric(row['rss_bytes'])
            if cpu is not None and dt:
                stat['cpu_samples'] += 1
                stat['cpu_weighted'] += cpu * dt
                stat['cpu_seconds'] += dt
                stat['cpu_peak'] = max(cpu, stat['cpu_peak'] or 0)
            if rss is not None:
                if stat['rss_first'] is None:
                    stat['rss_first'] = rss
                stat['rss_last'] = rss
                stat['rss_peak'] = max(rss, stat['rss_peak'] or 0)
    for stat in stats.values():
        stat['cpu_mean'] = stat.pop('cpu_weighted') / stat['cpu_seconds'] if stat['cpu_seconds'] else None
        stat['rss_delta_bytes'] = (stat['rss_last'] - stat['rss_first']) if stat['rss_first'] is not None else None
    timing_counts = {}
    with (output / 'timing.csv').open() as stream:
        for row in csv.DictReader(stream):
            name = f"{row['metric_name']} [{row['unit']}; {row['aggregation_scope']}]"
            label = f"{row['node']} {row['side']} · PID {row['pid'] or '?'}"
            point(name, label, float(row['observed_elapsed_s']), numeric(row['value']))
            timing_counts[name] = timing_counts.get(name, 0) + 1
    with (output / 'host.csv').open() as stream:
        for row in csv.DictReader(stream):
            for name in ('cpu_busy_pct', 'iowait_pct', 'available_bytes'):
                point('host_' + name, '主机（独立口径）', float(row['elapsed_s']), numeric(row[name]))
    functions = json.loads((output / 'functions.json').read_text()) if (output / 'functions.json').exists() else None
    return dict(summary=summary, stats=stats, timing_counts=timing_counts, functions=functions,
                plots={name: {label: plot.points() for label, plot in series.items()}
                       for name, series in plots.items()})


HTML = r'''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>AVIATOR 性能评估</title><link rel="icon" href="data:,">
<style>
:root{color-scheme:light}body{font:15px system-ui,sans-serif;margin:0;background:#f3f6fa;color:#17263a}
main{max-width:1250px;margin:auto;padding:28px}h1{margin:0 0 8px}h2{font-size:20px}section{background:white;border:1px solid #dbe4ee;border-radius:10px;padding:20px;margin:18px 0}
.muted{color:#536379}table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:9px;border-bottom:1px solid #e0e6ef;text-align:left}th{background:#f6f8fc}pre{white-space:pre-wrap;overflow-wrap:anywhere}select,input,button{font:inherit;padding:6px;border:1px solid #bac8d8;border-radius:5px}button{cursor:pointer;background:#e9f1ff}label{margin:5px 12px 5px 0;display:inline-block}#chart{width:100%;height:340px;display:block}#legend label{font-size:12px}.scroll{overflow:auto}.bad{color:#a32323}#hover{min-height:42px;white-space:pre-wrap;font:13px monospace}.cards{display:flex;gap:15px;flex-wrap:wrap}.cards span{padding:12px;background:#edf4ff;border-radius:5px}summary{cursor:pointer}a{color:#1657ad}
</style><main><h1>AVIATOR 性能评估</h1><div id="intro" class="muted"></div>
<section><div class="cards" id="cards"></div><p id="quality"></p><p class="muted">CPU 100% = 一个逻辑核；主机 CPU 为整机百分比（busy 含 steal，不含 idle/iowait）。RSS 不能直接相加为实际总占用。缺失值不是零。</p></section>
<section><h2>资源与控制时序趋势</h2><label>指标 <select id="metric"></select></label><label>从 <input id="from" type="number" min="0" step="0.1" value="0" size="8"> 秒</label><label>到 <input id="to" type="number" min="0" step="0.1" size="8"> 秒</label><button id="reset">重置范围</button><div id="legend"></div><svg id="chart" viewBox="0 0 1100 340" role="img" aria-label="可筛选时间序列图"></svg><div id="hover"></div><p class="muted">图表按时间桶保留首尾、极值和缺口；完整数据见 CSV。悬停查看近邻样本。时序横轴为日志读取时刻；since_enable_max 可包含采集前的历史，snapshot_joint_max 是当前快照跨关节最大值。没有逐周期样本时不计算 P99。</p></section>
<section><h2>函数 CPU 热点（离线）</h2><p id="function-quality"></p><p class="muted">Self % 是同一事件、筛选范围内的 cycles 占比，包含未知符号；不是整机 CPU%、调用次数或单次耗时。混合 CPU 的不同 PMU 分开统计。少量样本不能证明稳定排名。无调用栈时无法确定调用者。</p><label>事件 <select id="function-event"></select></label><label>函数 / 模块 <input id="function-filter" type="search"></label><div class="scroll"><table><thead><tr><th>Self %</th><th>样本数</th><th>模块</th><th>函数</th></tr></thead><tbody id="functions"></tbody></table></div><p class="muted">显示筛选后的前 100 项，完整数据见 functions.csv。原始 perf 报告见 perf.report.txt。</p></section>
<section><h2>各进程摘要</h2><div class="scroll"><table><thead><tr><th>节点 / PID / 启动批次</th><th>样本 / CPU 有效</th><th>CPU 均值 / 峰值 %</th><th>RSS 首 / 尾 / 峰 MiB</th><th>缺字段样本</th></tr></thead><tbody id="stats"></tbody></table></div></section>
<section><h2>事件与数据限制</h2><p>日志错误仅记录来源和偏移，请回原日志核实；事件不自动证明因果关系。</p><div class="scroll"><table><thead><tr><th>时间 / 秒</th><th>事件</th><th>节点</th><th>详情 / 次数</th></tr></thead><tbody id="events"></tbody></table></div></section>
<section><h2>数据文件与深度分析</h2><p><a href="metrics.csv">metrics.csv</a> · <a href="host.csv">host.csv</a> · <a href="timing.csv">timing.csv</a> · <a href="summary.json">summary.json</a> · <a href="processes.json">processes.json</a> · <a href="events.json">events.json</a></p><pre id="commands"></pre><details><summary>采集元数据</summary><pre id="metadata"></pre></details></section></main>
<script id="data" type="application/json">__DATA__</script>
<script>
'use strict';const D=JSON.parse(document.getElementById('data').textContent),S=D.summary,$=id=>document.getElementById(id);
const names={cpu_pct:'进程 CPU %',rss_bytes:'RSS / bytes',pss_bytes:'PSS / bytes（低频）',thread_count:'线程数',read_bytes_per_s:'存储读取 / bytes/s',write_bytes_per_s:'存储写入 / bytes/s',host_cpu_busy_pct:'主机 CPU busy %',host_iowait_pct:'主机 iowait %',host_available_bytes:'主机可用内存 / bytes'};
const text=(parent,tag,value)=>{const e=document.createElement(tag);e.textContent=value;parent.append(e);return e};
const fmt=v=>v==null?'缺失':Number(v).toFixed(2),mib=v=>v==null?'缺失':fmt(v/1048576);
$('intro').textContent=(S.label||'未命名场景')+' · '+S.started_at+' · '+S.kernel;
for(const x of ['模式 '+S.requested_mode,'时长 '+fmt(S.duration_s)+' 秒','进程 '+S.process_count,'结果 '+S.status])text($('cards'),'span',x);
$('quality').textContent='perf: '+S.perf.status+'；结束原因: '+S.stop_reason+'；缺失字段/异常详情见下方。';
if(S.status!=='complete')$('quality').className='bad';
for(const [label,v] of Object.entries(D.stats)){const tr=text($('stats'),'tr','');for(const x of [label,v.samples+' / '+v.cpu_samples,fmt(v.cpu_mean)+' / '+fmt(v.cpu_peak),[v.rss_first,v.rss_last,v.rss_peak].map(mib).join(' / '),v.missing_samples])text(tr,'td',x)}
for(const e of S.events){const tr=text($('events'),'tr','');for(const x of [fmt(e.elapsed_s),e.kind,e.node,e.detail+' (×'+e.count+')'])text(tr,'td',x)}
$('commands').textContent=S.perf.status==='recorded'?S.perf.open_command+'\n实际采集命令：'+S.perf.command:'本次没有完整的 perf 深度采样文件。基础指标仍可查看。';
$('metadata').textContent=JSON.stringify(S,null,2);
const F=D.functions,fe=$('function-event'),ff=$('function-filter');
$('function-quality').textContent=F?('分析状态: '+F.status+'；'+(F.error||'')+' '+(F.warnings||[]).join('；')):'尚无函数数据。basic 不能推断函数排名；CPU 采样后在开发机使用 --report 目录 --functions。';
if(F){for(const [event,v] of Object.entries(F.events||{})){const opt=text(fe,'option',event+' · '+v.samples+' 样本 · 未解析 '+fmt(v.unresolved_pct)+'%');opt.value=event}}
function drawFunctions(){ $('functions').replaceChildren();if(!F)return;const q=ff.value.toLowerCase();for(const r of (F.rows||[]).filter(r=>r.event===fe.value&&(!q||(r.symbol+' '+r.dso).toLowerCase().includes(q))).slice(0,100)){const tr=text($('functions'),'tr','');for(const v of [fmt(r.self_pct),r.samples,r.dso,r.symbol])text(tr,'td',v)}}
fe.onchange=drawFunctions;ff.oninput=drawFunctions;drawFunctions();

const colors=['#215ec7','#c83b3b','#16826c','#9b46ad','#ba6500','#007eac','#8b642b','#db4388'];
let selected=new Set(),visible=[],bounds={};const metric=$('metric');
for(const key of Object.keys(D.plots)){const opt=text(metric,'option',names[key]||key);opt.value=key}
const svg=$('chart'),NS='http://www.w3.org/2000/svg';function el(tag,attrs,content){const n=document.createElementNS(NS,tag);for(const [k,v]of Object.entries(attrs))n.setAttribute(k,v);if(content!=null)n.textContent=content;svg.append(n);return n}
function draw(){svg.replaceChildren();$('hover').textContent='';const a=Math.max(0,Number($('from').value)),b=Math.min(S.duration_s,Number($('to').value));if(!(b>a)){el('text',{x:50,y:60},'请选择有效的时间范围');return}
visible=Object.entries(D.plots[metric.value]||{}).filter(([key])=>selected.has(key));let ymax=0,has=false;
for(const [,pts]of visible)for(const [x,y]of pts)if(x>=a&&x<=b&&y!=null){ymax=Math.max(ymax,y);has=true}ymax=ymax||1;bounds={a,b,ymax};
for(let i=0;i<=5;i++){let y=20+i*52;el('line',{x1:95,y1:y,x2:1080,y2:y,stroke:'#e0e6ef'});el('text',{x:88,y:y+4,'text-anchor':'end','font-size':11},(ymax*(5-i)/5).toLocaleString(undefined,{maximumFractionDigits:2}));let x=95+i*197;el('text',{x,y:310,'text-anchor':'middle','font-size':11},fmt(a+(b-a)*i/5)+'s')}
visible.forEach(([label,pts])=>{const index=Object.keys(D.plots[metric.value]).indexOf(label);let path='',pen=false;for(const [x,y]of pts){if(x<a||x>b||y==null){pen=false;continue}path+=(pen?' L':' M')+(95+(x-a)/(b-a)*985)+' '+(280-y/ymax*260);pen=true}el('path',{d:path,fill:'none',stroke:colors[index%colors.length],'stroke-width':1.7});for(const [x,y]of pts)if(x>=a&&x<=b&&y!=null&&pts.length<4)el('circle',{cx:95+(x-a)/(b-a)*985,cy:280-y/ymax*260,r:3,fill:colors[index%colors.length]})});
if(!has)el('text',{x:150,y:100},'此范围没有有效样本');}
function choose(){selected=new Set(Object.keys(D.plots[metric.value]||{}));$('legend').replaceChildren();[...selected].forEach((key,index)=>{let lab=text($('legend'),'label','');let box=document.createElement('input');box.type='checkbox';box.checked=true;lab.append(box);let span=text(lab,'span',' '+key);span.style.color=colors[index%colors.length];box.onchange=()=>{box.checked?selected.add(key):selected.delete(key);draw()}});draw()}
svg.onmousemove=e=>{if(!bounds.b)return;let rect=svg.getBoundingClientRect(),x=(e.clientX-rect.left)/rect.width*1100,t=bounds.a+(x-95)/985*(bounds.b-bounds.a);if(t<bounds.a||t>bounds.b)return;let lines=[];for(const [label,pts]of visible){let best=null;for(const p of pts)if(p[0]>=bounds.a&&p[0]<=bounds.b&&(!best||Math.abs(p[0]-t)<Math.abs(best[0]-t)))best=p;if(best)lines.push(label+': '+fmt(best[1])+' @ '+fmt(best[0])+'s')}$('hover').textContent=lines.join('\n')};
$('to').value=S.duration_s;$('from').onchange=draw;$('to').onchange=draw;metric.onchange=choose;$('reset').onclick=()=>{$('from').value=0;$('to').value=S.duration_s;draw()};choose();
</script></html>'''


def write_report(output, summary):
    data = json.dumps(report_data(output, summary), ensure_ascii=False, allow_nan=False)
    # Prevent literal closing script tags even when data contains hostile labels.
    data = data.replace('&', '\\u0026').replace('<', '\\u003c').replace('>', '\\u003e')
    data = data.replace('\u2028', '\\u2028').replace('\u2029', '\\u2029')
    (output / 'report.html').write_text(HTML.replace('__DATA__', data))


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='AVIATOR 独立性能评估；不启动或停止业务节点')
    parser.add_argument('--version', action='version', version=VERSION)
    parser.add_argument('--duration', type=float, default=300, help='采集秒数，0.1..86400，默认 300')
    parser.add_argument('--interval', type=float, default=2, help='资源间隔秒数，0.1..60，默认 2')
    parser.add_argument('--launcher-pid', type=int)
    parser.add_argument('--log-dir', type=Path)
    parser.add_argument('--nodes', help='逗号分隔节点名称；默认所有识别到的节点')
    parser.add_argument('--output', type=Path, help='新建结果目录，拒绝覆盖')
    parser.add_argument('--label', default='')
    parser.add_argument('--cpu', action='store_true', help='额外采集 CPU 热点；默认只采 Core，不采调用栈')
    parser.add_argument('--perf-sudo', action='store_true', help='启动前自动 sudo 验证，仅 perf 子任务提权')
    parser.add_argument('--perf-nodes', default='core', help='CPU 采样节点，逗号分隔，默认 core；独立于基础监控')
    parser.add_argument('--perf-duration', type=float, default=30, help='CPU 采样秒数，最多为总时长，默认 30')
    parser.add_argument('--frequency', type=int, default=19, help='请求 perf 频率，1..999，默认 19')
    parser.add_argument('--call-graph', choices=['none', 'fp', 'dwarf'], default='none',
                        help='默认 none；fp 要求保留帧指针；dwarf 有额外栈复制开销')
    parser.add_argument('--stack-bytes', type=int, choices=[1024, 2048, 4096, 8192], default=2048,
                        help='DWARF 每次采样栈字节数，默认 2048，较小的栈可能截断调用链')
    parser.add_argument('--perf-max-mb', type=int, default=128,
                        help='perf 文件软上限 MiB，默认 128；按资源间隔检查，收尾可能超出')
    parser.add_argument('--pss', action='store_true', help='每 5 秒读取一次 PSS，权限不足留空')
    parser.add_argument('--open', action='store_true', help='结束后尝试打开离线报告')
    parser.add_argument('--collect-only', action='store_true', help='只保存原始数据，稍后在开发机 --report')
    parser.add_argument('--report', type=Path, metavar='RESULT_DIR', help='仅从已有结果生成 HTML，不发现或采集业务进程')
    parser.add_argument('--functions', action='store_true', help='配合 --report 离线解析 perf，输出函数排名和 CSV')
    parser.add_argument('--symbol-file', type=Path, action='append', default=[],
                        help='离线符号文件，可重复；必须与 perf 中 build ID 匹配且包含符号表')
    parser.add_argument('--function-comms', help='离线函数筛选，逗号分隔 perf comm；默认所有采样任务')
    parser.add_argument('--perf-bin', default='perf', help='perf 可执行文件路径；可用于离线分析机内核包装器不可用时')
    args = parser.parse_args(argv)
    for name, low, high in [('duration', .1, 86400), ('interval', .1, 60), ('frequency', 1, 999),
                          ('perf_duration', .1, 300), ('perf_max_mb', 1, 4096)]:
        if not low <= getattr(args, name) <= high:
            parser.error(f'--{name} 必须在 {low}..{high} 范围内')
    if args.perf_sudo and not args.cpu:
        parser.error('--perf-sudo 需要 --cpu')
    if args.functions and not args.report:
        parser.error('--functions 必须配合 --report；函数解析只允许离线运行')
    if (args.symbol_file or args.function_comms) and not args.functions:
        parser.error('--symbol-file / --function-comms 需要 --functions')
    if args.collect_only and (args.open or args.report):
        parser.error('--collect-only 不能与 --open 或 --report 同时使用')
    if args.report and (args.cpu or args.output or args.launcher_pid or args.log_dir or args.nodes or args.pss):
        parser.error('--report 为独立离线模式，不能同时指定采集参数')
    args.selected = set(args.nodes.split(',')) if args.nodes else None
    if args.selected and args.selected - NODES:
        parser.error('未知节点: ' + ','.join(sorted(args.selected - NODES)))
    args.perf_selected = set(args.perf_nodes.split(','))
    if args.perf_selected - NODES:
        parser.error('未知 perf 节点: ' + ','.join(sorted(args.perf_selected - NODES)))
    return args


def lower_priority():
    """Only change the evaluator's own scheduling; never touch business PIDs."""
    result = {}
    try:
        os.setpriority(os.PRIO_PROCESS, 0, max(10, os.getpriority(os.PRIO_PROCESS, 0)))
        result['nice'] = os.getpriority(os.PRIO_PROCESS, 0)
    except (AttributeError, OSError) as error:
        result['nice_error'] = str(error)
    try:
        check = subprocess.run(['ionice', '-c', '2', '-n', '7', '-p', str(os.getpid())],
                               capture_output=True, text=True, timeout=2)
        result['io_priority'] = 'best-effort/7' if check.returncode == 0 else check.stderr.strip()
    except (OSError, subprocess.TimeoutExpired) as error:
        result['io_priority_error'] = str(error)
    return result


def present_report(output, open_browser):
    print(f"报告: {output / 'report.html'}", flush=True)
    if open_browser:
        if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
            try:
                webbrowser.open((output / 'report.html').as_uri())
            except webbrowser.Error as error:
                print(f'无法打开浏览器: {error}', file=sys.stderr)
        else:
            print('没有桌面会话；可复制整个结果目录到开发机打开 report.html')


def collect(args):
    instance = Instance(args.launcher_pid, args.log_dir)
    initial = instance.discover()
    if args.selected:
        missing = args.selected - {p['node'] for p in initial.values()}
        if missing:
            raise ValueError('所选节点未发现: ' + ','.join(sorted(missing)))
        initial = {pid: p for pid, p in initial.items() if p['node'] in args.selected}
    if not initial:
        raise ValueError('没有识别到业务节点；请等待启动完成后重试')
    perf_targets = {pid: p for pid, p in initial.items() if p['node'] in args.perf_selected}
    if args.cpu:
        missing = args.perf_selected - {p['node'] for p in perf_targets.values()}
        if missing:
            raise ValueError('CPU 采样节点不在监控范围或未发现: ' + ','.join(sorted(missing)) +
                             '；请用 --perf-nodes 明确指定，不自动扩大采样范围')
    state_home = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state')))
    output = (args.output or state_home / 'aviator-evaluate/runs' /
              (datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8])).resolve()
    output.mkdir(parents=True, mode=0o700, exist_ok=False)
    print(f'实例 PID {instance.pid}；日志目录 {instance.log_dir or "不可见"}', flush=True)
    print('采集节点: ' + ', '.join(f"{p['node']}={pid}" for pid, p in sorted(initial.items())), flush=True)
    print(f'结果目录: {output}', flush=True)
    print('轻量资源采集；评估进程降低优先级，不调整业务进程。', flush=True)
    if args.cpu:
        print(f'CPU 采样: {args.perf_nodes}；{args.frequency} Hz；'
              f'{min(args.perf_duration, args.duration):g}s；调用栈 {args.call_graph}；'
              '采样仍有运行时开销。', flush=True)
    began = time.monotonic()
    own_usage_start = resource.getrusage(resource.RUSAGE_SELF)
    events, event_index, identities, previous = [], {}, {}, {}
    interrupted = []
    suppressed_events = 0
    summary = dict(schema_version=1, collector_version=VERSION, started_at=utc_now(),
                   label=args.label, kernel=os.uname().release, logical_cpus=os.cpu_count(),
                   requested_mode='cpu' if args.cpu else 'basic', requested_duration_s=args.duration,
                   interval_s=args.interval, pss_interval_s=5 if args.pss else None,
                   discovery_interval_s=max(5, args.interval),
                   priority=getattr(args, 'priority', {}), report_deferred=args.collect_only,
                   launcher_pid=instance.pid, launcher_start_ticks=instance.start,
                   log_dir=str(instance.log_dir) if instance.log_dir else None,
                   boot_id=(PROC / 'sys/kernel/random/boot_id').read_text().strip(),
                   limits=['不提供逐周期 P99、端到端延迟或泄漏判定',
                           '时序图使用日志读取时刻；累计最大值可包含本次采集前历史',
                           '不采集线程级上下文切换和物理磁盘延迟；存储 I/O 不代表 fsync 完成',
                           '未自动读取或归档构建身份和业务配置；对比前需自行固定负载并归档'],
                   status='complete', stop_reason='duration')

    def event(kind, node, detail):
        nonlocal suppressed_events
        key = (kind, node, detail)
        if key in event_index:
            events[event_index[key]]['count'] += 1
            events[event_index[key]]['last_elapsed_s'] = time.monotonic() - began
        elif len(events) < 3000:
            event_index[key] = len(events)
            events.append(dict(elapsed_s=time.monotonic() - began, kind=kind, node=node,
                               detail=detail, count=1))
            if kind not in ('process_discovered',):
                print(f'[{kind}] {node}: {detail}', file=sys.stderr, flush=True)
        else:
            suppressed_events += 1

    def stopped(signum, frame):
        interrupted.append(signum)

    old_handlers = {sig: signal.signal(sig, stopped) for sig in (signal.SIGINT, signal.SIGTERM)}
    handles, writers = [], {}
    recorder = None
    timing_count, previous_ids = 0, set()
    tails = {}
    try:
        for name, fields in [('metrics', METRICS), ('timing', TIMING),
                             ('host', ['elapsed_s', 'cpu_busy_pct', 'iowait_pct', 'available_bytes', 'missing_reason'])]:
            stream = (output / (name + '.csv')).open('w', newline='')
            handles.append(stream)
            writers[name] = csv.DictWriter(stream, fieldnames=fields)
            writers[name].writeheader()
        if not instance.log_dir:
            event('log_directory_unavailable', '', '无法定位日志目录；仅采集资源')
        else:
            for node in {p['node'] for p in initial.values()}:
                tails[node] = LogTail(instance.log_dir / (node + '.log'), node, event)
        if args.cpu:
            recorder = PerfRecorder(output, perf_targets, min(args.perf_duration, args.duration),
                                    args.frequency, args.perf_sudo, event,
                                    args.call_graph, args.stack_bytes, args.perf_max_mb, args.perf_bin)
        sampler = HostSampler()
        ticks = os.sysconf('SC_CLK_TCK')
        next_tick, next_discovery, next_pss, next_print = began, began, began, began + 10
        nodes = initial
        while True:
            now = time.monotonic()
            if interrupted:
                summary.update(status='partial', stop_reason='interrupted')
                event('interrupted', '', f'signal={interrupted[0]}')
                break
            if now >= next_discovery:
                nodes = instance.discover()
                for issue in instance.issues:
                    event('discovery_incomplete', '', issue)
                if args.selected:
                    nodes = {pid: p for pid, p in nodes.items() if p['node'] in args.selected}
                next_discovery = now + max(5, args.interval)
            elapsed = now - began
            ids = {(p['pid'], p['start_ticks']) for p in nodes.values()}
            for identity in previous_ids - ids:
                old = identities[identity]
                event('process_exit', old['node'], f'PID {identity[0]} 退出或不可见')
                writers['metrics'].writerow(dict(elapsed_s=elapsed, wall_time=utc_now(),
                    node=old['node'], pid=identity[0], start_ticks=identity[1],
                    sample_valid=False, missing_reason='process_exit'))
            for pid, item in nodes.items():
                identity = (pid, item['start_ticks'])
                if identity not in identities:
                    identities[identity] = dict(node=item['node'], pid=pid, start_ticks=item['start_ticks'],
                        ppid=item['ppid'], pgid=item['pgid'], executable=item['exe'] or item['comm'],
                        first_elapsed_s=elapsed, uid=item['uid'])
                    event('process_discovered', item['node'], f'PID {pid}')
                    if recorder and not recorder.finished and item['node'] in args.perf_selected and pid not in perf_targets:
                        event('perf_coverage_uncertain', item['node'], f'新增 PID {pid}；未验证是否被 perf 继承覆盖')
                identities[identity]['last_elapsed_s'] = elapsed
                try:
                    current = sample_process(item, pss=args.pss and now >= next_pss)
                    row = metric_row(item, current, previous.get(identity), elapsed, ticks)
                    previous[identity] = current
                    if row['missing_reason']:
                        event('metric_missing', item['node'], row['missing_reason'])
                except (OSError, ValueError, IndexError) as error:
                    previous.pop(identity, None)
                    row = dict(elapsed_s=elapsed, wall_time=utc_now(), node=item['node'], pid=pid,
                               start_ticks=item['start_ticks'], sample_valid=False,
                               missing_reason=type(error).__name__)
                    event('process_unreadable', item['node'], f'PID {pid}: {type(error).__name__}')
                writers['metrics'].writerow(row)
                if instance.log_dir and item['node'] not in tails:
                    tails[item['node']] = LogTail(instance.log_dir / (item['node'] + '.log'), item['node'], event)
            previous_ids = ids
            for node, tail in tails.items():
                pids = [pid for pid, p in nodes.items() if p['node'] == node]
                rows = tail.read(elapsed, pids[0] if len(pids) == 1 else '')
                writers['timing'].writerows(rows)
                timing_count += len(rows)
            host = sampler.read(elapsed)
            writers['host'].writerow(host)
            if host['missing_reason']:
                event('host_metric_missing', '', host['missing_reason'])
            for stream in handles:
                stream.flush()
            if now >= next_pss:
                next_pss = now + 5
            if recorder:
                recorder.poll()
            if not nodes:
                summary.update(status='partial', stop_reason='all_targets_exited')
                break
            if elapsed >= args.duration:
                break
            if now >= next_print:
                print(f'已采集 {elapsed:.0f}s；{len(nodes)} 个进程；时序字段 {timing_count}', flush=True)
                next_print = now + 10
            if shutil.disk_usage(output).free < 16 * 1024 * 1024:
                raise OSError('剩余空间不足 16 MiB，停止采集')
            next_tick += args.interval
            delay = min(next_tick, began + args.duration) - time.monotonic()
            if delay < 0:
                next_tick = time.monotonic()
                event('sampling_late', '', '采集超出计划间隔；速率采用实际时间差')
            else:
                # Short sleeps allow signal-driven interruption without touching business tasks.
                until = time.monotonic() + delay
                while not interrupted and time.monotonic() < until:
                    time.sleep(min(.1, max(0, until - time.monotonic())))
    except (OSError, RuntimeError, ValueError) as error:
        summary.update(status='partial', stop_reason='collection_error')
        event('collection_error', '', str(error))
    finally:
        summary['duration_s'] = time.monotonic() - began
        own_usage_end = resource.getrusage(resource.RUSAGE_SELF)
        summary['collector'] = dict(
            cpu_seconds=(own_usage_end.ru_utime + own_usage_end.ru_stime
                         - own_usage_start.ru_utime - own_usage_start.ru_stime),
            max_rss_bytes=own_usage_end.ru_maxrss * 1024,
            scope='采集器自身至采集结束；不含报告生成和 perf；RSS 为进程启动以来峰值')
        if recorder:
            if interrupted or summary['stop_reason'] != 'duration':
                recorder.stop_reason = summary['stop_reason'] if not recorder.finished else recorder.stop_reason
            recorder.stop()
        for stream in handles:
            stream.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    if not timing_count:
        event('no_timing_samples', '', '没有新时序样本；不推断为零延迟或控制正常')
    summary.update(ended_at=utc_now(), process_count=len(identities), timing_field_count=timing_count,
                   suppressed_event_count=suppressed_events,
                   events=events, perf=dict(status=recorder.status if recorder else 'not_requested',
                       command=shlex.join(recorder.command) if recorder else '',
                       version=getattr(recorder, 'version', None),
                       returncode=recorder.returncode if recorder else None,
                       nodes=sorted(args.perf_selected) if recorder else [],
                       pids=sorted(perf_targets) if recorder else [],
                       requested_duration_s=min(args.duration, args.perf_duration) if recorder else 0,
                       frequency=args.frequency if recorder else None,
                       call_graph=args.call_graph if recorder else None,
                       stack_bytes=args.stack_bytes if recorder and args.call_graph == 'dwarf' else None,
                       max_mb=args.perf_max_mb if recorder else None,
                       stop_reason=recorder.stop_reason if recorder else None,
                       observed_elapsed_s=recorder.elapsed_s if recorder else 0,
                       open_command=shlex.join(['hotspot', str(output / 'perf.data')]),
                       data_path=str(output / 'perf.data')))
    if any(e['kind'] not in ('process_discovered', 'no_timing_samples') for e in events):
        summary['status'] = 'partial'
    write_json(output / 'processes.json', list(identities.values()))
    write_json(output / 'events.json', events)
    write_json(output / 'summary.json', summary)
    print(f"评估结束：{summary['status']}；数据 {output}", flush=True)
    if args.collect_only:
        print('离线生成报告: python3 evaluate_aviator.py --report <复制后的结果目录> --open')
    else:
        write_report(output, summary)
        present_report(output, args.open)
    return 0 if summary['status'] == 'complete' else 2


def main(argv=None):
    try:
        args = arguments(argv)
        args.priority = lower_priority()
        if args.report:
            output = args.report.expanduser().resolve()
            summary = json.loads((output / 'summary.json').read_text())
            if summary.get('schema_version') != 1:
                raise ValueError('不支持的结果格式')
            analysis = analyze_functions(output, args, summary) if args.functions else None
            write_report(output, summary)
            present_report(output, args.open)
            return 2 if analysis and analysis['status'] != 'analyzed' else 0
        if args.perf_sudo and os.geteuid() != 0:
            # Authenticate before the collection clock starts. No shell or target PID is involved.
            command = ['sudo', '-v'] if sys.stdin.isatty() else ['sudo', '-n', '-v']
            if subprocess.run(command, timeout=60).returncode:
                raise RuntimeError('sudo 认证失败；可去掉 --cpu --perf-sudo 使用无需提权的轻量模式')
        return collect(args)
    except subprocess.TimeoutExpired:
        print('评估失败: sudo 认证超时；未开始采集', file=sys.stderr)
        return 1
    except (OSError, ValueError, RuntimeError) as error:
        print(f'评估失败: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
