#!/usr/bin/env python3
"""Hand-only grip UI using the production SDK transport, scheduler and policy."""
import argparse
import json
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'nodes/xCoreSDK_hand'))
from main import SdkHandNode, FakeSdkHandNode, prepare_sdk_directory
from scheduler import Scheduler
from transport import HandLink, SdkConnection
from rh56ftp_node import DEFAULT_SAFE_POSE, monotonic_us
import yaml
from fault_journal import FaultJournal, TZ
from datetime import datetime


class DemoController:
    """One independent policy owner per selected hand; no arm/bus dependency."""
    def __init__(self, system, library, fake=False, log_file=None):
        self.system = Path(system).resolve()
        self.library = Path(library).resolve()
        self.fake = fake
        self.schedulers = {}
        self.session, self.epoch = str(uuid.uuid4()), str(uuid.uuid4())
        self.sequence = 0
        self.errors = {}
        self.connections = {}
        self.sdk_prepared = False
        self.faults = FaultJournal(log_file or ROOT / 'logs/hand_grip_demo' /
            (datetime.now(TZ).strftime('%Y%m%d-%H%M%S-%f') + '-faults.log'))

    def connect(self, selection):
        if self.schedulers:
            raise RuntimeError('请先断开，再切换测试范围')
        if selection not in ('left', 'right', 'both'):
            raise ValueError('Invalid hand selection')
        root = yaml.safe_load(self.system.read_text())
        settings_path = self.system.parent / root.get('xcore_sdk_hand', {}).get('config', 'rh56ftp_hand.yaml')
        settings = yaml.safe_load(settings_path.read_text())
        self.poses = root['core_hand']
        robot = yaml.safe_load((self.system.parent / root['robot']).read_text())['rokae'] if not self.fake else {}
        sides = ('left', 'right') if selection == 'both' else (selection,)
        if not self.fake:
            if not self.library.is_file():
                raise FileNotFoundError(f'请先构建 aviator_xcore_hand_sdk：{self.library}')
            if not self.sdk_prepared:
                prepare_sdk_directory()
                self.sdk_prepared = True
        self.errors.clear()
        for side in sides:
            scheduler = None
            try:
                if self.fake:
                    from fake_rh56ftp_hand import FakeHandLink
                    link = FakeHandLink('simulated-' + side, motion_rate=settings.get('motion_rate', 2))
                else:
                    key = (side, robot[side + '_ip'], settings.get('rs485_timeout_ms', 50))
                    if key not in self.connections:
                        self.connections[key] = SdkConnection(self.library, key[1], key[2])
                    link = HandLink(self.connections[key], settings.get(side + '_id', 1))
                links = {s: link if s == side else None for s in ('left', 'right')}
                def setting(name, default):
                    value = settings.get(name, default)
                    return value.get('grasp', default) if isinstance(value, dict) else value
                node = (FakeSdkHandNode if self.fake else SdkHandNode)(links,
                    speed=setting('speed', 800), force=setting('force', 2000),
                    command_timeout_ms=settings.get('command_timeout_ms', 100),
                    feedback_timeout_ms=settings.get('feedback_timeout_ms', 500),
                    closing_hold_ms=settings.get('closing_hold_ms', 10000),
                    closing_motion_raw=settings.get('threshold', 10))
                scheduler = Scheduler(node, {**settings, 'error_read_hz': 20},
                                      error_observer=self.faults.observe)
                scheduler.start()
                self.schedulers[side] = scheduler
            except Exception as exc:
                self.errors[side] = str(exc)
                if scheduler:
                    scheduler.close()
        if not self.schedulers:
            raise RuntimeError(str(self.errors))

    def action(self, name):
        if name not in ('open', 'close'):
            raise ValueError('Invalid action')
        if not self.schedulers:
            raise RuntimeError('请先连接所选手')
        # Every click is a new action; delivery retries would keep this ID.
        action_id = str(uuid.uuid4())
        for side, scheduler in self.schedulers.items():
            try:
                target = self.poses.get(name, {}).get(side)
                if target is None:
                    if name == 'close':
                        raise ValueError(f'core_hand.close.{side} 未配置')
                    target = list(DEFAULT_SAFE_POSE)
                now = monotonic_us()
                self.sequence += 1
                command = dict(msg_type='HandCommand', version='1.0', sequence=self.sequence,
                    timestamp=time.time_ns() // 1000, sample_mono_us=now,
                    clock_id=scheduler.node.clock_id, publisher_id='hand_grip_demo',
                    session_id=self.session, control_epoch=self.epoch, valid=True,
                    mode='NORMALIZED_POSITION', action_id=action_id,
                    origin=dict(publisher_id='hand_grip_demo', session_id=self.session,
                                sequence=self.sequence, sample_mono_us=now, clock_id=scheduler.node.clock_id),
                    hands={s: dict(drive_position_normalized=target if s == side else list(DEFAULT_SAFE_POSE))
                           for s in ('left', 'right')})
                if not scheduler.receive('hand.command', json.dumps(command)):
                    raise RuntimeError('动作未被接收，请查看日志')
                self.errors.pop(side, None)
            except Exception as exc:
                self.errors[side] = str(exc)

    def status(self):
        result = {}
        for side, scheduler in self.schedulers.items():
            state = scheduler.make_state()
            hand = state['hands'][side]
            result[side] = dict(hand, command_valid=state['command_valid'],
                diagnostic=self.errors.get(side) or str(scheduler.failure or '') or
                state.get('hold_control_error') or scheduler.view.last_command_error or
                scheduler.view.last_write_error)
        history, log_error = self.faults.snapshot()
        return dict(hands=result, errors=dict(self.errors), faults=history, log_error=log_error)

    def close(self):
        for side, scheduler in list(self.schedulers.items()):
            try:
                scheduler.close()
            except Exception as exc:
                self.errors[side] = str(exc)
        self.schedulers.clear()


def main(argv=None):
    parser = argparse.ArgumentParser(description='RH56FTP 握持测试 UI（仅控制手，不发送机械臂运动指令）')
    parser.add_argument('--system-config', type=Path, default=ROOT / 'config/system.yaml')
    parser.add_argument('--sdk-library', type=Path, default=ROOT / 'build/lib/libaviator_xcore_hand_sdk.so')
    parser.add_argument('--fake-hand', action='store_true', help='离线模拟手，无设备连接')
    parser.add_argument('--log-file', type=Path, help='故障日志路径（默认 logs/hand_grip_demo/时间-faults.log）')
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='left')
    args = parser.parse_args(argv)
    import tkinter as tk
    from tkinter import ttk
    controller = DemoController(args.system_config, args.sdk_library, args.fake_hand, args.log_file)
    jobs, updates = queue.Queue(), queue.Queue()
    def worker():
        while True:
            try:
                operation, value = jobs.get(timeout=.1)
            except queue.Empty:
                operation, value = 'status', None
            try:
                if operation == 'quit':
                    controller.close()
                    updates.put(('closed', None))
                    return
                if operation == 'connect': controller.connect(value)
                elif operation == 'disconnect': controller.close()
                elif operation == 'action': controller.action(value)
                updates.put(('state', controller.status()))
            except Exception as exc:
                updates.put(('error', str(exc)))
    thread = threading.Thread(target=worker, name='grip-demo')
    window = tk.Tk()
    thread.start()
    window.title('RH56FTP · 手部握持测试' + (' · 模拟' if args.fake_hand else ' · 真机'))
    window.geometry('1080x780')
    window.minsize(900, 720)
    frame = ttk.Frame(window, padding=20)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='手部握持测试', font=('', 20, 'bold')).pack(anchor='w')
    ttk.Label(frame, text='连接时张开所选手；闭合后自动监测，六通道满足判据后整手暂停。断开或退出时张开。').pack(anchor='w', pady=(8, 16))
    toolbar = ttk.Frame(frame)
    toolbar.pack(fill='x')
    selection = tk.StringVar(value=args.side)
    radios = []
    for text, value in (('仅左手', 'left'), ('仅右手', 'right'), ('双手', 'both')):
        radio = ttk.Radiobutton(toolbar, text=text, value=value, variable=selection)
        radio.pack(side='left', padx=(0, 16)); radios.append(radio)
    def connect():
        for radio in radios: radio.config(state='disabled')
        connect_button.config(state='disabled')
        message.set('正在连接所选手…')
        jobs.put(('connect', selection.get()))
    connect_button = ttk.Button(toolbar, text='连接', command=connect)
    connect_button.pack(side='left', padx=5)
    ttk.Button(toolbar, text='断开', command=lambda: jobs.put(('disconnect', None))).pack(side='left', padx=5)
    buttons = ttk.Frame(frame)
    buttons.pack(fill='x', pady=20)
    for text, value in (('张开', 'open'), ('闭合', 'close')):
        ttk.Button(buttons, text=text, command=lambda v=value: jobs.put(('action', v))).pack(side='left', ipadx=35, ipady=12, padx=(0, 15))
    labels = {}
    for side, title in (('left', '左手'), ('right', '右手')):
        box = ttk.LabelFrame(frame, text=title, padding=12)
        box.pack(fill='x', pady=6)
        labels[side] = tk.StringVar(value='未选择 / 未连接')
        ttk.Label(box, textvariable=labels[side], justify='left', wraplength=820).pack(anchor='w')
    message = tk.StringVar(value='请选择测试范围并连接。单手测试不连接另一只手。')
    ttk.Label(frame, textvariable=message, wraplength=840).pack(anchor='w', pady=14)
    fault_box = ttk.LabelFrame(frame, text='故障记录 · 每 50 ms 读取 · 无故障不记录 · 最近 1000 条', padding=8)
    fault_box.pack(fill='both', expand=True)
    columns = ('time', 'side', 'channel', 'code', 'description')
    fault_table = ttk.Treeview(fault_box, columns=columns, show='headings', height=8)
    for name, title, width in zip(columns, ('故障时间（北京时间）', '手侧', '通道', '故障码', '故障说明'),
                                  (250, 60, 90, 95, 420)):
        fault_table.heading(name, text=title)
        fault_table.column(name, width=width, stretch=name in ('time', 'description'))
    scroll = ttk.Scrollbar(fault_box, orient='vertical', command=fault_table.yview)
    fault_table.configure(yscrollcommand=scroll.set)
    scroll.pack(side='right', fill='y')
    fault_table.pack(side='left', fill='both', expand=True)
    log_status = tk.StringVar(value=f'故障日志：{controller.faults.path}（有故障时创建）')
    ttk.Label(frame, textvariable=log_status, wraplength=1000).pack(anchor='w', pady=(8, 0))
    last_fault_id = 0
    closing = False
    def quit_ui():
        nonlocal closing
        if not closing:
            closing = True
            message.set('正在张开并关闭所选手，请稍候…')
            jobs.put(('quit', None))
    window.protocol('WM_DELETE_WINDOW', quit_ui)
    signal.signal(signal.SIGINT, lambda *_: quit_ui())
    signal.signal(signal.SIGTERM, lambda *_: quit_ui())
    def poll():
        nonlocal last_fault_id
        while True:
            try: kind, data = updates.get_nowait()
            except queue.Empty: break
            if kind == 'closed':
                window.destroy(); return
            if kind == 'error':
                message.set(data)
                if not controller.schedulers:
                    connect_button.config(state='normal')
                    for radio in radios: radio.config(state='normal')
            else:
                for fault in data['faults']:
                    if fault['id'] <= last_fault_id:
                        continue
                    code = fault['code']
                    fault_table.insert('', 'end', values=(fault['time'], fault['side'], fault['channel'],
                        f'{code} / 0x{code:02X}' if code is not None else '未读到', fault['description']))
                    last_fault_id = fault['id']
                rows = fault_table.get_children()
                for row in rows[:-1000]: fault_table.delete(row)
                log_status.set(data['log_error'] or f'故障日志：{controller.faults.path}（有故障时创建）')
                connected = bool(data['hands'])
                if not connected and not closing:
                    connect_button.config(state='normal')
                    for radio in radios: radio.config(state='normal')
                for side, label in labels.items():
                    hand = data['hands'].get(side)
                    if not hand:
                        label.set(data['errors'].get(side, '未选择 / 未连接')); continue
                    ready = hand['stop_criteria_met']
                    positions = hand.get('drive_position_normalized', [])
                    phase = '暂停保持' if hand['paused'] else '动作监测中' if hand['command_valid'] else '待命'
                    actual = ', '.join(f'{v:.3f}' for v in positions)
                    label.set(f'{phase}  |  停止判据 {sum(ready)}/6  |  反馈 {hand.get("status", "")}\n'
                              f'位置（侧摆、拇指、食指、中指、无名指、小指）：{actual}\n'
                              f'各通道判据：{" / ".join("满足" if r else "观察中" for r in ready)}'
                              + (f'\n诊断：{hand["diagnostic"]}' if hand['diagnostic'] else ''))
                if connected and not closing: message.set('已连接。可点击张开或闭合；再次点击会发起新的动作。')
        window.after(100, poll)
    poll()
    try:
        window.mainloop()
    finally:
        if thread.is_alive(): jobs.put(('quit', None))
        thread.join()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
