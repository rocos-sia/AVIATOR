#!/usr/bin/env python3
"""Launch the Modbus TCP hand node with the selected system bus/configuration."""
import argparse
import inspect
from pathlib import Path

import yaml


def node_options(system_file, hand_file=None, fake=None):
    system_file = Path(system_file).resolve()
    system = yaml.safe_load(system_file.read_text())
    if system.get('xcore_sdk_hand', {}).get('enabled', False):
        raise ValueError('Modbus 手节点要求 xcore_sdk_hand.enabled=false（Core 必须持续发送命令）')
    section = system.get('rh56ftp_hand', {})
    if not section.get('enabled', True):
        raise ValueError('rh56ftp_hand.enabled=false，未启动手节点')
    hand_file = (Path(hand_file).resolve() if hand_file else
                 (system_file.parent / section.get('config', 'rh56ftp_hand.yaml')).resolve())
    settings = yaml.safe_load(hand_file.read_text())
    core = system.get('core_hand', {})
    if core.get('enabled', False) and not 200 <= core.get('feedback_timeout_ms', 500) <= 1000:
        raise ValueError('core_hand.feedback_timeout_ms 必须为 200..1000，避免 Core 禁用手控制')
    args = ['--config', str(hand_file), '--endpoint', system['bus']['subscribe'],
            '--state-endpoint', system['bus']['publish'],
            '--publisher-id', core.get('publisher_id', 'rh56ftp_hand')]
    # The standalone Modbus node disables a side through its host, not SDK flags.
    for side in ('left', 'right'):
        if not settings.get(side + '_enabled', True):
            args.extend(['--' + side + '-host', ''])
    return args, section.get('fake_hand', False) if fake is None else fake


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system-config', type=Path, required=True)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--fake-hand', action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument('--check-config', action='store_true', help='检查配置与依赖，不连接设备')
    args = parser.parse_args(argv)
    options, fake = node_options(args.system_config, args.config, args.fake_hand)
    # Import/check dependencies before any hardware connection, including preflight.
    import zmq
    from rh56ftp_node import load_handlink, main as run_node
    if not fake:
        from pymodbus.client import ModbusTcpClient
        if 'device_id' not in inspect.signature(ModbusTcpClient.read_holding_registers).parameters:
            raise RuntimeError('Modbus 手节点需要支持 device_id 的 pymodbus，请安装 pymodbus==3.11.4')
        load_handlink()
    if args.check_config:
        print('rh56ftp_hand: Modbus TCP 配置与依赖检查通过' + ('（模拟手）' if fake else ''))
        return 0
    return run_node(options, simulated=fake)


if __name__ == '__main__':
    raise SystemExit(main())
