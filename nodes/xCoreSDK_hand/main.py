#!/usr/bin/env python3
"""Standalone SDK RS485 hand process, sharing the RH56FTP node policy."""
import argparse
import os
from pathlib import Path
import sys
import tempfile
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'rh56ftp_hand'))
from rh56ftp_node import Logger, Rh56FtpNode, main as hand_main
from transport import HandLink, SdkConnection
from scheduler import run_scheduled

class SdkHandNode(Rh56FtpNode):
    def make_state(self, now=None):
        state = super().make_state(now)
        for hand in state['hands'].values():
            hand['sample_time_basis'] = 'host_rs485_read'
        return state

def prepare_sdk_directory(parent=None):
    # xCoreSDK writes relative logs/ and _rokae_log_/ during connect, and
    # aborts on permission errors. Never share the sudo arm process's cwd.
    if parent is not None:
        parent = Path(parent).resolve()
        parent.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix='xcore-hand-', dir=parent))
    os.chdir(directory)
    Logger.info(f'xCoreSDK_hand SDK 工作目录: {directory}')
    return directory

def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--system-config', type=Path)
    parser.add_argument('--sdk-library', type=Path)
    parser.add_argument('--sdk-work-dir', type=Path)
    parser.add_argument('--fake-hand', action='store_true')
    known, rest = parser.parse_known_args(argv)
    if '--help' in rest or '-h' in rest:
        print('xCoreSDK_hand [--system-config system.yaml] [--config rh56ftp_hand.yaml] '
              '[--fake-hand] [RH56FTP options]\n'
              'Independent SDK RS485 process; no arm power/mode/stop/disconnect calls.')
        return hand_main(['--help'], simulated=True)
    root = {}
    section = {}
    if known.system_config:
        known.system_config = known.system_config.resolve()
        root = yaml.safe_load(known.system_config.read_text())
        section = root.get('xcore_sdk_hand', {})
        if not section.get('enabled', True):
            Logger.info('xCoreSDK_hand disabled by configuration')
            return 0
        def default(option, value):
            if not any(a == option or a.startswith(option + '=') for a in rest):
                rest.extend([option, str(value)])
        default('--config', known.system_config.parent / section.get('config', 'rh56ftp_hand.yaml'))
        default('--endpoint', root['bus']['subscribe'])
        default('--state-endpoint', root['bus']['publish'])
        default('--publisher-id', root.get('core_hand', {}).get('publisher_id', 'rh56ftp_hand'))
    fake = known.fake_hand or section.get('fake_hand', False)
    if fake:
        # Exactly the same simulator and policy as the old standalone fake node.
        return hand_main(rest, simulated=True, node_runner=run_scheduled)
    def transport(args, settings):
        if not known.system_config:
            raise ValueError('Real SDK hands require --system-config (robot.yaml connection addresses)')
        robot_file = known.system_config.parent / root['robot']
        robot = yaml.safe_load(robot_file.read_text())['rokae']
        timeout = settings.get('rs485_timeout_ms', 50)
        if type(timeout) is not int or not 50 <= timeout <= 10000:
            raise ValueError('rs485_timeout_ms must be 50..10000')
        if not known.sdk_library:
            raise ValueError('SDK library path missing; use the built xCoreSDK_hand launcher')
        library = known.sdk_library.resolve()
        if args.config:
            args.config = args.config.resolve()
        links = {}
        for side in ('left', 'right'):
            links[side] = (HandLink(SdkConnection(library, robot[side + '_ip'], timeout),
                                   settings.get(side + '_id', 1))
                           if settings.get(side + '_enabled', True) else None)
        # Resolve every input before changing cwd; SDK initialization happens
        # later in node.connect(). Fake hands never create this directory.
        prepare_sdk_directory(known.sdk_work_dir)
        # One SDK connection/serialized port per hand, shared by read/write threads.
        return links, links, SdkHandNode
    return hand_main(rest, transport_factory=transport, node_runner=run_scheduled)

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        Logger.error(f'xCoreSDK_hand: {exc}', file=sys.stderr)
        raise SystemExit(1)
