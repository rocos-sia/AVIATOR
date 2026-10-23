"""Pure option validation and argument construction, shared by preview and runner."""
from dataclasses import asdict, dataclass, fields
from pathlib import Path
import os
import re
import shutil
from paths import user_dir

ENV_FIELDS = {'bin_dir': 'AVIATOR_BIN', 'conda_root': 'CONDA_ROOT',
              'camera_python': 'CAMERA_PYTHON', 'monitor_bin': 'MONITOR_BIN',
              'tts_config': 'TTS_CONFIG', 'recording_dir': 'AVIATOR_RECORDING_DIR'}


@dataclass
class Options:
    mode: str = 'simulation'
    rs422: bool = False
    headless: bool = False
    logger: bool = False
    tts: bool = False
    bin_dir: str = ''
    conda_root: str = ''
    camera_python: str = ''
    monitor_bin: str = ''
    tts_config: str = ''
    recording_dir: str = ''
    start_delay: str = '2'

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict) or set(data) - {f.name for f in fields(cls)}:
            raise ValueError('启动选项包含未知字段')
        result = cls(**data)
        for field in fields(cls):
            if type(getattr(result, field.name)) is not type(field.default):
                raise ValueError(f'选项类型错误：{field.name}')
        if result.mode not in ('hardware', 'fake-hand', 'simulation'):
            raise ValueError('未知设备模式')
        if result.headless and result.mode != 'simulation':
            raise ValueError('隐藏仿真窗口仅适用于整机仿真')
        if not re.fullmatch(r'[0-9]+(?:\.[0-9]+)?', result.start_delay) or len(result.start_delay) > 12:
            raise ValueError('启动间隔须为非负十进制秒数')
        for key in ENV_FIELDS:
            value = getattr(result, key)
            if value and ('\x00' in value or '\n' in value or not Path(value).expanduser().is_absolute()):
                raise ValueError(f'{key} 必须为绝对路径')
        return result

    def to_dict(self):
        return asdict(self)

    def command(self, layout, preview=False):
        args = ['/bin/bash', str(layout.script)]
        if self.mode != 'hardware':
            args.append('--simulation' if self.mode == 'simulation' else '--fake-hand')
        args.extend('--' + name for name in ('rs422', 'headless', 'logger', 'tts') if getattr(self, name))
        if preview:
            args.append('--dry-run')
        return args

    def environment(self, layout):
        env = dict(os.environ)
        for name in (*ENV_FIELDS.values(), 'START_DELAY'):
            env.pop(name, None)
        defaults = {
            'bin_dir': self.binary_directory(layout),
            'conda_root': Path.home() / 'miniconda3',
            'recording_dir': user_dir('XDG_DATA_HOME', '.local/share') / 'aviator/recordings',
            'tts_config': layout.root / 'config/tts.yaml',
        }
        for key, value in defaults.items():
            env[ENV_FIELDS[key]] = str(Path(getattr(self, key) or value).expanduser())
        env['CAMERA_PYTHON'] = str(Path(self.camera_python or (layout.prefix / 'cache/python/bin/python' if (layout.prefix / 'cache/python/bin/python').is_file() else Path(env['CONDA_ROOT']) / 'envs/apriltag_realsense/bin/python')).expanduser())
        env['MONITOR_BIN'] = str(Path(self.monitor_bin or Path(env['AVIATOR_BIN']) / 'aviator_monitor').expanduser())
        env['START_DELAY'] = self.start_delay
        env['PYTHONDONTWRITEBYTECODE'] = '1'
        return env

    def required_binaries(self):
        names = ['aviator_bus', 'flight_gateway', 'aviator_core_managed']
        names += ['simulation'] if self.mode == 'simulation' else ['manipulator', 'xCoreSDK_hand']
        if self.logger:
            names.append('aviator_logger')
        if self.tts:
            names.append('aviator_tts')
        if not self.monitor_bin:
            names.append('aviator_monitor')
        return names

    def binary_directory(self, layout):
        if self.bin_dir:
            return Path(self.bin_dir).expanduser()
        return layout.select_bin(self.required_binaries())

    def preflight(self, layout):
        env = self.environment(layout)
        executables = [Path(env['AVIATOR_BIN']) / name for name in self.required_binaries()]
        if self.monitor_bin:
            executables.append(Path(env['MONITOR_BIN']))
        if self.mode != 'simulation':
            executables.append(Path(env['CAMERA_PYTHON']))
        for file in executables:
            if not file.is_file() or not os.access(file, os.X_OK):
                raise ValueError(f'可执行文件不存在或不可执行：{file}')
        commands = ['setsid', 'flock', 'ps', 'awk', 'google-chrome-stable', 'curl']
        if self.mode != 'simulation':
            commands.append('sudo')
        if self.tts:
            commands.append('aplay')
        for command in commands:
            if not shutil.which(command, path=env.get('PATH')):
                raise ValueError(f'缺少命令：{command}')
        return env
