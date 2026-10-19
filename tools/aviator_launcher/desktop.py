"""User registration is explicit and separate from CMake package installation."""
from pathlib import Path
from paths import user_dir

APP_ID = 'org.aviator.Launcher'
MARKER = 'X-AVIATOR-Launcher=true'


def exec_quote(value):
    # Desktop Entry has its own escape rules; this is not shell quoting.
    value = str(value)
    if any(c in value for c in '\n\r\x00'):
        raise ValueError('桌面入口路径包含非法字符')
    value = value.replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$')
    value = value.replace('\\', '\\\\').replace('%', '%%')
    return '"' + value + '"'


def entry_path(autostart=False):
    base = user_dir('XDG_CONFIG_HOME', '.config') / 'autostart' if autostart else user_dir('XDG_DATA_HOME', '.local/share') / 'applications'
    return base / (APP_ID + '.desktop')


def registered(autostart=False):
    path = entry_path(autostart)
    return path.is_file() and MARKER in path.read_text()


def register(enable=True, autostart=False):
    path = entry_path(autostart)
    if path.exists() and MARKER not in path.read_text():
        raise ValueError(f'不覆盖其他程序创建的桌面文件：{path}')
    if not enable:
        path.unlink(missing_ok=True)
        return path
    here = Path(__file__).resolve().parent
    args = ['/usr/bin/python3', '-B', str(here / 'main.py'), '--tray' if autostart else '--show']
    icon = str(here / 'assets/aviator-launcher.svg').replace('\\', '\\\\')
    path.parent.mkdir(parents=True, exist_ok=True)
    text = '\n'.join(['[Desktop Entry]', 'Type=Application', 'Name=AVIATOR 启动器',
        'Comment=配置与管理 AVIATOR 启动任务', 'Exec=' + ' '.join(map(exec_quote, args)),
        'Icon=' + icon, 'Terminal=false', 'StartupNotify=true', 'Categories=Utility;',
        'StartupWMClass=' + APP_ID, MARKER, ''])
    # Atomic replacement; user entries must never modify the package desktop file.
    import os
    import tempfile
    fd, temp = tempfile.mkstemp(dir=path.parent, prefix='.aviator-')
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(text)
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)
    return path
