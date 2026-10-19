#!/usr/bin/python3
"""AVIATOR desktop launcher. Non-GUI diagnostics work without GTK."""
import argparse
import json
import os
from pathlib import Path
import sys
sys.dont_write_bytecode = True
from paths import resolve_layout
from options import Options


def main():
    parser = argparse.ArgumentParser(description='AVIATOR GNOME 启动器；打开界面不会自动启动节点。')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--show', action='store_true', help='显示配置窗口')
    mode.add_argument('--tray', action='store_true', help='优先驻留托盘；不可用时显示窗口')
    mode.add_argument('--check', action='store_true', help='输出路径与图形依赖诊断，不启动节点')
    mode.add_argument('--register-desktop', action='store_true', help='注册当前用户应用菜单入口')
    mode.add_argument('--unregister-desktop', action='store_true', help='移除本工具管理的用户菜单入口')
    args = parser.parse_args()
    try:
        layout = resolve_layout()
        if args.check:
            dependencies = {}
            try:
                import gi
                for name, version in [('Gtk', '3.0'), ('AppIndicator3', '0.1'), ('AyatanaAppIndicator3', '0.1')]:
                    try:
                        gi.require_version(name, version)
                        dependencies[name] = True
                    except ValueError:
                        dependencies[name] = False
            except ImportError:
                dependencies['PyGObject'] = False
            print(json.dumps({'kind': layout.kind, 'root': str(layout.root), 'script': str(layout.script),
                'bin': str(Options().binary_directory(layout)), 'dependencies': dependencies,
                'desktop_session': bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')),
                'note': 'bin 按默认整机仿真选项自动选择；依赖检查不代表托盘宿主已启用，也不检查设备就绪。'}, ensure_ascii=False, indent=2))
            return 0
        if os.geteuid() == 0:
            raise ValueError('请以普通用户运行启动器，不要 sudo 启动整个界面。')
        if args.register_desktop or args.unregister_desktop:
            import desktop
            print(desktop.register(not args.unregister_desktop))
            return 0
        from app import Application
        from gi.repository import Gtk
        if not Gtk.init_check()[0]:
            raise ValueError('无法连接图形会话，请在 GNOME 桌面中运行。')
        return Application(layout, not args.show).run([str(Path(__file__))])
    except (ValueError, OSError, ImportError) as exc:
        print(f'AVIATOR 启动器：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
