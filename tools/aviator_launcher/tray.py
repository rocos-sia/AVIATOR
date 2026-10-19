"""Indicator backend and GNOME host availability tracking."""
from pathlib import Path
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gio, GLib
from window import STATUS


class Tray:
    def __init__(self, app):
        self.app, self.available, self.indicator = app, False, None
        self.watch = 0
        backend = None
        for name in ('AppIndicator3', 'AyatanaAppIndicator3'):
            try:
                gi.require_version(name, '0.1')
                import importlib
                backend = importlib.import_module('gi.repository.' + name)
                break
            except (ValueError, ImportError):
                continue
        if backend is None:
            GLib.idle_add(self.host_changed, False)
            return
        icon = str(Path(__file__).parent / 'assets/aviator-launcher.svg')
        self.indicator = backend.Indicator.new('aviator-launcher', icon, backend.IndicatorCategory.APPLICATION_STATUS)
        self.indicator.set_status(backend.IndicatorStatus.ACTIVE)
        self.menu = Gtk.Menu()
        self.title = Gtk.MenuItem(label='AVIATOR · 未启动')
        self.title.set_sensitive(False)
        self.menu.append(self.title)
        self.menu.append(Gtk.SeparatorMenuItem())
        for title, callback in [('配置与启动…', app.show), ('显示运行日志…', app.show_logs),
                                ('打开 Monitor', app.open_monitor), ('打开本次日志目录', app.open_logs)]:
            item = Gtk.MenuItem(label=title)
            item.connect('activate', lambda _, cb=callback: cb())
            self.menu.append(item)
        self.stop = Gtk.MenuItem(label='停止本次运行')
        self.stop.connect('activate', lambda _: app.stop())
        self.menu.append(self.stop)
        self.menu.append(Gtk.SeparatorMenuItem())
        import desktop
        self.autostart = Gtk.CheckMenuItem(label='登录后显示启动器')
        self.autostart.set_active(desktop.registered(True))
        self.autostart.connect('toggled', lambda item: app.set_autostart(item.get_active()))
        self.menu.append(self.autostart)
        self.exit = Gtk.MenuItem(label='退出启动器')
        self.exit.connect('activate', lambda _: app.exit_requested())
        self.menu.append(self.exit)
        self.menu.show_all()
        self.indicator.set_menu(self.menu)
        self.watch = Gio.bus_watch_name(Gio.BusType.SESSION, 'org.kde.StatusNotifierWatcher',
            Gio.BusNameWatcherFlags.NONE, self.appeared, lambda *_: self.host_changed(False))

    def appeared(self, connection, name, owner):
        def complete(conn, result):
            try:
                value = conn.call_finish(result).unpack()[0]
                if isinstance(value, GLib.Variant):
                    value = value.unpack()
                self.host_changed(bool(value))
            except GLib.Error:
                self.host_changed(False)
        connection.call(name, '/StatusNotifierWatcher', 'org.freedesktop.DBus.Properties',
            'Get', GLib.Variant('(ss)', ('org.kde.StatusNotifierWatcher', 'IsStatusNotifierHostRegistered')),
            GLib.VariantType.new('(v)'), Gio.DBusCallFlags.NONE, 1000, None, complete)

    def host_changed(self, available):
        self.available = available
        if not available:
            self.app.show()
            self.app.window.message('当前桌面未提供可用托盘区域，已切换为窗口模式。')
        return False

    def update(self, state):
        if not self.indicator:
            return
        status = state.get('status', 'IDLE')
        self.title.set_label('AVIATOR · ' + STATUS.get(status, '状态未确认'))
        active = status in ('STARTING', 'RUNNING', 'STOPPING')
        self.stop.set_sensitive(active and status != 'STOPPING')
        self.exit.set_label('停止并退出' if active else '退出启动器')

    def close(self):
        if self.watch:
            Gio.bus_unwatch_name(self.watch)
