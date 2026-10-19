"""GTK3 presentation; no process lifecycle logic lives in widgets."""
from pathlib import Path
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, Pango
from options import Options

STATUS = {'IDLE': '未启动', 'VALIDATING': '检查中', 'STARTING': '启动中',
          'RUNNING': '脚本运行中', 'STOPPING': '停止中', 'FAILED': '启动未完成',
          'NEEDS_ATTENTION': '需要处理'}


def styled(widget, name):
    widget.get_style_context().add_class(name)
    return widget


def label(text, style=None):
    result = Gtk.Label(label=text, xalign=0)
    result.set_line_wrap(True)
    if style:
        styled(result, style)
    return result


class Window(Gtk.ApplicationWindow):
    def __init__(self, app):
        super().__init__(application=app, title='AVIATOR 启动器')
        self.controller = app
        self.set_default_size(860, 800)
        self.set_size_request(720, 580)
        styled(self, 'aviator-window')
        self.set_icon_from_file(str(Path(__file__).parent / 'assets/aviator-launcher.svg'))
        css = Gtk.CssProvider()
        css.load_from_path(str(Path(__file__).parent / 'assets/style.css'))
        Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
        header = Gtk.HeaderBar(title='AVIATOR', subtitle='桌面启动器', show_close_button=True)
        self.set_titlebar(header)
        self.stack = Gtk.Stack(transition_type=Gtk.StackTransitionType.CROSSFADE)
        switcher = Gtk.StackSwitcher(stack=self.stack)
        header.set_custom_title(switcher)
        menu = Gtk.MenuButton()
        menu.add(Gtk.Image.new_from_icon_name('open-menu-symbolic', Gtk.IconSize.BUTTON))
        header.pack_end(menu)
        actions = Gtk.Menu()
        for title, callback in [('注册到应用菜单', app.register_desktop), ('打开 Monitor', app.open_monitor), ('停止并退出 / 退出', app.exit_requested)]:
            item = Gtk.MenuItem(label=title)
            item.connect('activate', lambda _, cb=callback: cb())
            actions.append(item)
        self.autostart = Gtk.CheckMenuItem(label='登录后显示启动器')
        import desktop
        self.autostart.set_active(desktop.registered(True))
        self.autostart.connect('toggled', lambda item: app.set_autostart(item.get_active()))
        actions.append(self.autostart)
        actions.show_all()
        menu.set_popup(actions)
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(outer)
        outer.pack_start(self.stack, True, True, 0)
        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12, margin=20)
        scroll.add(content)
        self.stack.add_titled(scroll, 'configure', '启动配置')
        hero = styled(Gtk.Box(spacing=20), 'hero')
        icon = Gtk.Image.new_from_file(str(Path(__file__).parent / 'assets/aviator-launcher.svg'))
        icon.set_pixel_size(64)
        # Render the vector at a bounded size rather than its intrinsic 128px.
        from gi.repository import GdkPixbuf
        icon.set_from_pixbuf(GdkPixbuf.Pixbuf.new_from_file_at_scale(str(Path(__file__).parent / 'assets/aviator-launcher.svg'), 64, 64, True))
        hero.pack_start(icon, False, False, 0)
        titles = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        titles.pack_start(label('FLIGHT OPERATIONS', 'eyebrow'), False, False, 0)
        titles.pack_start(label('AVIATOR', 'hero-title'), False, False, 0)
        titles.pack_start(label('选择运行环境，准备本次任务。', 'hero-subtitle'), False, False, 0)
        hero.pack_start(titles, True, True, 0)
        self.badge = styled(Gtk.Label(label='●  未启动', valign=Gtk.Align.CENTER), 'pill')
        hero.pack_end(self.badge, False, False, 0)
        content.pack_start(hero, False, False, 0)
        self.notice = label('', 'notice')
        self.notice.set_no_show_all(True)
        content.pack_start(self.notice, False, False, 0)
        self.editor = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        content.pack_start(self.editor, False, False, 0)
        source = styled(Gtk.Box(orientation=Gtk.Orientation.VERTICAL), 'card')
        self.editor.pack_start(source, False, False, 0)
        row = Gtk.Box(spacing=12)
        row.pack_start(label('程序位置', 'section-title'), False, False, 0)
        self.location = label('', 'muted')
        self.location.set_ellipsize(Pango.EllipsizeMode.MIDDLE)
        self.location.set_max_width_chars(65)
        row.pack_start(self.location, True, True, 0)
        choose = Gtk.Button(label='选择位置…')
        choose.connect('clicked', lambda _: app.choose_location())
        row.pack_end(choose, False, False, 0)
        source.pack_start(row, False, False, 0)
        mode_card = self.card('01  运行环境', self.editor)
        modes = Gtk.Box(spacing=10, homogeneous=True)
        self.modes = {}
        for key, title, sub in [('simulation', '整机仿真', '仿真设备 · 无真实机械臂'),
                                ('hardware', '真机运行', '双手状态沿用系统配置'),
                                ('fake-hand', '真机 + 模拟手', '真实机械臂 · 模拟双手')]:
            button = styled(Gtk.ToggleButton(), 'mode')
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
            box.pack_start(label(title, 'mode-title'), False, False, 0)
            box.pack_start(label(sub, 'muted'), False, False, 0)
            button.add(box)
            button.connect('clicked', lambda _, value=key: self.set_mode(value))
            modes.pack_start(button, True, True, 0)
            self.modes[key] = button
        mode_card.pack_start(modes, False, False, 0)
        row = Gtk.Box(spacing=12)
        row.pack_start(label('输入来源'), False, False, 0)
        self.source = Gtk.ComboBoxText()
        self.source.append('local', '本机摇杆 / 键盘')
        self.source.append('rs422', 'RS422 双路串口')
        self.source.connect('changed', lambda _: self.changed())
        row.pack_end(self.source, False, False, 0)
        mode_card.pack_start(row, False, False, 0)
        features = self.card('02  附加功能', self.editor)
        self.flags = {}
        for key, title, hint in [('logger', 'MCAP 录制', '保存业务消息与相机数据，策略沿用 recording.yaml'),
                                 ('tts', '离线语音提醒', '播放运行状态与告警，需要已准备的音频包'),
                                 ('headless', '隐藏仿真窗口', '保留相机渲染与传输；Monitor 浏览器仍会打开')]:
            row = Gtk.Box(spacing=16)
            texts = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
            texts.pack_start(label(title), False, False, 0)
            texts.pack_start(label(hint, 'muted'), False, False, 0)
            row.pack_start(texts, True, True, 0)
            toggle = Gtk.Switch(valign=Gtk.Align.CENTER)
            toggle.connect('notify::active', lambda *_: self.changed())
            row.pack_end(toggle, False, False, 0)
            features.pack_start(row, False, False, 0)
            self.flags[key] = toggle
        advanced = Gtk.Expander(label='高级选项 · 路径与启动间隔')
        self.editor.pack_start(advanced, False, False, 0)
        grid = Gtk.Grid(row_spacing=10, column_spacing=12, margin_top=14)
        advanced.add(grid)
        self.entries = {}
        for i, (key, title) in enumerate([('bin_dir', '二进制目录'), ('camera_python', '相机 Python'),
            ('conda_root', 'Conda 根目录'), ('monitor_bin', 'Monitor 程序'), ('tts_config', '语音配置'),
            ('recording_dir', '录制输出目录'), ('start_delay', '启动间隔 / 秒')]):
            grid.attach(label(title, 'muted'), 0, i, 1, 1)
            entry = Gtk.Entry(hexpand=True)
            entry.set_placeholder_text('自动：使用同套工程或安装包的默认值')
            if key == 'bin_dir':
                entry.set_placeholder_text('自动：Release 优先，其余按编译时间选择')
                entry.set_tooltip_text('源码搜索 build/release/bin、build/debug/bin、build/bin；安装包使用同包 bin。留空启用自动选择。')
            self.entries[key] = entry
            grid.attach(entry, 1, i, 1, 1)
            if key != 'start_delay':
                browse = Gtk.Button(label='选择…')
                browse.connect('clicked', lambda _, k=key: self.browse(k))
                grid.attach(browse, 2, i, 1, 1)
        content.pack_start(label('启动后脚本会打开 Monitor；窗口关闭后可从顶栏重新打开。', 'muted'), False, False, 0)
        logs = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12, margin=24)
        self.stack.add_titled(logs, 'logs', '输出与日志')
        bar = Gtk.Box(spacing=12)
        bar.pack_start(label('启动命令与本次脚本输出', 'section-title'), True, True, 0)
        recheck = Gtk.Button(label='重新检查运行状态')
        recheck.connect('clicked', lambda _: app.recheck())
        bar.pack_end(recheck, False, False, 0)
        folder = Gtk.Button(label='打开日志目录')
        folder.connect('clicked', lambda _: app.open_logs())
        bar.pack_end(folder, False, False, 0)
        logs.pack_start(bar, False, False, 0)
        self.console = styled(Gtk.TextView(editable=False, cursor_visible=False, monospace=True,
            wrap_mode=Gtk.WrapMode.WORD_CHAR, left_margin=16, right_margin=16, top_margin=16, bottom_margin=16), 'console')
        logscroll = Gtk.ScrolledWindow()
        logscroll.add(self.console)
        logs.pack_start(logscroll, True, True, 0)
        self.log_location = label('尚无本次运行日志', 'muted')
        self.log_location.set_selectable(True)
        logs.pack_start(self.log_location, False, False, 0)
        footer = styled(Gtk.Box(spacing=12), 'footer')
        outer.pack_end(footer, False, False, 0)
        self.summary = label('整机仿真', 'muted')
        footer.pack_start(self.summary, True, True, 0)
        self.preview = Gtk.Button(label='预览命令')
        self.preview.connect('clicked', lambda _: app.preview())
        footer.pack_start(self.preview, False, False, 0)
        self.save = Gtk.Button(label='保存选项')
        self.save.connect('clicked', lambda _: app.save_options())
        footer.pack_start(self.save, False, False, 0)
        self.stop = styled(Gtk.Button(label='停止'), 'stop')
        self.stop.connect('clicked', lambda _: app.stop())
        footer.pack_start(self.stop, False, False, 0)
        self.start = styled(Gtk.Button(label='启动 AVIATOR'), 'primary')
        self.start.connect('clicked', lambda _: app.start())
        footer.pack_end(self.start, False, False, 0)
        self.connect('delete-event', app.close_window)
        self.loading = False
        self.mode = 'simulation'
        self.load(app.settings.get(app.layout))
        self.set_status({'status': 'IDLE'})

    def card(self, title, parent):
        box = styled(Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10), 'card')
        box.pack_start(label(title, 'section-title'), False, False, 0)
        parent.pack_start(box, False, False, 0)
        return box

    def browse(self, key):
        directory = key in ('bin_dir', 'conda_root', 'recording_dir')
        dialog = Gtk.FileChooserDialog(title='选择路径', transient_for=self,
            action=Gtk.FileChooserAction.SELECT_FOLDER if directory else Gtk.FileChooserAction.OPEN)
        dialog.add_buttons('取消', Gtk.ResponseType.CANCEL, '选择', Gtk.ResponseType.OK)
        if dialog.run() == Gtk.ResponseType.OK:
            self.entries[key].set_text(dialog.get_filename())
        dialog.destroy()

    def set_mode(self, value):
        if getattr(self, '_changing_mode', False):
            return
        self._changing_mode = True
        self.mode = value
        for key, button in self.modes.items():
            button.set_active(key == value)
        self.flags['headless'].set_sensitive(value == 'simulation')
        if value != 'simulation':
            self.flags['headless'].set_active(False)
        self._changing_mode = False
        self.changed()

    def load(self, options):
        self.loading = True
        for key, entry in self.entries.items():
            entry.set_text(getattr(options, key))
        self.source.set_active_id('rs422' if options.rs422 else 'local')
        for key, toggle in self.flags.items():
            toggle.set_active(getattr(options, key))
        self.set_mode(options.mode)
        self.loading = False
        self.location.set_text(('安装包' if self.controller.layout.kind == 'installed' else '源码目录') + '  ·  ' + str(self.controller.layout.root))
        self.location.set_tooltip_text(str(self.controller.layout.script))
        self.changed()

    def options(self):
        return Options.from_dict(dict(mode=self.mode, rs422=self.source.get_active_id() == 'rs422',
            **{k: v.get_active() for k, v in self.flags.items()},
            **{k: v.get_text().strip() for k, v in self.entries.items()}))

    def changed(self):
        if not hasattr(self, 'summary') or getattr(self, 'loading', False):
            return
        mode = {'simulation': '整机仿真', 'hardware': '真机运行', 'fake-hand': '真机 + 模拟手'}[self.mode]
        self.summary.set_text(mode + ' · ' + ('RS422' if self.source.get_active_id() == 'rs422' else '本机输入'))

    def message(self, text):
        self.notice.set_text(text)
        self.notice.set_visible(bool(text))

    def output(self, text):
        self.console.get_buffer().set_text(text[-48000:])

    def set_status(self, state, busy=False):
        status = state.get('status', 'IDLE')
        self.badge.set_text('●  ' + STATUS.get(status, '状态未确认'))
        context = self.badge.get_style_context()
        for cls in ('running', 'attention'):
            context.remove_class(cls)
        if status == 'RUNNING':
            context.add_class('running')
        elif status not in ('IDLE',):
            context.add_class('attention')
        active = status in ('VALIDATING', 'STARTING', 'RUNNING', 'STOPPING')
        blocked = active or busy or status == 'NEEDS_ATTENTION' or state.get('external_lock', False)
        self.editor.set_sensitive(not blocked)
        self.start.set_sensitive(not blocked)
        self.preview.set_sensitive(not blocked)
        self.save.set_sensitive(not blocked)
        self.stop.set_sensitive(active and status != 'STOPPING')
        self.start.set_label('停止中…' if status == 'STOPPING' else '启动 AVIATOR')
        self.log_location.set_text(state.get('node_log_dir') or state.get('log_path') or '尚无本次运行日志')
