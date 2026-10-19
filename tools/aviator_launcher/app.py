"""Asynchronous desktop controller. Opening the app never starts AVIATOR."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import shlex
import subprocess
import time
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gio, GLib
import desktop
from options import ENV_FIELDS
from paths import resolve_layout, runtime_dir
from protocol import request
from settings import Settings
from window import Window
from tray import Tray


class Application(Gtk.Application):
    def __init__(self, layout, tray_only=False):
        super().__init__(application_id=desktop.APP_ID, flags=Gio.ApplicationFlags.FLAGS_NONE)
        self.layout = layout
        self.tray_only = tray_only
        self.settings = Settings()
        self.window = self.tray = None
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.state = {'status': 'IDLE'}
        self.busy = self.poll_pending = self.exiting = False
        self.runner_process = None
        self.last_output = None
        self.was_connected = False
        self.generation = 0
        self._autostart_sync = False
        self.connect('activate', self.activate_window)

    def activate_window(self, *_):
        if self.window is not None:
            self.show()
            return
        self.hold()
        self.window = Window(self)
        self.window.show_all()
        if self.tray_only:
            self.window.hide()
        self.tray = Tray(self)
        if self.settings.warning:
            self.window.message(self.settings.warning)
        GLib.timeout_add(1000, self.poll)
        self.poll()

    def show(self):
        if self.window:
            self.window.show_all()
            self.window.notice.set_visible(bool(self.window.notice.get_text()))
            self.window.present()

    def show_logs(self):
        self.show()
        self.window.stack.set_visible_child_name('logs')

    def work(self, operation, completed=None):
        future = self.executor.submit(operation)

        def dispatch(done):
            def finish():
                try:
                    result = done.result()
                    if completed:
                        completed(result)
                except Exception as exc:
                    self.window.message(str(exc))
                    self.exiting = False
                finally:
                    self.busy = False
                    self.window.set_status(self.state)
                return False
            GLib.idle_add(finish)
        future.add_done_callback(dispatch)

    def ensure_runner(self):
        try:
            return request('status')
        except (FileNotFoundError, ConnectionRefusedError):
            pass
        directory = runtime_dir()
        with (directory / 'runner.log').open('w') as output:
            self.runner_process = subprocess.Popen(['/usr/bin/python3', '-B', str(Path(__file__).with_name('runner.py'))],
                stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            try:
                return request('status')
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(.05)
        raise ValueError('运行管理器未启动，请检查 ' + str(directory / 'runner.log'))

    def update(self, state):
        self.was_connected = True
        self.state = state
        # Reopening a different checkout must show the actual active run's source.
        if state.get('root') and state['status'] in ('STARTING', 'RUNNING', 'STOPPING', 'NEEDS_ATTENTION'):
            if state['root'] != str(self.layout.root):
                self.layout = resolve_layout(state['root'])
                from options import Options
                self.window.load(Options.from_dict(state.get('options', {})))
        self.window.set_status(state, self.busy)
        self.tray.update(state)
        if state.get('warning'):
            self.window.message(state['warning'])
        elif state.get('external_lock'):
            self.window.message('其他启动脚本或残留节点持有启动锁，请先处理该运行。')
        if state.get('output') and state['output'] != self.last_output:
            self.last_output = state['output']
            self.window.output(state['output'])
        if self.exiting and state['status'] not in ('STARTING', 'RUNNING', 'STOPPING'):
            if state['status'] == 'NEEDS_ATTENTION':
                self.exiting = False
                self.show()
            else:
                self.quit_app()

    def poll(self):
        if self.runner_process:
            self.runner_process.poll()  # reap a manager that completed a requested exit
        if self.poll_pending or self.busy or self.exiting:
            return True
        self.poll_pending = True
        generation = self.generation
        future = self.executor.submit(lambda: request('status'))
        def done(future):
            def finish():
                self.poll_pending = False
                try:
                    state = future.result()
                    if generation == self.generation:
                        self.update(state)
                except (FileNotFoundError, ConnectionRefusedError):
                    if self.was_connected and self.state['status'] in ('STARTING', 'RUNNING', 'STOPPING'):
                        self.state['status'] = 'NEEDS_ATTENTION'
                        self.window.set_status(self.state)
                        self.window.message('运行管理器失联，状态未知；未接管或强制结束节点。')
                except Exception as exc:
                    self.window.message(f'无法读取运行状态：{exc}')
                return False
            GLib.idle_add(finish)
        future.add_done_callback(done)
        return True

    def save_options(self):
        try:
            self.settings.save(self.layout, self.window.options())
            self.window.message('启动选项已保存。')
        except (ValueError, OSError) as exc:
            self.window.message(str(exc))

    def preview(self):
        try:
            options, layout = self.window.options(), self.layout
        except ValueError as exc:
            self.window.message(str(exc))
            return
        self.busy = True
        self.window.set_status(self.state, True)
        def run():
            env = options.environment(layout)
            command = options.command(layout, True)
            result = subprocess.run(command, cwd=layout.root, env=env, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=15)
            effective = '\n'.join(f'{key}={env[key]}' for key in (*ENV_FIELDS.values(), 'START_DELAY'))
            return f'{shlex.join(command)}\n\n{effective}\n\n{result.stdout}\n预览退出码：{result.returncode}'
        def completed(output):
            self.window.output(output)
            self.show_logs()
        self.work(run, completed)

    def start(self):
        if self.busy:
            return
        try:
            options = self.window.options()
            self.settings.save(self.layout, options)
            root = str(self.layout.root)
        except (ValueError, OSError) as exc:
            self.window.message(str(exc))
            return
        self.busy = True
        self.generation += 1
        self.window.message('正在检查启动条件…')
        self.window.set_status(self.state, True)
        def launch():
            self.ensure_runner()
            return request('start', root=root, options=options.to_dict())
        def completed(state):
            self.window.message('已提交启动，节点将按脚本顺序启动。')
            self.update(state)
        self.work(launch, completed)

    def stop(self):
        self.generation += 1
        self.work(lambda: request('stop'), self.update)

    def recheck(self):
        self.generation += 1
        def check():
            self.ensure_runner()
            return request('recheck')
        self.work(check, self.update)

    def exit_requested(self):
        if self.busy:
            self.window.message('当前操作尚未完成，请稍后退出。')
            return
        if self.state['status'] == 'NEEDS_ATTENTION':
            self.window.message('运行状态待处理，请检查日志。关闭窗口可保留此状态。')
            self.show()
            return
        if self.state['status'] in ('STARTING', 'RUNNING', 'STOPPING'):
            self.exiting = True
            def stop_and_wait():
                state = request('stop')
                while state['status'] in ('STARTING', 'RUNNING', 'STOPPING'):
                    time.sleep(.5)
                    state = request('status')
                return state
            self.window.message('正在等待脚本按顺序停止节点，完成后退出。')
            self.window.set_status({'status': 'STOPPING'})
            self.work(stop_and_wait, self.update)
        else:
            self.quit_app()

    def quit_app(self):
        if self.tray:
            self.tray.close()
        # Idle runner exits itself after clients disconnect. Never kill it here.
        self.executor.shutdown(wait=False)
        self.quit()

    def close_window(self, *_):
        if self.tray and self.tray.available:
            self.window.hide()
        elif self.state['status'] in ('STARTING', 'RUNNING', 'STOPPING'):
            dialog = Gtk.MessageDialog(transient_for=self.window, modal=True,
                message_type=Gtk.MessageType.QUESTION, text='当前没有托盘入口，是否停止并退出？')
            dialog.add_buttons('保持窗口', Gtk.ResponseType.CANCEL, '停止并退出', Gtk.ResponseType.OK)
            response = dialog.run()
            dialog.destroy()
            if response == Gtk.ResponseType.OK:
                self.exit_requested()
        else:
            self.exit_requested()
        return True

    def choose_location(self):
        dialog = Gtk.FileChooserDialog(title='选择源码目录或安装目录', transient_for=self.window,
            action=Gtk.FileChooserAction.SELECT_FOLDER)
        dialog.add_buttons('取消', Gtk.ResponseType.CANCEL, '选择', Gtk.ResponseType.OK)
        if dialog.run() == Gtk.ResponseType.OK:
            try:
                self.layout = resolve_layout(dialog.get_filename())
                self.window.load(self.settings.get(self.layout))
                self.window.message('已切换程序位置；启动前请检查二进制目录。')
            except ValueError as exc:
                self.window.message(str(exc))
        dialog.destroy()

    def open_monitor(self):
        Gio.AppInfo.launch_default_for_uri('http://127.0.0.1:8081', None)

    def open_logs(self):
        value = self.state.get('node_log_dir') or self.state.get('log_path')
        if value:
            path = Path(value)
            Gio.AppInfo.launch_default_for_uri((path if path.is_dir() else path.parent).as_uri(), None)
        else:
            self.show_logs()

    def register_desktop(self):
        try:
            path = desktop.register()
            self.window.message('已注册应用菜单入口：' + str(path))
        except (ValueError, OSError) as exc:
            self.window.message(str(exc))

    def set_autostart(self, enabled):
        if self._autostart_sync:
            return
        try:
            desktop.register(enabled, True)
            self.window.message('登录后将显示启动器，不自动运行节点。' if enabled else '已关闭登录自启动。')
        except (ValueError, OSError) as exc:
            self.window.message(str(exc))
        finally:
            self._autostart_sync = True
            try:
                actual = desktop.registered(True)
                self.window.autostart.set_active(actual)
                if self.tray and hasattr(self.tray, 'autostart'):
                    self.tray.autostart.set_active(actual)
            finally:
                self._autostart_sync = False
