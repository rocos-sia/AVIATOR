"""Isolated launcher regressions. No hardware, real sudo, or production bus."""
from pathlib import Path
import fcntl
import itertools
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1]
REPO = MODULE.parents[1]
sys.path.insert(0, str(MODULE))
from options import Options
from paths import resolve_layout, runtime_dir
from settings import Settings
from protocol import request, receive
from runner import Runner, process_identity
import desktop


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='aviator launcher 空格 ')
        self.base = Path(self.temp.name)
        self.runtime = self.base / 'runtime'
        self.runtime.mkdir(mode=0o700)
        self.env = patch.dict(os.environ, {
            'XDG_RUNTIME_DIR': str(self.runtime), 'XDG_CONFIG_HOME': str(self.base / 'config'),
            'XDG_STATE_HOME': str(self.base / 'state'), 'XDG_DATA_HOME': str(self.base / 'data'),
        })
        self.env.start()
        self.addCleanup(self.env.stop)
        self.addCleanup(self.temp.cleanup)
        self.root = self.base / '源码'
        (self.root / 'scripts').mkdir(parents=True)
        (self.root / 'config').mkdir()
        (self.root / 'CMakeLists.txt').write_text('project(AVIATOR VERSION 6.0.0)')
        (self.root / 'scripts/start_aviator.sh').write_text('#!/bin/bash\nexit 0\n')

    def test_modes_match_real_dry_run(self):
        layout = resolve_layout(REPO)
        for mode, rs422, logger, tts in itertools.product(('hardware', 'fake-hand', 'simulation'), (False, True), (False, True), (False, True)):
            with self.subTest(mode=mode, rs422=rs422, logger=logger, tts=tts):
                options = Options(mode=mode, rs422=rs422, logger=logger, tts=tts, headless=mode == 'simulation')
                command = options.command(layout, True)
                result = subprocess.run(command, env=options.environment(layout), cwd=self.base,
                    capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual('--fake-hand' in result.stdout, mode == 'fake-hand')
                self.assertEqual('启动 simulation:' in result.stdout, mode == 'simulation')
                self.assertEqual('启动 logger:' in result.stdout, logger)
                self.assertEqual('启动 tts:' in result.stdout, tts)
                self.assertEqual('flight-rs422.yaml' in result.stdout, rs422)
        self.assertFalse((self.base / 'data').exists(), 'dry-run created recording directory')

    def test_option_types_and_boundaries(self):
        for data in [{'mode': 'bad'}, {'mode': 'hardware', 'headless': True}, {'logger': 'yes'},
                     {'bin_dir': 'relative'}, {'start_delay': 'NaN'}, {'start_delay': '-1'},
                     {'start_delay': '1e2'}, {'command': 'bad'}]:
            with self.assertRaises(ValueError):
                Options.from_dict(data)
        self.assertEqual(Options.from_dict({'start_delay': '0.25'}).start_delay, '0.25')

    def test_no_inherited_cross_install_overrides(self):
        with patch.dict(os.environ, {'AVIATOR_BIN': '/wrong', 'CAMERA_PYTHON': '/wrong', 'MONITOR_BIN': '/wrong'}):
            layout = resolve_layout(self.root)
            special = str(self.base / 'binary $(touch never); 空格')
            env = Options(bin_dir=special).environment(layout)
            self.assertEqual(env['AVIATOR_BIN'], special)
            self.assertEqual(env['MONITOR_BIN'], special + '/aviator_monitor')
            self.assertNotEqual(env['CAMERA_PYTHON'], '/wrong')

    def make_install(self, prefix):
        resource = prefix / 'share/aviator'
        shutil.copytree(self.root, resource)
        (prefix / 'bin').mkdir()
        (prefix / 'release-version.json').write_text('{"version":"6.0.0"}')
        (resource / 'CMakeLists.txt').unlink()
        return resource

    def make_build(self, relative, names, timestamp):
        directory = self.root / relative
        directory.mkdir(parents=True, exist_ok=True)
        for name in names:
            binary = directory / name
            binary.write_text('#!/bin/sh\nexit 0\n')
            binary.chmod(0o755)
            os.utime(binary, (timestamp, timestamp))
        return directory

    def test_auto_build_release_priority_and_latest_fallback(self):
        layout = resolve_layout(self.root)
        options = Options()
        release = self.make_build('build/release/bin', options.required_binaries(), 100)
        debug = self.make_build('build/debug/bin', options.required_binaries(), 300)
        legacy = self.make_build('build/bin', options.required_binaries(), 200)
        self.assertEqual(options.environment(layout)['AVIATOR_BIN'], str(release))
        # A non-executable Release program makes that build incomplete.
        (release / 'aviator_bus').chmod(0o644)
        self.assertEqual(options.binary_directory(layout), debug)
        # Use required executable timestamps, not directory mtime or unrelated files.
        os.utime(legacy / 'simulation', (400, 400))
        (debug / 'unrelated-new-file').touch()
        self.assertEqual(options.binary_directory(layout), legacy)
        self.assertEqual(options.environment(layout)['MONITOR_BIN'], str(legacy / 'aviator_monitor'))

    def test_auto_build_matches_mode_optional_nodes_and_monitor_override(self):
        layout = resolve_layout(self.root)
        release = self.make_build('build/release/bin', Options().required_binaries(), 100)
        debug = self.make_build('build/debug/bin', Options(mode='hardware', logger=True, tts=True).required_binaries(), 200)
        self.assertEqual(Options().binary_directory(layout), release)
        for mode in ('hardware', 'fake-hand'):
            self.assertEqual(Options(mode=mode, logger=True, tts=True).binary_directory(layout), debug)
        self.make_build('build/debug/bin', ['simulation'], 200)
        self.assertEqual(Options(logger=True).binary_directory(layout), debug)
        self.assertEqual(Options(tts=True).binary_directory(layout), debug)
        (release / 'aviator_monitor').unlink()
        self.assertEqual(Options().binary_directory(layout), debug)
        self.assertEqual(Options(monitor_bin='/custom/monitor').binary_directory(layout), release)

    def test_auto_build_preview_incomplete_and_explicit_override(self):
        layout = resolve_layout(self.root)
        self.assertEqual(Options().binary_directory(layout), self.root / 'build/bin')
        release = self.make_build('build/release/bin', ['aviator_bus'], 100)
        self.assertEqual(Options().binary_directory(layout), release)
        with self.assertRaisesRegex(ValueError, 'flight_gateway'):
            Options().preflight(layout)
        debug = self.make_build('build/debug/bin', Options().required_binaries(), 200)
        self.assertEqual(Options().binary_directory(layout), debug)
        # Explicit selections never silently switch to an automatic candidate.
        self.assertEqual(Options(bin_dir=str(release)).binary_directory(layout), release)
        missing = self.root / 'custom missing'
        self.assertEqual(Options(bin_dir=str(missing)).binary_directory(layout), missing)
        with self.assertRaisesRegex(ValueError, 'aviator_bus'):
            Options(bin_dir=str(missing)).preflight(layout)

    def test_install_nested_in_checkout_and_relocation(self):
        prefix = self.root / 'build/release/dist'
        resource = prefix / 'share/aviator'
        (resource / 'scripts').mkdir(parents=True)
        (resource / 'config').mkdir()
        (resource / 'scripts/start_aviator.sh').write_text('# package script')
        (prefix / 'bin').mkdir()
        (prefix / 'release-version.json').write_text('{"version":"6.0.0"}')
        self.assertEqual(resolve_layout(prefix).script, resource / 'scripts/start_aviator.sh')
        moved = self.base / '移动后的 安装包'
        shutil.move(prefix, moved)
        layout = resolve_layout(moved)
        self.assertEqual(layout.kind, 'installed')
        self.assertEqual(layout.bin, moved / 'bin')
        # Even with complete builds inside the resource tree, installed mode stays local.
        source_bin = self.make_build('build/release/bin', Options().required_binaries(), 100)
        shutil.copytree(source_bin, layout.root / 'build/release/bin')
        self.assertEqual(Options().binary_directory(layout), moved / 'bin')
        (moved / 'release-version.json').unlink()
        with self.assertRaises(ValueError):
            resolve_layout(moved)
        with self.assertRaises(ValueError):
            resolve_layout(self.root / 'build')

    def test_settings_source_isolation_and_corruption(self):
        settings = Settings()
        layout = resolve_layout(self.root)
        settings.save(layout, Options(logger=True))
        self.assertTrue(Settings().get(layout).logger)
        self.assertFalse(Settings().get(resolve_layout(REPO)).logger)
        settings.path.write_text('{broken')
        self.assertTrue(Settings().warning)
        self.assertFalse(Settings().get(layout).logger)

    def test_desktop_registration_and_ownership(self):
        path = desktop.register()
        self.assertIn('--show', path.read_text())
        self.assertNotIn('sh -c', path.read_text())
        self.assertTrue(desktop.registered())
        desktop.register(False)
        self.assertFalse(path.exists())
        path.write_text('[Desktop Entry]\nName=Someone else\n')
        with self.assertRaises(ValueError):
            desktop.register()
        start = desktop.register(True, True)
        self.assertIn('--tray', start.read_text())
        self.assertNotIn('start_aviator.sh', start.read_text())
        self.assertIn('%%', desktop.exec_quote('/a%name'))
        with self.assertRaises(ValueError):
            desktop.exec_quote('bad\npath')

    def test_private_runtime_and_bounded_protocol(self):
        self.assertEqual(runtime_dir().stat().st_mode & 0o777, 0o700)
        self.runtime.chmod(0o755)
        with self.assertRaises(ValueError):
            runtime_dir()
        left, right = socket.socketpair()
        with left, right:
            right.sendall(b'{not json}\n')
            with self.assertRaises(ValueError):
                receive(left)

    def test_unknown_run_survives_manager_restart_and_rechecks_identity(self):
        directory = runtime_dir()
        record = {'status': 'NEEDS_ATTENTION', 'pid': os.getpid(),
                  'identity': process_identity(os.getpid()), 'root': str(self.root)}
        (directory / 'state.json').write_text(json.dumps(record))
        manager = Runner(directory)
        self.assertEqual(manager.state['status'], 'NEEDS_ATTENTION')
        with self.assertRaises(ValueError):
            manager.recheck()
        # A recycled PID with a different start time is never signaled.
        manager.state['identity'] = ['previous-boot', '0']
        manager.recheck()
        self.assertEqual(manager.state['status'], 'IDLE')

    def test_cmake_desktop_file_is_valid(self):
        if not shutil.which('desktop-file-validate'):
            self.skipTest('desktop-file-validate unavailable')
        entry = desktop.register()
        result = subprocess.run(['desktop-file-validate', str(entry)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def start_runner(self, script):
        (self.root / 'scripts/start_aviator.sh').write_text(script)
        binaries = self.root / 'build/bin'
        binaries.mkdir(parents=True)
        for name in ('aviator_bus', 'flight_gateway', 'aviator_core_managed', 'aviator_monitor', 'simulation', 'google-chrome-stable', 'curl'):
            path = binaries / name
            path.write_text('#!/bin/sh\nexit 0\n')
            path.chmod(0o755)
        self.path_env = patch.dict(os.environ, {'PATH': str(binaries) + ':' + os.environ['PATH']})
        self.path_env.start()
        self.addCleanup(self.path_env.stop)
        self.runner_log = (self.base / 'runner-output').open('w')
        process = subprocess.Popen([sys.executable, '-B', str(MODULE / 'runner.py')],
            stdout=self.runner_log, stderr=subprocess.STDOUT)
        def cleanup():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            self.runner_log.close()
        self.addCleanup(cleanup)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            try:
                request('status')
                return process
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(.03)
        self.fail((self.base / 'runner-output').read_text())

    def wait_state(self, expected):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = request('status')
            if state['status'] == expected:
                return state
            time.sleep(.05)
        self.fail(str(state))

    def test_runner_start_duplicate_stop_and_reconnect(self):
        process = self.start_runner('''#!/bin/bash
trap 'echo cleaned; sleep .2; exit 143' TERM
echo '日志目录: /tmp/example-node-logs'
echo '所有节点已启动；保持此终端打开，按 Ctrl+C 停止。'
while true; do sleep .05; done
''')
        state = request('start', root=str(self.root), options={})
        pid = state['pid']
        self.assertEqual(state['identity'], process_identity(pid))
        with self.assertRaises(ValueError):
            request('start', root=str(self.root), options={})
        running = self.wait_state('RUNNING')
        self.assertEqual(running['node_log_dir'], '/tmp/example-node-logs')
        # Each call makes a new connection, modeling a UI disconnect/reconnect.
        self.assertEqual(request('status')['pid'], pid)
        request('stop')
        request('stop')
        stopped = self.wait_state('IDLE')
        self.assertEqual(stopped['exit_code'], 143)
        self.assertIn('cleaned', stopped['output'])
        request('quit')
        process.wait(timeout=3)

    def test_failure_and_external_lock(self):
        self.start_runner('#!/bin/bash\necho startup-failure\nexit 7\n')
        with (self.runtime / f'aviator-start-{os.getuid()}.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            with self.assertRaises(ValueError):
                request('start', root=str(self.root), options={})
        request('start', root=str(self.root), options={})
        self.assertEqual(self.wait_state('FAILED')['exit_code'], 7)

    def test_runner_rejects_unknown_actions_and_bad_messages(self):
        self.start_runner('#!/bin/bash\nexit 0\n')
        with self.assertRaises(ValueError):
            request('execute', command='touch bad')
        with socket.socket(socket.AF_UNIX) as sock:
            sock.connect(str(runtime_dir() / 'control.sock'))
            sock.sendall(b'[]\n')
            self.assertFalse(receive(sock)['ok'])
        self.assertEqual(request('status')['status'], 'IDLE')

    def test_cmake_install_entry_is_relocatable(self):
        # Exercise the actual standalone tool install rules without rebuilding robotics.
        source, build, prefix = self.base / 'cmake', self.base / 'cmake-build', self.base / 'package'
        source.mkdir()
        (source / 'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.22)\nproject(Launcher NONE)\ninclude(GNUInstallDirs)\n' +
            f'add_subdirectory("{MODULE}" launcher)\n')
        for command in [['cmake', '-S', str(source), '-B', str(build)],
                        ['cmake', '--install', str(build), '--prefix', str(prefix)]]:
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
        resource = prefix / 'share/aviator'
        shutil.copytree(self.root / 'scripts', resource / 'scripts')
        (resource / 'config').mkdir()
        (prefix / 'release-version.json').write_text('{"version":"6.0.0"}')
        moved = self.base / 'moved package'
        shutil.move(prefix, moved)
        self.assertFalse((moved / 'bin/aviator-launcher').exists())
        desktop_entry = moved / 'share/applications/org.aviator.Launcher.desktop'
        self.assertIn('Exec=aviator_launcher --show\n', desktop_entry.read_text())
        result = subprocess.run([str(moved / 'bin/aviator_launcher'), '--check'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['bin'], str(moved / 'bin'))
        self.assertFalse(list(moved.rglob('__pycache__')))


if __name__ == '__main__':
    unittest.main()
