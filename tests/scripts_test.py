"""Offline regression for every scripts/ entry point. No robot motion/system installs."""
import ast
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'

class ScriptsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='aviator script tests ')
        self.directory = Path(self.temp.name)
    def tearDown(self):
        self.temp.cleanup()
    def run_script(self, name, *args, ok=True, env=None, timeout=15):
        path = SCRIPTS / name
        command = ['bash' if path.suffix == '.sh' else sys.executable, str(path), *map(str,args)]
        p = subprocess.run(command, cwd=self.directory, env={**os.environ, **(env or {})},
                           text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        self.assertEqual(p.returncode == 0, ok, p.stdout)
        return p.stdout
    def test_all_entrypoints_syntax_help_and_bad_arguments(self):
        for path in sorted(SCRIPTS.iterdir()):
            if path.suffix not in ('.sh','.py'): continue
            with self.subTest(script=path.name):
                if path.suffix == '.sh':
                    subprocess.run(['bash','-n',str(path)], check=True)
                else: ast.parse(path.read_text())
                self.run_script(path.name,'--help')
                self.run_script(path.name,'--this-option-does-not-exist',ok=False)
    def test_control_launch_matrix(self):
        for name in ('start_aviator.sh','start_camera_servo.sh'):
            for options in ([],['--fake-hand'],['--logger'],['--fake-hand','--logger']):
                text=self.run_script(name,'--dry-run',*options,
                                     env={'AVIATOR_BIN':str(self.directory/'binary dir'),'HAND_PYTHON':'/missing/python'})
                self.assertIn('manipulator',text)
                self.assertIn('xCoreSDK_hand',text)
                arm=next(line for line in text.splitlines() if line.startswith('启动 manipulator:'))
                self.assertNotIn('--fake-hand',arm)
                self.assertNotIn('rh56ftp_node.py',text)
                self.assertNotIn('fake_rh56ftp_hand.py',text)
                self.assertEqual('--fake-hand' in text,'--fake-hand' in options)
        text=self.run_script('start_aviator.sh','--dry-run','--simulation','--headless','--logger','--tts')
        self.assertIn('simulation',text);self.assertNotIn('/manipulator ',text)
        for flags, system in ((['--rs422'], 'system-rs422.yaml'),
                              (['--rs422','--simulation','--headless'], 'system-simulation-rs422.yaml')):
            text=self.run_script('start_aviator.sh','--dry-run',*flags)
            self.assertIn('flight-rs422.yaml',text)
            self.assertIn(system,text)
        self.run_script('start_aviator.sh','--dry-run','--simulation','--fake-hand',ok=False)
        self.run_script('start_aviator.sh','--dry-run','--headless',ok=False)
        self.assertEqual(list(self.directory.iterdir()),[],'dry-run wrote files')

    def test_recording_directory_override_and_early_failure(self):
        output = self.directory/'recordings 空格'
        text = self.run_script('start_aviator.sh','--simulation','--logger','--dry-run',
                               env={'AVIATOR_RECORDING_DIR':str(output)})
        self.assertIn('recordings',text)
        self.assertFalse(output.exists(),'dry-run created recording directory')
        text = self.run_script('start_aviator.sh','--logger','--dry-run',ok=False,
                              env={'AVIATOR_RECORDING_DIR':'relative'})
        self.assertIn('必须为绝对目录',text)
        mock = self.directory/'mock-bin'
        mock.mkdir()
        marker = self.directory/'unexpected-node'
        for name in ('aviator_bus','flight_gateway','aviator_core_managed','aviator_monitor',
                     'simulation','aviator_logger','google-chrome-stable','curl'):
            file = mock/name
            file.write_text('#!/usr/bin/python3\nfrom pathlib import Path\nPath('+repr(str(marker))+').touch()\n')
            file.chmod(0o755)
        blocked = self.directory/'not-a-directory'
        blocked.write_text('occupied')
        self.run_script('start_aviator.sh','--simulation','--logger',ok=False,
                        env={'AVIATOR_RECORDING_DIR':str(blocked/'child'), 'AVIATOR_BIN':str(mock),
                             'MONITOR_BIN':str(mock/'aviator_monitor'),
                             'PATH':str(mock)+':'+os.environ['PATH']})
        self.assertFalse(marker.exists(),'recording path failure started a node')
    def test_launchers_own_processes_and_stop_core_before_manipulator(self):
        mock = self.directory / "fake bin"
        mock.mkdir()
        events = self.directory / "events"
        def cleanup_mock_nodes():
            if events.exists():
                for row in events.read_text().splitlines():
                    if row.startswith('start '):
                        try: os.kill(int(row.split()[2]), signal.SIGKILL)
                        except ProcessLookupError: pass
        self.addCleanup(cleanup_mock_nodes)
        node = mock / "node"
        node.write_text("#!/usr/bin/python3\n" +
            "import os,signal,sys,time\n" +
            "name=os.path.basename(sys.argv[0])\n" +
            "def event(kind):\n    with open(os.environ['TEST_EVENTS'],'a') as f: f.write(kind+' '+name+' '+str(os.getpid())+'\\n')\n" +
            "def stop(*args):\n    signal.signal(signal.SIGTERM,signal.SIG_IGN)\n    event('stop')\n    if name.startswith('aviator_core'): time.sleep(.4)\n    sys.exit(0)\n" +
            "signal.signal(signal.SIGTERM,stop)\nevent('start')\n" +
            "if name in ('manipulator','camera-python') and os.fork()==0:\n" +
            "    name+='-worker'\n    signal.signal(signal.SIGTERM,signal.SIG_IGN)\n    event('start')\n" +
            "while True: time.sleep(.02)\n")
        node.chmod(0o755)
        for name in ('aviator_bus','manipulator','xCoreSDK_hand','flight_gateway','aviator_core_managed',
                     'aviator_core_camera_servo','camera-python','aviator_monitor'):
            (mock/name).symlink_to(node)
        for name in ('google-chrome-stable','curl'):
            (mock/name).write_text('#!/bin/bash\nexit 0\n')
            (mock/name).chmod(0o755)
        # Never invoke real sudo. Consume only the launcher's own stdin password.
        (mock/'sudo').write_text('#!/usr/bin/python3\n' +
            'import os,subprocess,sys\n' +
            'args=sys.argv[1:]\n' +
            'while args:\n' +
            '    if args[0]=="-S": sys.stdin.readline();args=args[1:]\n' +
            '    elif args[0]=="-p": args=args[2:]\n' +
            '    elif args[0]=="-v": sys.exit(0)\n' +
            '    else: break\n' +
            'if args[0]=="kill": os.execvp(args[0],args)\n' +
            '# Model sudo use_pty: the actual command runs in a different session.\n' +
            'child=subprocess.Popen(args,start_new_session=True)\n' +
            'sys.exit(child.wait())\n')
        (mock/'sudo').chmod(0o755)
        env = {**os.environ, 'PATH':str(mock)+':'+os.environ['PATH'], 'AVIATOR_BIN':str(mock),
               'CAMERA_PYTHON':str(mock/'camera-python'), 'START_DELAY':'.1',
               'TEST_EVENTS':str(events), 'TMPDIR':str(self.directory), 'XDG_RUNTIME_DIR':str(self.directory)}
        # START_DELAY deliberately uses the accepted decimal spelling.
        env['START_DELAY']='0.1'
        cases = [(script, core, stop_signal)
                 for script, core in (('start_aviator.sh','aviator_core_managed'),
                                      ('start_camera_servo.sh','aviator_core_camera_servo'))
                 for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)]
        for script, core, stop_signal in cases:
            events.write_text('')
            log=self.directory/(script+'.log')
            with log.open('w') as stream:
                child=subprocess.Popen(['bash',str(SCRIPTS/script),'--fake-hand'],env=env,
                                       stdout=stream,stderr=subprocess.STDOUT)
                try:
                    until=time.monotonic()+10
                    while time.monotonic()<until and child.poll() is None:
                        text=log.read_text()
                        if '所有节点已启动' in text or '相机随动测试已启动' in text:break
                        time.sleep(.05)
                    self.assertIsNone(child.poll(),log.read_text())
                    self.assertIn('start '+core,events.read_text())
                    blocked=self.run_script(script,'--fake-hand',ok=False,env=env)
                    self.assertIn('仍持有启动锁',blocked)
                finally:
                    child.send_signal(stop_signal)
                    if stop_signal == signal.SIGINT:
                        until=time.monotonic()+3
                        while time.monotonic()<until and 'stop '+core+' ' not in events.read_text():
                            time.sleep(.01)
                        # A second Ctrl+C/TERM/HUP during Core shutdown must not abandon other nodes.
                        for repeated in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                            if child.poll() is None: child.send_signal(repeated)
                    try:child.wait(timeout=20)
                    except subprocess.TimeoutExpired:child.kill();child.wait();raise
            self.assertEqual(child.returncode,128+stop_signal,log.read_text())
            subprocess.run(['flock','-n',str(self.directory/f'aviator-start-{os.getuid()}.lock'),
                            'true'],check=True)
            rows=events.read_text().splitlines()
            core_stop=next(i for i,row in enumerate(rows) if row.startswith('stop '+core+' '))
            arm_stop=next(i for i,row in enumerate(rows) if row.startswith('stop manipulator '))
            hand_stop=next(i for i,row in enumerate(rows) if row.startswith("stop xCoreSDK_hand "))
            self.assertLess(core_stop,hand_stop,rows)
            self.assertLess(hand_stop,arm_stop,rows)
            for row in rows:
                if row.startswith('start '):
                    pid=int(row.split()[2])
                    try:
                        # Orphaned workers may briefly remain zombies until the system reaps them.
                        stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                    except FileNotFoundError: continue
                    if stat[0]=='Z': continue
                    self.fail(f'launcher left child {pid} running: {rows}')
            self.assertNotIn('SIGKILL 后仍未退出', log.read_text())

    def test_bandwidth_launch_matrix(self):
        for options in ([],['--fake-hand'],['--simulation','--headless']):
            text=self.run_script('start_wheel_bandwidth.sh','--dry-run',*options)
            self.assertNotIn('rh56ftp_node.py',text);self.assertNotIn('fake_rh56ftp_hand.py',text)
            self.assertEqual('--fake-hand' in text,'--fake-hand' in options)
        self.run_script('start_wheel_bandwidth.sh','--dry-run','--simulation','--fake-hand',ok=False)
        self.assertEqual(list(self.directory.iterdir()),[])
    def test_dependency_install_with_mock_package_manager(self):
        mock=self.directory/'bin';mock.mkdir();log=self.directory/'commands'
        for name in ('apt-get','apt-cache','sudo'):
            path=mock/name
            path.write_text('#!/bin/bash\n'+ ('exec "$@"\n' if name=='sudo' else
                'printf "%s %s\\n" "'+name+'" "$*" >> "$TEST_COMMAND_LOG"\n'))
            path.chmod(0o755)
        env={'PATH':str(mock)+':'+os.environ['PATH'],'TEST_COMMAND_LOG':str(log)}
        self.run_script('install_dependencies.sh','--dry-run','--with-examples',env=env)
        self.assertFalse(log.exists())
        self.run_script('install_dependencies.sh','--with-examples',env=env)
        text=log.read_text();self.assertIn('apt-get update',text);self.assertIn('apt-get install',text)
        self.assertIn('protobuf-compiler',text)
    def test_joystick_readonly_preview_and_invalid_device(self):
        self.run_script('setup_joystick_udev.sh','--device','/dev/null','--dry-run',ok=False)
        device=Path('/dev/input/by-id/usb-LiteStar_PXN-F16-event-joystick')
        if device.exists():
            text=self.run_script('setup_joystick_udev.sh','--dry-run')
            self.assertIn('MODE="0640"',text);self.assertIn('仅预览',text)
    def test_split_stl_cli_and_no_overwrite(self):
        source=self.directory/'mesh.stl';source.write_bytes(bytes(80)+struct.pack('<I',1)+bytes(50))
        output=self.directory/'parts'
        # CLI positional argument contract is exercised, including paths with spaces.
        self.run_script('split_stl.py',source,output)
        parts=list(output.glob('*.stl'));self.assertEqual(len(parts),1)
        self.assertEqual(parts[0].read_bytes()[80:],source.read_bytes()[80:])
        self.run_script('split_stl.py',source,output,ok=False)
    def test_tts_wrapper_and_real_audio_validation(self):
        text=self.run_script('generate_tts.sh','--dry-run','--speed','0.9','--output',self.directory/'audio')
        self.assertIn('--check',text);self.assertFalse((self.directory/'audio').exists())
        self.run_script('generate_tts.sh','--check',timeout=30)
    def test_release_cli(self):
        self.run_script('aviator_release.py','verify',self.directory,ok=False)
        out=self.directory/'info.json'
        self.run_script('aviator_release.py','configure','--version','1.2.3','--profile','test',
                        '--output',out,'--compiler','test','--cmake-version','3.22')
        self.assertEqual(json.loads(out.read_text())['version'],'1.2.3')
    def test_calibration_ui_dry_run_http_and_shutdown(self):
        log=self.directory/'ui.log'
        with log.open('w') as stream:
            child=subprocess.Popen(['bash',str(SCRIPTS/'start_grasp_calibration_ui.sh'),
                '--dry-run','--no-browser','--port','0','--log-dir',str(self.directory/'logs')],
                cwd=self.directory,stdout=stream,stderr=subprocess.STDOUT)
            try:
                until=time.monotonic()+10;match=None
                while time.monotonic()<until and child.poll() is None:
                    match=re.search(r'http://127.0.0.1:\d+/',log.read_text())
                    if match:break
                    time.sleep(.05)
                self.assertIsNotNone(match,log.read_text())
                opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
                with opener.open(match.group(),timeout=3) as response:self.assertEqual(response.status,200)
            finally:
                child.terminate()
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:child.kill();child.wait();raise
            self.assertEqual(child.returncode,0,log.read_text())

if __name__=='__main__': unittest.main()
