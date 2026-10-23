#!/usr/bin/env python3
"""Root-side installer. Never starts robot nodes or enables an autostart service."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import pwd
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import uuid

PREFIX = Path('/opt/aviator')
RELEASES = Path('/opt/aviator-releases')  # Legacy layout, used only for migration/cleanup.
WORK = Path('/opt/.aviator-install')
DATA = Path('/opt/aviator-data')
CONFIG_PATHS = ('share/aviator/config', 'share/aviator/monitor/monitor.yaml',
                'share/aviator/examples/xCoreSDK_RH56FTP_485/config.ini')


def run(args, **kwargs):
    print('+ ' + ' '.join(map(str, args)), flush=True)
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def release_module(root):
    spec = importlib.util.spec_from_file_location('aviator_release', root / 'share/aviator/scripts/aviator_release.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def safe_extract(archive, directory):
    """Extract only regular files/directories and contained file links; no tar traversal."""
    with tarfile.open(archive) as tar:
        members = tar.getmembers()
        names = {}
        for entry in members:
            path = PurePosixPath(entry.name)
            if (path.is_absolute() or '..' in path.parts or not path.parts or
                    path.parts[0] != 'aviator' or '\\' in entry.name or
                    any(c in entry.name for c in '\n\r\x00')):
                raise ValueError(f'Unsafe archive path: {entry.name}')
            if entry.name in names or not (entry.isfile() or entry.isdir() or entry.issym() or entry.islnk()):
                raise ValueError(f'Unsupported archive entry: {entry.name}')
            names[entry.name] = entry
        for entry in members:
            for parent in PurePosixPath(entry.name).parents:
                if str(parent) in names and not names[str(parent)].isdir():
                    raise ValueError(f'Archive parent is not a directory: {entry.name}')
            if entry.issym() or entry.islnk():
                base = directory / entry.name
                target = (base.parent / entry.linkname if entry.issym() else directory / entry.linkname).resolve()
                if not target.is_relative_to((directory / 'aviator').resolve()):
                    raise ValueError(f'Escaping archive link: {entry.name}')
        for entry in members:
            path = directory / entry.name
            if entry.isdir():
                path.mkdir(parents=True, exist_ok=True)
            elif entry.isfile():
                path.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(entry) as source, path.open('xb') as destination:
                    shutil.copyfileobj(source, destination)
                path.chmod(0o755 if entry.mode & 0o111 else 0o644)
        for entry in members:
            path = directory / entry.name
            if entry.issym():
                path.symlink_to(entry.linkname)
            elif entry.islnk():
                target = directory / entry.linkname
                if target.is_symlink() or not target.is_file():
                    raise ValueError(f'Invalid hardlink: {entry.name}')
                os.link(target, path)


def deployment_record(path):
    marker = path / 'deployment.json'
    if marker.is_symlink() or not marker.is_file():
        raise RuntimeError(f'Not a managed AVIATOR installation: {path}')
    record = json.loads(marker.read_text())
    if record.get('format') not in (1, 2) or not record.get('user'):
        raise RuntimeError(f'Invalid deployment record: {path}')
    return record


def current_release():
    if not PREFIX.exists() and not PREFIX.is_symlink():
        return None
    if PREFIX.is_symlink():
        current = PREFIX.resolve(strict=True)
        if current.parent != RELEASES:
            raise RuntimeError('Refusing to replace an unmanaged /opt/aviator link')
    else:
        current = PREFIX
    deployment_record(current)
    return current


def remove_path(path):
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.exists():
        shutil.rmtree(path)


def preserve_configuration(previous, new):
    """Overlay whole on-device config files; retain defaults for new filenames only."""
    if previous is None:
        return

    def validate(path):
        if path.is_symlink():
            raise RuntimeError(f'Configuration symlink is unsupported: {path}')
        if path.is_dir():
            for child in path.iterdir():
                validate(child)
        elif path.exists() and not path.is_file():
            raise RuntimeError(f'Unsupported configuration file: {path}')

    def overlay(source, destination):
        if source.is_dir():
            if destination.exists() and not destination.is_dir():
                raise RuntimeError(f'Configuration file/directory conflict: {destination}')
            destination.mkdir(parents=True, exist_ok=True)
            for child in source.iterdir():
                overlay(child, destination / child.name)
        elif source.is_file():
            if destination.is_dir():
                raise RuntimeError(f'Configuration file/directory conflict: {destination}')
            destination.parent.mkdir(parents=True, exist_ok=True)
            # Break any archive hardlink before copying, without changing the old installation.
            destination.unlink(missing_ok=True)
            shutil.copy2(source, destination)

    for name in CONFIG_PATHS:
        for root in (previous, new):
            path = root
            for part in Path(name).parts:
                path /= part
                if path.is_symlink():
                    raise RuntimeError(f'Configuration symlink is unsupported: {path}')
            validate(path)
        overlay(previous / name, new / name)


def write_transaction(record):
    temporary = WORK / 'transaction.json.tmp'
    with temporary.open('w') as stream:
        json.dump(record, stream)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, WORK / 'transaction.json')


def cleanup_legacy():
    # Delete only directories positively identified as former deployments.
    if not RELEASES.exists() or RELEASES.is_symlink():
        return
    for path in RELEASES.iterdir():
        if path.is_symlink() or not path.is_dir():
            continue
        try:
            record = deployment_record(path)
        except (OSError, ValueError, RuntimeError):
            continue
        if record['format'] == 1:
            shutil.rmtree(path)
    if not any(RELEASES.iterdir()):
        RELEASES.rmdir()


def recover_transaction():
    if not WORK.exists() and not WORK.is_symlink():
        return
    if WORK.is_symlink() or not (WORK / 'transaction.json').is_file():
        raise RuntimeError(f'Unrecognized temporary directory: {WORK}; inspect it before retrying')
    record = json.loads((WORK / 'transaction.json').read_text())
    if record.get('format') != 1 or not re.fullmatch(r'[0-9a-f]{32}', record.get('id', '')):
        raise RuntimeError('Invalid interrupted deployment record')
    backup = WORK / 'previous'
    if record['phase'] == 'committed':
        # The final-path checks passed; only housekeeping was interrupted.
        if PREFIX.is_symlink() or deployment_record(PREFIX).get('transaction') != record['id']:
            raise RuntimeError('Committed installation changed; refusing automatic cleanup')
        remove_path(backup)
        cleanup_legacy()
    elif record['phase'] in ('preparing', 'replacing'):
        installed_new = (PREFIX.exists() and not PREFIX.is_symlink() and
                         (PREFIX / 'deployment.json').is_file() and
                         deployment_record(PREFIX).get('transaction') == record['id'])
        has_backup = backup.exists() or backup.is_symlink()
        if installed_new:
            remove_path(PREFIX)
        if has_backup:
            if PREFIX.exists() or PREFIX.is_symlink():
                raise RuntimeError('Installation changed during recovery; backup retained')
            os.replace(backup, PREFIX)
    else:
        raise RuntimeError('Unknown deployment transaction phase')
    shutil.rmtree(WORK)


def replace_installation(new, account, record):
    """Use a temporary backup only; rebuild venv at its permanent path before commit."""
    ensure_idle()
    record['phase'] = 'replacing'
    write_transaction(record)
    if PREFIX.exists() or PREFIX.is_symlink():
        os.replace(PREFIX, WORK / 'previous')
    os.replace(new, PREFIX)
    # Virtualenv entrypoints contain absolute paths: never keep staging shebangs.
    prepare_python(PREFIX, account)
    check_runtime(PREFIX, account)
    release_module(PREFIX).verify(PREFIX)
    desktop_entries(account)
    ensure_idle()
    record['phase'] = 'committed'
    write_transaction(record)


def ensure_idle():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            exe = os.readlink(proc / 'exe')
            args = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            # Ignore this installer and its sudo/ssh wrappers only.
            if 'remote.py' in args:
                continue
            if any(marker in exe or marker in args for marker in (str(PREFIX) + '/', str(RELEASES) + '/')):
                raise RuntimeError(f'Deployed process {proc.name} is still running; stop it before deployment')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue



def user_run(account, command, **kwargs):
    env = ['env', 'PYTHONDONTWRITEBYTECODE=1', 'PYTHONNOUSERSITE=1',
           'HOME=' + account.pw_dir]
    return run(['runuser', '-u', account.pw_name, '--', *env, *command], **kwargs)


def prepare_python(root, account):
    resource = root / 'share/aviator'
    cache = root / 'cache'
    cache.mkdir(exist_ok=True)
    os.chown(cache, account.pw_uid, account.pw_gid)
    remove_path(cache / 'python')
    python = cache / 'python/bin/python'
    user_run(account, ['/usr/bin/python3.10', '-m', 'venv', cache / 'python'])
    user_run(account, [python, '-m', 'pip', 'install', '--no-index', '--find-links',
                       resource / 'deploy/wheels', '-r', resource / 'deploy/requirements.txt'])


def setup_dependencies(root, account, skip_apt=False):
    resource = root / 'share/aviator'
    manifest = json.loads((resource / 'deploy/manifest.json').read_text())
    if manifest.get('ubuntu') != '22.04' or manifest.get('architecture') != 'x86_64':
        raise RuntimeError('Unsupported runtime manifest')
    packages = manifest['apt_packages']
    if not all(re.fullmatch(r'[a-z0-9][a-z0-9+.-]*(?::amd64)?', p) for p in packages):
        raise ValueError('Invalid apt package name')
    # This runtime targets CPython 3.10. The generic metapackage pins the
    # python3-defaults revision, which can differ between target apt repositories.
    # Normalize older manifests too so a retained archive can be retried.
    packages = sorted({'python3.10-venv' if p.split(':')[0] == 'python3-venv' else p
                       for p in packages})
    if not skip_apt:
        run(['apt-get', 'update'])
        run(['apt-get', 'install', '-y', '--no-remove', '--no-install-recommends', *packages])
        if not shutil.which('google-chrome-stable'):
            run(['apt-get', 'install', '-y', '--no-remove',
                 resource / 'deploy/google-chrome-stable_current_amd64.deb'])
    prepare_python(root, account)
    # Startup helpers are called both through shell launchers and directly by desktop tools.
    logs = DATA / 'logs'
    logs.mkdir(parents=True, exist_ok=True)
    os.chown(logs, account.pw_uid, account.pw_gid)
    (resource / 'logs').symlink_to(logs)
    # The on-device TTS generator writes audio and its manifest here.
    for name in (*CONFIG_PATHS, 'share/aviator/assets/tts'):
        path = root / name
        if path.exists():
            for item in [path, *(path.rglob('*') if path.is_dir() else [])]:
                os.chown(item, account.pw_uid, account.pw_gid)
    run(['usermod', '-aG', 'dialout,video,plugdev', account.pw_name])
    rules = resource / 'deploy/99-aviator-realsense.rules'
    shutil.copy2(rules, '/etc/udev/rules.d/99-aviator-realsense.rules')
    run(['udevadm', 'control', '--reload-rules'])
    # Avoid triggering/rebinding devices while other robot software may be using them.
    print('设备组权限在重新登录后生效；相机规则在重新插拔设备或重启后生效。', flush=True)


def check_runtime(root, account):
    resource = root / 'share/aviator'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', QT_QPA_PLATFORM='offscreen')
    for directory in ('bin', 'lib'):
        for path in (root / directory).rglob('*'):
            if not path.is_file() or path.is_symlink():
                continue
            with path.open('rb') as stream:
                if stream.read(4) != b'\x7fELF':
                    continue
            result = run(['ldd', path], capture_output=True, text=True)
            if 'not found' in result.stdout:
                raise RuntimeError(f'Missing runtime library: {path}\n{result.stdout}')
    python = root / 'cache/python/bin/python'
    user_run(account, [python, '-c', 'import tkinter, yaml, numpy, zmq, cv2, pyrealsense2, '
                      'apriltag, pymodbus, matplotlib, sherpa_onnx; '
                      'assert hasattr(cv2.aruco, "CharucoDetector"); '
                      'apriltag.Detector(); print("Python runtime OK")'], env=env)
    for command in ('google-chrome-stable', 'curl', 'aplay', 'zenity'):
        if not shutil.which(command):
            raise RuntimeError(f'Missing command: {command}')
    for script in resource.rglob('*.sh'):
        run(['bash', '-n', script], stdout=subprocess.DEVNULL)
    for script in ('start_aviator.sh', 'start_camera_servo.sh', 'start_wheel_bandwidth.sh'):
        user_run(account, ['bash', resource / 'scripts' / script, '--dry-run'], env=env,
                 stdout=subprocess.DEVNULL)
    user_run(account, [root / 'bin/aviator_photoelectric', '--config',
                       resource / 'config/photoelectric.yaml', '--check-config'], env=env)
    user_run(account, [root / 'bin/rh56_485_probe', '--dry-run'], env=env)
    # Help modes parse options/import modules but do not open devices.
    for script in ('start_grasp_calibration_ui.sh', 'start_steering_wheel_calibration_ui.sh'):
        user_run(account, ['bash', resource / 'scripts' / script, '--help'], env=env,
                 stdout=subprocess.DEVNULL)
    user_run(account, ['bash', resource / 'examples/xCoreSDK_RH56FTP_485/run_grip_demo.sh', '--help'],
             env=env, stdout=subprocess.DEVNULL)


def desktop_entries(account):
    applications = Path(account.pw_dir) / '.local/share/applications'
    applications.mkdir(parents=True, exist_ok=True)
    # Make newly created per-user directories usable without changing existing unrelated trees.
    for path in (Path(account.pw_dir) / '.local', Path(account.pw_dir) / '.local/share', applications):
        os.chown(path, account.pw_uid, account.pw_gid)
    entries = {
        'aviator-launcher': ('AVIATOR 启动器', '/opt/aviator/bin/aviator_launcher --show'),
        'aviator-hand-485': ('AVIATOR 双手 485 测试', '/opt/aviator/share/aviator/examples/xCoreSDK_RH56FTP_485/run.sh'),
        'aviator-grip-demo': ('AVIATOR 手部演示', '/opt/aviator/share/aviator/examples/xCoreSDK_RH56FTP_485/run_grip_demo.sh'),
    }
    for name, (title, command) in entries.items():
        path = applications / (name + '.desktop')
        path.write_text(f'[Desktop Entry]\nType=Application\nName={title}\nExec={command}\n'
                        'Terminal=false\nCategories=Utility;\n')
        os.chown(path, account.pw_uid, account.pw_gid)


def install(args):
    if not re.fullmatch(r'[0-9a-f]{64}', args.sha256 or '') or digest(args.archive) != args.sha256:
        raise RuntimeError('Archive SHA256 mismatch')
    account = pwd.getpwnam(args.user)
    if account.pw_uid == 0:
        raise RuntimeError('Deployment account must be a normal user')
    previous = current_release()  # Reject unrelated data; support both managed layouts.
    with tarfile.open(args.archive) as archive:
        required_space = sum(entry.size for entry in archive.getmembers() if entry.isfile()) * 2 + 512 * 1024 * 1024
    if shutil.disk_usage(PREFIX.parent).free < required_space:
        raise RuntimeError(f'Insufficient /opt space: need approximately {required_space // 1024 // 1024} MiB free')
    WORK.mkdir(mode=0o755)
    transaction = {'format': 1, 'id': uuid.uuid4().hex, 'phase': 'preparing'}
    write_transaction(transaction)
    try:
        safe_extract(args.archive, WORK)
        new = WORK / 'aviator'
        module = release_module(new)
        module.verify(new)
        # Verify the shipped defaults first, then validate using the retained field settings.
        preserve_configuration(previous, new)
        setup_dependencies(new, account)
        check_runtime(new, account)
        record = {'format': 2, 'layout': 'fixed-directory', 'user': args.user,
                  'configuration_policy': 'preserve-existing',
                  'archive_sha256': args.sha256, 'transaction': transaction['id']}
        (new / 'deployment.json').write_text(json.dumps(record, indent=2) + '\n')
        module.seal(new)
        module.verify(new)
        replace_installation(new, account, transaction)
    except BaseException:
        recover_transaction()
        print('部署失败，已恢复原安装；未启动设备。', file=sys.stderr)
        raise
    # Once committed, cleanup errors must not roll a validated installation back.
    recover_transaction()
    print(f'部署成功：{PREFIX}（固定实体目录）。程序和 Python 环境已替换，已有配置已保留，新增配置已补齐。')
    print(f'运行日志继续保存在 {DATA}/logs；未启动设备或配置开机自启。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path)
    parser.add_argument('--sha256')
    parser.add_argument('--user')
    parser.add_argument('--refresh-checksums', action='store_true', help='Record intentional configuration edits')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error('Run through sudo')
    if args.refresh_checksums == bool(args.archive):
        parser.error('Choose archive installation or refresh-checksums')
    if args.archive and (not args.user or not args.sha256):
        parser.error('--archive requires --user and --sha256')
    if os.uname().machine != 'x86_64' or 'VERSION_ID="22.04"' not in Path('/etc/os-release').read_text():
        parser.error('Target must be Ubuntu 22.04 x86_64')
    with open('/run/lock/aviator-deploy.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ensure_idle()
        recover_transaction()
        if args.refresh_checksums:
            current = current_release()
            if current is None:
                raise RuntimeError('No managed deployment')
            release_module(current).seal(current)
            release_module(current).verify(current)
            print('已按当前配置更新校验清单。')
        else:
            install(args)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.CalledProcessError) as error:
        sys.exit(f'Deployment failed: {error}')
