#!/usr/bin/env python3
"""Install a verified evaluation-only archive in an independently owned directory."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import py_compile
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile

FILES = {'evaluate_aviator.py', 'README.md', 'manifest.json'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verified(directory, *, allow_extra=False):
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError(f'拒绝操作非实体评估目录: {directory}')
    marker = directory / 'manifest.json'
    if marker.is_symlink() or not marker.is_file():
        raise ValueError(f'缺少评估包标识: {directory}')
    data = json.loads(marker.read_text())
    if data.get('kind') != 'aviator-evaluate' or data.get('format') != 1:
        raise ValueError(f'不是本工具的安装目录: {directory}')
    if set(data.get('files', {})) != FILES - {'manifest.json'}:
        raise ValueError('Invalid evaluator file manifest')
    for name, checksum in data['files'].items():
        path = directory / name
        if path.is_symlink() or not path.is_file() or digest(path) != checksum:
            raise ValueError(f'评估包文件校验失败: {name}')
    extra = sorted(p.name for p in directory.iterdir() if p.name not in FILES)
    if extra and not allow_extra:
        raise ValueError(f'评估安装目录含额外文件；拒绝覆盖: {directory}: {extra!r}')
    return data


def retire_backup(backup):
    """Only discard a pristine package; keep the entire tree if anything was added."""
    verified(backup, allow_extra=True)
    extra = sorted(p.name for p in backup.iterdir() if p.name not in FILES)
    if not extra:
        shutil.rmtree(backup)
        return
    # Reserve a private unique parent so no earlier archive can be overwritten.
    # Rename the tree without traversing added files or following their symlinks.
    saved = Path(tempfile.mkdtemp(prefix='.aviator-evaluate.saved-', dir=backup.parent))
    destination = saved / 'aviator-evaluate'
    try:
        backup.rename(destination)
    except BaseException:
        saved.rmdir()
        raise
    print(f'旧安装含额外文件 {extra!r}，已完整保留: {destination}', flush=True)


def unpack(archive, destination):
    with tarfile.open(archive, 'r:gz') as tar:
        seen = set()
        total = 0
        for member in tar:
            parts = member.name.split('/')
            if (len(parts) != 2 or parts[0] != 'aviator-evaluate' or parts[1] not in FILES
                    or parts[1] in seen or not member.isfile()):
                raise ValueError('Unexpected archive member: ' + member.name)
            total += member.size
            if member.size < 0 or total > 16 * 1024 * 1024:
                raise ValueError('Evaluation archive exceeds size limit')
            seen.add(parts[1])
            with tar.extractfile(member) as source, (destination / parts[1]).open('xb') as output:
                shutil.copyfileobj(source, output)
            (destination / parts[1]).chmod(0o755 if parts[1].endswith('.py') else 0o644)
        if seen != FILES:
            raise ValueError('Incomplete evaluation archive')
    verified(destination)


def install(archive, checksum, prefix):
    if digest(archive) != checksum:
        raise ValueError('Archive SHA256 mismatch')
    prefix = prefix.expanduser().absolute()
    if prefix.name != 'aviator-evaluate' or prefix.is_symlink():
        raise ValueError('安装目录末级必须为 aviator-evaluate，且不能为符号链接')
    if prefix.resolve() == Path('/opt/aviator') or Path('/opt/aviator') in prefix.resolve().parents:
        raise ValueError('评估工具不能安装到 AVIATOR 主程序目录内')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    backup = prefix.parent / '.aviator-evaluate.backup'
    lock_path = prefix.parent / '.aviator-evaluate.lock'
    # O_NOFOLLOW avoids following a pre-existing symlink for installation state.
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if backup.exists() or backup.is_symlink():
            verified(backup, allow_extra=True)
            if not prefix.exists():
                backup.rename(prefix)
            else:
                verified(prefix, allow_extra=True)
                retire_backup(backup)
        if prefix.exists():
            verified(prefix, allow_extra=True)
        with tempfile.TemporaryDirectory(prefix='.evaluate-stage-', dir=prefix.parent) as temporary:
            stage = Path(temporary) / 'aviator-evaluate'
            stage.mkdir()
            unpack(archive, stage)
            py_compile.compile(str(stage / 'evaluate_aviator.py'),
                               cfile=str(Path(temporary) / 'check.pyc'), doraise=True)
            subprocess.run([sys.executable, '-B', stage / 'evaluate_aviator.py', '--help'],
                           check=True, stdout=subprocess.DEVNULL)
            old = prefix.exists()
            if old:
                prefix.rename(backup)
            try:
                stage.rename(prefix)
            except BaseException:
                if old and not prefix.exists():
                    backup.rename(prefix)
                raise
            if old:
                retire_backup(backup)
    command = shlex.join(['python3', str(prefix / 'evaluate_aviator.py'), '--open'])
    print(f'独立评估工具已安装: {prefix}\n运行: {command}')
    print('未安装系统 profiler，未修改或启动 AVIATOR；结果默认写入用户 state 目录。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--sha256', required=True)
    parser.add_argument('--prefix', type=Path, default=Path.home() / '.local/share/aviator-evaluate')
    args = parser.parse_args()
    install(args.archive, args.sha256, args.prefix)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError, py_compile.PyCompileError) as error:
        sys.exit(f'安装失败: {error}')
