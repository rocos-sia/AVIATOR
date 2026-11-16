#!/usr/bin/env python3
"""Validate and deploy only the independent AVIATOR evaluator; never invoke CMake."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import py_compile
import re
import shlex
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PAYLOAD = {'evaluate_aviator.py': ROOT / 'scripts/evaluate_aviator.py',
           'README.md': ROOT / 'tools/evaluate/README.md'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command, **kwargs):
    print('+ ' + shlex.join(map(str, command)), flush=True)
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def package(destination):
    destination.mkdir(parents=True, exist_ok=True)
    # Compile into temporary files to catch syntax errors without distributing
    # host-version-specific bytecode or writing __pycache__ into the checkout.
    with tempfile.TemporaryDirectory(prefix='evaluate-build-', dir=destination) as temp:
        temp = Path(temp)
        for path in [PAYLOAD['evaluate_aviator.py'], Path(__file__), ROOT / 'tools/evaluate/install.py']:
            py_compile.compile(str(path), cfile=str(temp / (path.stem + '.pyc')), doraise=True)
        run([sys.executable, '-B', PAYLOAD['evaluate_aviator.py'], '--help'], stdout=subprocess.DEVNULL)
        manifest = {'kind': 'aviator-evaluate', 'format': 1, 'python_min': '3.9',
                    'files': {name: digest(source) for name, source in PAYLOAD.items()}}
        (temp / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        archive = destination / 'aviator-evaluate.tar.gz'
        with tarfile.open(temp / archive.name, 'w:gz') as tar:
            for name, source in {**PAYLOAD, 'manifest.json': temp / 'manifest.json'}.items():
                tar.add(source, arcname='aviator-evaluate/' + name, recursive=False)
        os.replace(temp / archive.name, archive)
        checksum = digest(archive)
        (destination / (archive.name + '.sha256')).write_text(f'{checksum}  {archive.name}\n')
        return archive, checksum


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='rocos@192.168.31.100', help='SSH user@host，沿用主部署默认目标')
    parser.add_argument('--port', type=int, default=22)
    parser.add_argument('--prefix', help='目标绝对目录，末级必须为 aviator-evaluate；默认 ~/.local/share/aviator-evaluate')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'build/evaluate-deployment')
    parser.add_argument('--package-only', action='store_true', help='仅本地语法编译、校验和打包；不联网')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[a-z_][a-z0-9_-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*', args.target):
        parser.error('--target 必须为 user@hostname 或 user@IPv4')
    if not 1 <= args.port <= 65535:
        parser.error('--port 必须在 1..65535')
    if args.prefix and (not Path(args.prefix).is_absolute() or Path(args.prefix).name != 'aviator-evaluate'):
        parser.error('--prefix 必须为绝对路径，末级目录必须为 aviator-evaluate')
    archive, checksum = package(args.output_dir.resolve())
    print(f'独立评估包: {archive}\nSHA256: {checksum}', flush=True)
    if args.package_only:
        return 0
    with tempfile.TemporaryDirectory(prefix='evaluate-ssh-') as control:
        options = ['-o', 'ConnectTimeout=10', '-o', 'ControlMaster=auto',
                   '-o', 'ControlPersist=120', '-o', 'ControlPath=' + control + '/socket']
        ssh = ['ssh', *options, '-p', str(args.port)]
        remote = None
        try:
            run([*ssh, args.target, shlex.join(['python3', '-c',
                'import sys; assert sys.version_info >= (3,9), "Python 3.9+ required"'])])
            remote = run([*ssh, args.target, 'mktemp -d /tmp/aviator-evaluate-upload.XXXXXXXX'],
                         capture_output=True, text=True).stdout.strip()
            if not re.fullmatch(r'/tmp/aviator-evaluate-upload\.[A-Za-z0-9]+', remote):
                raise RuntimeError('Unexpected remote upload directory')
            installer = ROOT / 'tools/evaluate/install.py'
            run(['scp', *options, '-P', str(args.port), archive, installer, f'{args.target}:{remote}/'])
            command = ['python3', remote + '/install.py', '--archive', remote + '/' + archive.name,
                       '--sha256', checksum]
            if args.prefix:
                command += ['--prefix', args.prefix]
            run([*ssh, args.target, shlex.join(command)])
            run([*ssh, args.target, shlex.join(['rm', '-rf', '--', remote])])
            remote = None
        finally:
            if remote:
                print(f'远端临时文件保留供排查: {args.target}:{remote}', file=sys.stderr)
            subprocess.run([*ssh, '-O', 'exit', args.target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError, py_compile.PyCompileError) as error:
        sys.exit(f'评估工具部署失败: {error}')
