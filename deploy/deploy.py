#!/usr/bin/env python3
"""Build a source-free C++ runtime and deploy it over an authenticated SSH session."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(args, **kwargs):
    print('+ ' + shlex.join(map(str, args)), flush=True)
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def output(args):
    return subprocess.check_output(list(map(str, args)), text=True).strip()


def load_release():
    spec = importlib.util.spec_from_file_location('aviator_release', ROOT / 'scripts/aviator_release.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def elf_files(root):
    for directory in ('bin', 'lib'):
        for path in (root / directory).rglob('*'):
            if path.is_file() and not path.is_symlink():
                with path.open('rb') as stream:
                    if stream.read(4) == b'\x7fELF':
                        yield path


def package_owner(library):
    candidates = {str(library), str(library.resolve())}
    for name in list(candidates):
        if name.startswith('/usr/lib/'):
            candidates.add(name[4:])
        elif name.startswith('/lib/'):
            candidates.add('/usr' + name)
    for name in sorted(candidates):
        query = subprocess.run(['dpkg-query', '-S', name], text=True, capture_output=True)
        for line in query.stdout.splitlines():
            if ': /' in line:
                package = line.split(': /', 1)[0]
                if re.fullmatch(r'[a-z0-9][a-z0-9+.-]*(?::amd64)?', package):
                    return package
    return None


def patch_tool():
    if shutil.which('patchelf'):
        return shutil.which('patchelf')
    directory = ROOT / 'build/deployment/packaging-tools'
    tool = directory / 'bin/patchelf'
    if not tool.is_file():
        run(['/usr/bin/python3', '-m', 'venv', directory])
        run([directory / 'bin/pip', 'install', 'patchelf==0.17.2.4'])
    return tool


def system_packages(stage):
    packages = {'python3.10-venv', 'python3-tk', 'python3-gi', 'gir1.2-gtk-3.0',
                'gir1.2-ayatanaappindicator3-0.1', 'zenity', 'curl', 'alsa-utils',
                'libgl1', 'libegl1', 'libglfw3', 'libusb-1.0-0', 'libqt5widgets5',
                'libxcb-xinerama0', 'libxcb-cursor0', 'fonts-noto-cjk', 'rsync'}
    libraries = set()
    owners = {}
    for binary in elf_files(stage):
        result = subprocess.run(['ldd', str(binary)], text=True, capture_output=True)
        if 'not found' in result.stdout:
            raise RuntimeError(f'Unresolved library for {binary}:\n{result.stdout}')
        if result.returncode and 'not a dynamic executable' not in result.stderr + result.stdout:
            raise RuntimeError(f'ldd failed: {binary}: {result.stderr}')
        for line in result.stdout.splitlines():
            match = re.search(r'(?:=>\s+)?(/\S+)\s+\(', line)
            if match:
                library = Path(match[1])
                if not library.resolve().is_relative_to(stage.resolve()):
                    libraries.add(library)
    for library in sorted(libraries):
        owner = owners[library] = package_owner(library)
        if owner:
            packages.add(owner)
        else:
            destination = stage / 'lib' / library.name
            if destination.exists() and load_release().digest_file(destination) != load_release().digest_file(library):
                raise RuntimeError(f'Conflicting library basename: {library.name}')
            print(f'Bundling non-system library: {library}', flush=True)
            shutil.copy2(library.resolve(), destination)
    tool = patch_tool()
    for binary in elf_files(stage):
        relative = os.path.relpath(stage / 'lib', binary.parent)
        rpath = '$ORIGIN' if relative == '.' else '$ORIGIN/' + relative
        run([tool, '--set-rpath', rpath, binary], stdout=subprocess.DEVNULL)
    # ldd reports the transitive closure. After relocation every resolved library
    # must be in this package or owned by an Ubuntu package, never the source tree.
    for binary in elf_files(stage):
        result = subprocess.run(['ldd', str(binary)], text=True, capture_output=True)
        if result.returncode or 'not found' in result.stdout:
            raise RuntimeError(f'Relocation failed: {binary}:\n{result.stdout}\n{result.stderr}')
        for line in result.stdout.splitlines():
            match = re.search(r'(?:=>\s+)?(/\S+)\s+\(', line)
            if match:
                path = Path(match[1])
                if not path.resolve().is_relative_to(stage.resolve()) and path not in libraries:
                    raise RuntimeError(f'Unexpected dependency after relocation: {path}')
                if path in libraries and owners[path] is None and not path.resolve().is_relative_to(stage.resolve()):
                    raise RuntimeError(f'Library still resolves outside runtime: {path}')
    return sorted(packages)


def sanitize(stage):
    # CMake's full install includes developer artifacts; deployment contains runtime only.
    for relative in ('include', 'share/doc', 'lib/cmake', 'lib/pkgconfig'):
        path = stage / relative
        if path.exists():
            shutil.rmtree(path)
    forbidden = {'.a', '.h', '.hh', '.hpp', '.hxx', '.c', '.cc', '.cpp', '.cxx', '.cmake', '.o'}
    for path in sorted(stage.rglob('*'), reverse=True):
        if path.is_file() and (path.suffix in forbidden or path.name in {'CMakeLists.txt', 'CMakeCache.txt', 'compile_commands.json'}):
            path.unlink()
        elif path.is_dir() and path.name in {'__pycache__', '.git'}:
            shutil.rmtree(path)
    for binary in elf_files(stage):
        run(['strip', '--strip-unneeded', binary], stdout=subprocess.DEVNULL)


def prepare_package(args):
    build = Path(args.build_dir).resolve()
    build.mkdir(parents=True, exist_ok=True)
    if not args.skip_build:
        run(['cmake', '-S', ROOT, '-B', build, '-DCMAKE_BUILD_TYPE=Release',
             '-DBUILD_TESTING=OFF', '-DAVIATOR_COMMUNICATION_ONLY=OFF',
             '-DAVIATOR_BUILD_EXAMPLES=OFF', '-DAVIATOR_BUILD_RH56FTP_485=ON',
             '-DAVIATOR_BUILD_ROBOTICS_LIBS=ON', '-DAVIATOR_BUILD_XCORE_SDK=ON',
             '-DAVIATOR_BUILD_MUJOCO=ON', '-DAVIATOR_BUILD_SIMULATION_NODE=ON',
             '-DAVIATOR_INSTALL_LAUNCHER=ON'])
        run(['cmake', '--build', build, '--parallel', str(args.jobs)])
    else:
        cache = (build / 'CMakeCache.txt').read_text()
        if not re.search(r'^CMAKE_BUILD_TYPE:\w+=Release$', cache, re.M):
            raise RuntimeError('--skip-build requires an existing Release build')
    artifacts = ROOT / 'build/deployment'
    artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='package-', dir=artifacts) as temporary:
        stage = Path(temporary) / 'aviator'
        run(['cmake', '--install', build, '--prefix', stage, '--strip'])
        sanitize(stage)
        resource = stage / 'share/aviator'
        required = ['bin/xCoreSDK_RH56FTP_485', 'bin/rh56_485_probe', 'bin/aviator_core_managed',
                    'bin/manipulator', 'bin/aviator_grasp_tool_session', 'bin/aviator_grasp_tool_state',
                    'bin/simulation', 'bin/aviator_launcher']
        required += ['share/aviator/scripts/' + p.name for p in (ROOT / 'scripts').glob('*.sh')]
        for name in required:
            if not (stage / name).is_file():
                raise RuntimeError(f'Incomplete runtime install: missing {name}; rebuild without --skip-build')
        for script in resource.rglob('*.sh'):
            run(['bash', '-n', script], stdout=subprocess.DEVNULL)
        packages = system_packages(stage)
        wheel_cache = artifacts / 'wheels'
        wheel_cache.mkdir(exist_ok=True)
        if not args.skip_python_download:
            # Build any source-only Python dependencies here, never on the IPC.
            run(['/usr/bin/python3', '-m', 'pip', 'wheel', '--wheel-dir', wheel_cache,
                 '-r', ROOT / 'deploy/requirements.txt'])
        wheel_files = list(wheel_cache.glob('*.whl'))
        if not wheel_files:
            raise RuntimeError('No Python wheels available; omit --skip-python-download')
        wheelhouse = resource / 'deploy/wheels'
        wheelhouse.mkdir(parents=True, exist_ok=True)
        for wheel in wheel_files:
            shutil.copy2(wheel, wheelhouse / wheel.name)
        # Validate wheel resolution for the target Python without changing this machine.
        with tempfile.TemporaryDirectory(dir=artifacts) as check:
            run(['/usr/bin/python3', '-m', 'venv', check])
            run([Path(check) / 'bin/pip', 'install', '--no-index', '--find-links', wheelhouse,
                 '-r', resource / 'deploy/requirements.txt'], stdout=subprocess.DEVNULL)
        # Ship rules matching the pinned SDK, not potentially stale host rules
        # that lack newer USB product IDs such as D436 (8086:1156).
        camera_rules = ROOT / 'deploy/99-aviator-realsense.rules'
        shutil.copy2(camera_rules, resource / 'deploy/99-aviator-realsense.rules')
        # The existing launch paths require Chrome, not Ubuntu's transitional Chromium snap.
        browser = artifacts / 'google-chrome-stable_current_amd64.deb'
        if not browser.exists():
            run(['curl', '--fail', '--location', '--proto', '=https', '--tlsv1.2',
                 'https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb',
                 '-o', str(browser) + '.part'])
            os.replace(str(browser) + '.part', browser)
        if output(['dpkg-deb', '-f', browser, 'Package']) != 'google-chrome-stable':
            raise RuntimeError('Unexpected Chrome package')
        shutil.copy2(browser, resource / 'deploy' / browser.name)
        manifest = {'format': 1, 'ubuntu': '22.04', 'architecture': 'x86_64', 'python': '3.10',
                    'apt_packages': packages, 'configuration_policy': 'preserve-existing', 'layout': 'fixed-directory'}
        (resource / 'deploy/manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        release = load_release()
        release.seal(stage)
        release.verify(stage)
        archive = artifacts / 'aviator-runtime.tar.gz'
        with tarfile.open(str(archive) + '.part', 'w:gz') as tar:
            tar.add(stage, arcname='aviator')
        os.replace(str(archive) + '.part', archive)
        digest = release.digest_file(archive)
        archive.with_suffix('.gz.sha256').write_text(f'{digest}  {archive.name}\n')
        print(f'Runtime package: {archive} ({archive.stat().st_size // 1024 // 1024} MiB)', flush=True)
        return archive, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='rocos@192.168.11.100', help='SSH user@host')
    parser.add_argument('--build-dir', default=str(ROOT / 'build'))
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--package-only', action='store_true', help='Build/package locally, no SSH')
    parser.add_argument('--skip-build', action='store_true', help='Package already-built Release targets')
    parser.add_argument('--skip-python-download', action='store_true', help='Use cached wheels')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z_][a-z0-9_-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*', args.target):
        parser.error('--target must be user@hostname or user@IPv4')
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    if not (ROOT / 'CMakeLists.txt').is_file():
        parser.error('Run this script from the development checkout, not the installed runtime')
    if output(['uname', '-m']) != 'x86_64' or 'VERSION_ID="22.04"' not in Path('/etc/os-release').read_text():
        parser.error('This deployment profile requires Ubuntu 22.04 x86_64 on the development machine')
    lock_path = ROOT / 'build/deployment.lock'
    lock_path.parent.mkdir(exist_ok=True)
    with lock_path.open('w') as lock, tempfile.TemporaryDirectory(prefix='aviator-ssh-') as control:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ssh = ['ssh', '-o', 'ConnectTimeout=10', '-o', 'ControlMaster=auto', '-o',
               'ControlPersist=600', '-o', 'ControlPath=' + control + '/socket']
        remote = None
        try:
            if not args.package_only:
                run([*ssh, args.target, "test \"$(uname -m)\" = x86_64 && . /etc/os-release && "
                     "test \"$ID:$VERSION_ID\" = ubuntu:22.04 && command -v python3 && id"])
            archive, digest = prepare_package(args)
            if args.package_only:
                return
            remote = output([*ssh, args.target, 'mktemp -d /tmp/aviator-upload.XXXXXXXX'])
            if not re.fullmatch(r'/tmp/aviator-upload\.[A-Za-z0-9]+', remote):
                raise RuntimeError('Unexpected remote upload directory')
            scp = ['scp', *ssh[1:]]
            run([*scp, archive, ROOT / 'deploy/remote.py', f'{args.target}:{remote}/'])
            command = shlex.join(['sudo', 'python3', remote + '/remote.py', '--archive',
                                  remote + '/' + archive.name, '--sha256', digest,
                                  '--user', args.target.split('@')[0]])
            run([*ssh, '-t', args.target, command])
            run([*ssh, args.target, shlex.join(['rm', '-rf', '--', remote])])
            remote = None
        finally:
            if remote:
                print(f'Upload retained for diagnosis: {args.target}:{remote}', file=sys.stderr)
            if not args.package_only:
                subprocess.run([*ssh, '-O', 'exit', args.target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        sys.exit(f'Deployment failed: {error}')
