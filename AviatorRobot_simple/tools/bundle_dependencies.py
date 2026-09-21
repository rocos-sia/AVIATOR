#!/usr/bin/env python3
"""Recreate the offline Ubuntu 22.04 x86_64 dependency archive from this workspace.
Normal builds use the committed archive and never run this maintainer utility.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

project = Path(__file__).resolve().parents[1]
repo = project.parent
out = project / 'third_party'
with tempfile.TemporaryDirectory(prefix='aviator-deps-') as tmp:
    root = Path(tmp)
    inc, lib, notices = (root / n for n in ('include', 'lib', 'licenses'))
    for p in (inc, lib, notices):
        p.mkdir()
    packages = set()
    def package_for(path):
        r = subprocess.run(['dpkg-query', '-S', str(path)], text=True, capture_output=True)
        if r.returncode == 0:
            packages.update(r.stdout.split(': /')[0].splitlines()[0].split(', '))
    def copy(src, dst):
        src = Path(src)
        if src.is_dir():
            shutil.copytree(src, dst, dirs_exist_ok=True)
        else:
            shutil.copy2(src, dst)
        package_for(src)
    for name in ['boost', 'eigen3', 'kdl', 'yaml-cpp', 'urdf_model', 'urdf_model_state',
                 'urdf_sensor', 'urdf_world', 'urdf_parser', 'urdf_exception',
                 'console_bridge', 'octomap', 'GLFW', 'GL', 'KHR', 'tinyxml.h', 'tinyxml2.h']:
        copy(Path('/usr/include') / name, inc / name)
    # Keep NLopt C and C++ headers paired with their library.
    for name in ['nlopt.h', 'nlopt.hpp']:
        copy(Path('/opt/rocos/app/include') / name, inc / name)
    prefix = repo / 'AviatorRobot/third_party/_install'
    copy(prefix / 'include', inc)
    copy(repo / 'reference/rocos-mujoco/third_party/mujoco-3.4.0/include/mujoco', inc / 'mujoco')
    copy(repo / 'xCoreSDK-CPP-main/include/rokae', inc / 'rokae')
    seeds = list((prefix / 'lib').glob('*.so*'))
    seeds += list((repo / 'reference/rocos-mujoco/third_party/mujoco-3.4.0/lib').glob('libmujoco.so*'))
    for name in ['yaml-cpp', 'orocos-kdl', 'tinyxml', 'tinyxml2', 'urdfdom_model',
                 'urdfdom_model_state', 'urdfdom_sensor', 'urdfdom_world', 'console_bridge',
                 'boost_filesystem', 'boost_system', 'boost_thread', 'boost_date_time',
                 'boost_serialization', 'octomap', 'glfw']:
        seeds.append(Path('/usr/lib/x86_64-linux-gnu') / f'lib{name}.so')
    seeds.append(Path('/opt/rocos/app/lib/libnlopt.so'))
    platform = re.compile(r'lib(c|m|pthread|rt|dl|stdc\+\+|gcc_s)\.so|ld-linux')
    seen = set()
    def library(path):
        path = Path(path)
        real = path.resolve()
        fresh = real not in seen
        if fresh:
            seen.add(real)
            copy(real, lib / real.name)
        info = subprocess.check_output(['readelf', '-d', str(real)], text=True)
        soname = re.search(r'\(SONAME\).*\[(.*?)\]', info)
        aliases = [path.name] + ([soname[1]] if soname else [])
        for alias in aliases:
            if alias != real.name and not (lib / alias).exists():
                (lib / alias).symlink_to(real.name)
        if not fresh:
            return
        # The executable's relative DT_RPATH covers this entire closure at runtime.
        deps = subprocess.check_output(['ldd', str(real)], text=True)
        for name, target in re.findall(r'^\s*(\S+) => (\S+) \(', deps, re.M):
            if not platform.match(name):
                library(target)
    for seed in seeds:
        library(seed)
    sdk = repo / 'xCoreSDK-0.7.1-linux-x86_64/lib/Linux/x86_64'
    for name in ['libxCoreSDK.a', 'libxMateModel.a']:
        copy(sdk / name, lib / name)
    for name in ('pinocchio', 'hpp-fcl'):
        copy(repo / f'AviatorRobot/third_party/{name}/LICENSE', notices / f'{name}.LICENSE')
    copy(repo / 'reference/rocos-mujoco/third_party/mujoco-3.4.0/THIRD_PARTY_NOTICES', notices / 'mujoco.THIRD_PARTY_NOTICES')
    copy(repo / 'xCoreSDK-CPP-main/README.md', notices / 'xCoreSDK.README.md')
    copy(Path('/usr/share/doc/libnlopt-dev/copyright'), notices / 'nlopt.COPYING')
    versions = {}
    for package in sorted(packages):
        versions[package] = subprocess.check_output(['dpkg-query', '-W', '-f=${Version}', package], text=True)
        copyright_file = Path('/usr/share/doc') / package.split(':')[0] / 'copyright'
        if copyright_file.exists():
            shutil.copy2(copyright_file, notices / (package.replace(':', '_') + '.copyright'))
    manifest = {'platform': 'Ubuntu 22.04 x86_64, GCC 11, glibc >= 2.35',
                'local': {'pinocchio': '3.9.0', 'coal': '3.0.4', 'mujoco': '3.4.0', 'xCoreSDK': '0.7.1', 'nlopt': '2.7.0'},
                'packages': versions,
                'files': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(root.rglob('*')) if p.is_file() and not p.is_symlink()}}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    archive = out / 'linux-x86_64.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for p in sorted(root.iterdir()):
            tar.add(p, arcname=p.name)
    (out / 'linux-x86_64.sha256').write_text(hashlib.sha256(archive.read_bytes()).hexdigest() + '\n')
    print(f'{archive}: {archive.stat().st_size / 1024**2:.1f} MiB')
for name in ('trac_ik', 'kdl_parser'):
    src = repo / 'AviatorRobot/third_party/rocos_app/3rdparty' / name
    for part in ('src', 'include'):
        shutil.copytree(src / part, out / name / part, dirs_exist_ok=True)
