#!/usr/bin/env python3
"""Independent packaging, safe installation and main-runtime exclusion invariants."""
import hashlib
import importlib.util
import io
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


deploy = module('evaluate_deploy', 'tools/evaluate/deploy.py')
installer = module('evaluate_install', 'tools/evaluate/install.py')
runtime = module('runtime_deploy', 'deploy/deploy.py')


class DeploymentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def package(self):
        return deploy.package(self.root / 'package')

    def test_package_only_has_evaluator_and_no_cmake_or_network(self):
        with mock.patch.object(deploy, 'run', wraps=deploy.run) as run:
            archive, checksum = self.package()
        self.assertEqual(checksum, hashlib.sha256(archive.read_bytes()).hexdigest())
        self.assertEqual(run.call_count, 1)
        self.assertIn('--help', run.call_args.args[0])
        with tarfile.open(archive) as tar:
            self.assertEqual(set(tar.getnames()), {'aviator-evaluate/' + name for name in installer.FILES})
        self.assertNotIn('tools/evaluate', (ROOT / 'cmake/RuntimeTools.cmake').read_text())

    def test_install_upgrade_checksum_and_unknown_directory(self):
        archive, checksum = self.package()
        prefix = self.root / 'installed/aviator-evaluate'
        installer.install(archive, checksum, prefix)
        installer.verified(prefix)
        installer.install(archive, checksum, prefix)
        result = subprocess.run(['python3', prefix / 'evaluate_aviator.py', '--version'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        before = (prefix / 'evaluate_aviator.py').read_bytes()
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            installer.install(archive, '0' * 64, prefix)
        self.assertEqual(before, (prefix / 'evaluate_aviator.py').read_bytes())
        (prefix / 'user-file').write_text('do not erase')
        with self.assertRaisesRegex(ValueError, '额外文件'):
            installer.install(archive, checksum, prefix)
        self.assertEqual((prefix / 'user-file').read_text(), 'do not erase')

    def test_recover_interrupted_rename_and_reject_symlink(self):
        archive, checksum = self.package()
        prefix = self.root / 'installed/aviator-evaluate'
        installer.install(archive, checksum, prefix)
        prefix.rename(prefix.parent / '.aviator-evaluate.backup')
        installer.install(archive, checksum, prefix)
        installer.verified(prefix)
        link = self.root / 'linked/aviator-evaluate'
        link.parent.mkdir()
        link.symlink_to(prefix, target_is_directory=True)
        with self.assertRaises(ValueError):
            installer.install(archive, checksum, link)

    def test_archive_rejects_traversal_symlinks_duplicates_and_extra_files(self):
        for names, kind in [(['aviator-evaluate/../../escape'], tarfile.REGTYPE),
                             (['aviator-evaluate/evaluate_aviator.py'], tarfile.SYMTYPE),
                             (['aviator-evaluate/README.md'] * 2, tarfile.REGTYPE),
                             (['aviator-evaluate/unexpected'], tarfile.REGTYPE)]:
            with self.subTest(names=names, kind=kind), tempfile.TemporaryDirectory(dir=self.root) as temp:
                path = Path(temp) / 'malicious.tar.gz'
                with tarfile.open(path, 'w:gz') as tar:
                    for name in names:
                        info = tarfile.TarInfo(name)
                        info.type = kind
                        if kind == tarfile.REGTYPE:
                            info.size = 1
                            tar.addfile(info, io.BytesIO(b'x'))
                        else:
                            info.linkname = '/etc/passwd'
                            tar.addfile(info)
                destination = Path(temp) / 'out'
                destination.mkdir()
                with self.assertRaises(ValueError):
                    installer.unpack(path, destination)
        self.assertFalse((self.root / 'escape').exists())

    def test_main_deploy_removes_evaluator_even_with_stale_cmake_install(self):
        stage = self.root / 'runtime'
        scripts = stage / 'share/aviator/scripts'
        scripts.mkdir(parents=True)
        for name in ['evaluate_aviator.py', 'deploy_evaluate.sh', 'start_aviator.sh']:
            (scripts / name).write_text('placeholder')
        runtime.sanitize(stage)
        self.assertEqual({p.name for p in scripts.iterdir()}, {'start_aviator.sh'})
        cmake = (ROOT / 'cmake/RuntimeTools.cmake').read_text()
        for name in runtime.EVALUATION_SCRIPTS:
            self.assertIn(f'PATTERN "{name}" EXCLUDE', cmake)

    @unittest.skipUnless(shutil.which('cmake'), 'CMake required for generated install rule test')
    def test_actual_cmake_install_excludes_evaluator(self):
        source = self.root / 'cmake-source'
        source.mkdir()
        directories = ['scripts', 'models/tts', 'deploy']
        directories += ['tools/' + name for name in ('grasp_tool_calibration', 'steering_wheel_calibration',
                                                     'wheel_bandwidth', 'wheel_accuracy', 'remote_launcher')]
        for relative in directories:
            (source / relative).mkdir(parents=True)
        for name in ('start_aviator.sh', 'evaluate_aviator.py', 'deploy_evaluate.sh'):
            (source / 'scripts' / name).write_text('# fixture\n')
        (source / 'tools/remote_launcher/connection.json').write_text('{}')
        (source / 'CMakeLists.txt').write_text(
            'cmake_minimum_required(VERSION 3.22)\nproject(InstallTest NONE)\n'
            'set(CMAKE_INSTALL_DATADIR share)\n'
            f'include("{ROOT / "cmake/RuntimeTools.cmake"}")\n')
        build, prefix = self.root / 'cmake-build', self.root / 'cmake-install'
        subprocess.run(['cmake', '-S', source, '-B', build], check=True, capture_output=True)
        subprocess.run(['cmake', '--install', build, '--prefix', prefix], check=True, capture_output=True)
        installed = prefix / 'share/aviator/scripts'
        self.assertEqual({p.name for p in installed.iterdir()}, {'start_aviator.sh'})


if __name__ == '__main__':
    unittest.main()
