#!/usr/bin/env python3
"""Deployment invariants: archive safety, config preservation, fixed paths and recovery."""
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


remote = module('deploy_remote', ROOT / 'deploy/remote.py')
release = module('deploy_release', ROOT / 'scripts/aviator_release.py')


class DeploymentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.releases = self.root / 'releases'
        self.releases.mkdir()
        self.prefix = self.root / 'aviator'
        for key, value in [('PREFIX', self.prefix), ('RELEASES', self.releases), ('DATA', self.root / 'data'), ('WORK', self.root / '.aviator-install')]:
            patch = mock.patch.object(remote, key, value)
            patch.start()
            self.addCleanup(patch.stop)

    def archive(self, entries):
        path = self.root / 'test.tar.gz'
        with tarfile.open(path, 'w:gz') as tar:
            for name, kind, value in entries:
                entry = tarfile.TarInfo(name)
                entry.type = kind
                if kind == tarfile.REGTYPE:
                    entry.size = len(value)
                    tar.addfile(entry, io.BytesIO(value))
                else:
                    entry.linkname = value
                    tar.addfile(entry)
        return path

    def test_archive_traversal_and_link_parent_rejected(self):
        for entries in [
            [('aviator/../../escape', tarfile.REGTYPE, b'bad')],
            [('/etc/escape', tarfile.REGTYPE, b'bad')],
            [('aviator/link', tarfile.SYMTYPE, '/etc')],
            [('aviator/link', tarfile.SYMTYPE, 'inside'), ('aviator/link/file', tarfile.REGTYPE, b'bad')],
        ]:
            with self.subTest(entries=entries), self.assertRaises(ValueError):
                remote.safe_extract(self.archive(entries), self.root / 'unpacked')
        self.assertFalse((self.root.parent / 'escape').exists())

    def test_internal_library_symlink_preserved(self):
        archive = self.archive([('aviator/lib/libx.so.1', tarfile.REGTYPE, b'library'),
                                ('aviator/lib/libx.so', tarfile.SYMTYPE, 'libx.so.1')])
        remote.safe_extract(archive, self.root / 'unpacked')
        self.assertEqual((self.root / 'unpacked/aviator/lib/libx.so').read_bytes(), b'library')

    def test_old_and_new_manifests_request_concrete_python_venv(self):
        resource = self.root / 'share/aviator/deploy'
        resource.mkdir(parents=True)
        for package in ('python3-venv', 'python3-venv:amd64', 'python3.10-venv'):
            with self.subTest(package=package):
                (resource / 'manifest.json').write_text(json.dumps({
                    'ubuntu': '22.04', 'architecture': 'x86_64',
                    'apt_packages': [package, 'python3-tk']}))
                # Stop before environment setup or any system mutation.
                with mock.patch.object(remote, 'run') as run, \
                        mock.patch.object(remote.shutil, 'which', return_value='/usr/bin/google-chrome-stable'), \
                        mock.patch.object(remote, 'prepare_python', side_effect=RuntimeError('stop after apt')):
                    with self.assertRaisesRegex(RuntimeError, 'stop after apt'):
                        remote.setup_dependencies(self.root, SimpleNamespace())
                run.assert_any_call(['apt-get', 'install', '-y', '--no-remove',
                                     '--no-install-recommends', 'python3-tk', 'python3.10-venv'])

    def test_venv_uses_python_310_and_offline_wheels(self):
        with mock.patch.object(remote.os, 'chown'), mock.patch.object(remote, 'user_run') as run:
            account = SimpleNamespace(pw_uid=1000, pw_gid=1000)
            remote.prepare_python(self.root, account)
        self.assertEqual(run.call_args_list[0].args[1],
                         ['/usr/bin/python3.10', '-m', 'venv', self.root / 'cache/python'])
        self.assertIn('--no-index', run.call_args_list[1].args[1])

    def make_release(self, name, config):
        path = self.releases / name
        scripts = path / 'share/aviator/scripts'
        scripts.mkdir(parents=True)
        shutil.copy2(ROOT / 'scripts/aviator_release.py', scripts)
        directory = path / 'share/aviator/config'
        directory.mkdir()
        (directory / 'system.yaml').write_text(config)
        (path / 'deployment.json').write_text(json.dumps({'format': 1, 'user': 'operator', 'previous': None}))
        (path / 'release-version.json').write_text(json.dumps({
            'product': 'AVIATOR', 'version': '1.0.0', 'version_u16': 256,
            'release_profile': 'hardware', 'checksum_scope': release.scope()}))
        release.seal(path)
        return path

    def transaction(self):
        remote.WORK.mkdir()
        record = {'format': 1, 'id': 'a' * 32, 'phase': 'preparing'}
        remote.write_transaction(record)
        new = self.make_release('incoming', 'new-config')
        (new / 'deployment.json').write_text(json.dumps({
            'format': 2, 'layout': 'fixed-directory', 'user': 'operator',
            'transaction': record['id']}))
        release.seal(new)
        stage = remote.WORK / 'aviator'
        new.rename(stage)
        return stage, record

    def make_current(self, legacy=False):
        old = self.make_release('old', 'field-config')
        (old / 'obsolete-file').write_text('remove on update')
        if legacy:
            self.prefix.symlink_to(old)
        else:
            old.rename(self.prefix)
        return old if legacy else self.prefix

    def replace(self, stage, record, failure=None):
        with mock.patch.object(remote, 'prepare_python') as prepare, \
                mock.patch.object(remote, 'check_runtime', side_effect=failure), \
                mock.patch.object(remote, 'desktop_entries'), mock.patch.object(remote, 'ensure_idle'):
            remote.replace_installation(stage, SimpleNamespace(), record)
            prepare.assert_called_once_with(self.prefix, mock.ANY)

    def test_full_replacement_removes_old_files_and_preserves_config(self):
        self.make_current()
        stage, record = self.transaction()
        remote.preserve_configuration(remote.current_release(), stage)
        release.seal(stage)
        self.replace(stage, record)
        remote.recover_transaction()
        self.assertFalse(self.prefix.is_symlink())
        self.assertEqual((self.prefix / 'share/aviator/config/system.yaml').read_text(), 'field-config')
        self.assertFalse((self.prefix / 'obsolete-file').exists())
        self.assertFalse(remote.WORK.exists())
        self.assertTrue(release.verify(self.prefix)['verified'])

    def test_install_preserves_all_config_paths_and_adds_new_files(self):
        old = self.make_current(legacy=True)
        # Field edits deliberately make the old checksum manifest stale.
        edits = {'share/aviator/config/system.yaml': 'field edit after sealing',
                 'share/aviator/config/site/custom.yaml': 'field-only setting',
                 remote.CONFIG_PATHS[1]: 'field monitor',
                 remote.CONFIG_PATHS[2]: 'field serial port'}
        for name, value in edits.items():
            path = old / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value)
        package = self.make_release('package', 'local defaults')
        for name in (*remote.CONFIG_PATHS[1:], 'share/aviator/config/new-feature.yaml'):
            path = package / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('new default')
        release.seal(package)
        archive = self.root / 'runtime.tar.gz'
        with tarfile.open(archive, 'w:gz') as tar:
            tar.add(package, arcname='aviator')
        account = SimpleNamespace(pw_uid=1000)
        with mock.patch.object(remote.pwd, 'getpwnam', return_value=account), \
                mock.patch.object(remote, 'setup_dependencies'), \
                mock.patch.object(remote, 'prepare_python'), \
                mock.patch.object(remote, 'check_runtime') as check, \
                mock.patch.object(remote, 'desktop_entries'), mock.patch.object(remote, 'ensure_idle'):
            remote.install(SimpleNamespace(archive=archive, sha256=remote.digest(archive), user='operator'))
            self.assertEqual(check.call_count, 2)
        for name, value in edits.items():
            self.assertEqual((self.prefix / name).read_text(), value)
        self.assertEqual((self.prefix / 'share/aviator/config/new-feature.yaml').read_text(), 'new default')
        self.assertFalse((self.prefix / 'obsolete-file').exists())
        self.assertFalse(self.prefix.is_symlink())
        self.assertFalse(old.exists())
        self.assertFalse(remote.WORK.exists())
        self.assertTrue(release.verify(self.prefix)['verified'])

    def test_first_install_keeps_packaged_defaults(self):
        stage, _ = self.transaction()
        remote.preserve_configuration(None, stage)
        self.assertEqual((stage / 'share/aviator/config/system.yaml').read_text(), 'new-config')
        self.assertTrue(release.verify(stage)['verified'])

    def test_config_symlinks_rejected_without_modifying_old_installation(self):
        old = self.make_current()
        stage, _ = self.transaction()
        outside = self.root / 'outside.yaml'
        outside.write_text('untouched')
        for root in (old, stage):
            with self.subTest(root=root):
                link = root / 'share/aviator/config/link.yaml'
                link.symlink_to(outside)
                with self.assertRaisesRegex(RuntimeError, 'symlink'):
                    remote.preserve_configuration(old, stage)
                link.unlink()
                self.assertEqual(outside.read_text(), 'untouched')
                self.assertEqual((old / 'share/aviator/config/system.yaml').read_text(), 'field-config')

    def test_config_type_conflict_retains_original(self):
        old = self.make_current()
        stage, _ = self.transaction()
        target = stage / 'share/aviator/config/system.yaml'
        target.unlink()
        target.mkdir()
        with self.assertRaisesRegex(RuntimeError, 'conflict'):
            remote.preserve_configuration(old, stage)
        remote.recover_transaction()
        self.assertEqual((old / 'share/aviator/config/system.yaml').read_text(), 'field-config')

    def test_failed_final_check_restores_old_directory_and_config(self):
        self.make_current()
        stage, record = self.transaction()
        with self.assertRaises(RuntimeError):
            self.replace(stage, record, RuntimeError('Python import failed'))
        remote.recover_transaction()
        self.assertFalse(self.prefix.is_symlink())
        self.assertEqual((self.prefix / 'share/aviator/config/system.yaml').read_text(), 'field-config')
        self.assertTrue((self.prefix / 'obsolete-file').exists())
        self.assertFalse(remote.WORK.exists())

    def test_initial_install_failure_leaves_no_partial_installation(self):
        stage, record = self.transaction()
        with self.assertRaises(RuntimeError):
            self.replace(stage, record, RuntimeError('Missing library'))
        remote.recover_transaction()
        self.assertFalse(self.prefix.exists())
        self.assertFalse(remote.WORK.exists())

    def test_legacy_link_migrated_and_all_managed_versions_removed(self):
        self.make_current(legacy=True)
        self.make_release('older', 'older-config')
        logs = remote.DATA / 'logs'
        logs.mkdir(parents=True)
        (logs / 'recording.mcap').write_text('user recording')
        stage, record = self.transaction()
        self.replace(stage, record)
        remote.recover_transaction()
        self.assertFalse(self.prefix.is_symlink())
        self.assertFalse(self.releases.exists())
        self.assertEqual((logs / 'recording.mcap').read_text(), 'user recording')

    def test_failed_migration_restores_legacy_link_and_keeps_versions(self):
        old = self.make_current(legacy=True)
        stage, record = self.transaction()
        with self.assertRaises(RuntimeError):
            self.replace(stage, record, RuntimeError('final check'))
        remote.recover_transaction()
        self.assertTrue(self.prefix.is_symlink())
        self.assertEqual(self.prefix.resolve(), old)
        self.assertTrue(old.exists())

    def test_interruption_after_moving_old_directory_is_recovered(self):
        self.make_current()
        stage, record = self.transaction()
        record['phase'] = 'replacing'
        remote.write_transaction(record)
        self.prefix.rename(remote.WORK / 'previous')
        remote.recover_transaction()
        self.assertEqual((self.prefix / 'share/aviator/config/system.yaml').read_text(), 'field-config')
        self.assertFalse(remote.WORK.exists())

    def test_preparation_failure_does_not_touch_old_directory(self):
        self.make_current()
        self.transaction()
        remote.recover_transaction()
        self.assertEqual((self.prefix / 'share/aviator/config/system.yaml').read_text(), 'field-config')

    def test_unrelated_files_in_legacy_store_are_retained(self):
        self.make_current(legacy=True)
        (self.releases / 'unrelated').mkdir()
        (self.releases / 'notes.txt').write_text('keep')
        stage, record = self.transaction()
        self.replace(stage, record)
        remote.recover_transaction()
        self.assertEqual((self.releases / 'notes.txt').read_text(), 'keep')
        self.assertTrue((self.releases / 'unrelated').is_dir())
        self.assertFalse((self.releases / 'old').exists())

    def test_unmanaged_directory_never_replaced(self):
        self.prefix.mkdir()
        (self.prefix / 'precious').write_text('keep')
        with self.assertRaises(RuntimeError):
            remote.current_release()
        self.assertEqual((self.prefix / 'precious').read_text(), 'keep')

    def test_interruption_after_new_directory_arrives_restores_old(self):
        self.make_current()
        stage, record = self.transaction()
        record['phase'] = 'replacing'
        remote.write_transaction(record)
        self.prefix.rename(remote.WORK / 'previous')
        stage.rename(self.prefix)
        remote.recover_transaction()
        self.assertEqual((self.prefix / 'share/aviator/config/system.yaml').read_text(), 'field-config')
        self.assertFalse(remote.WORK.exists())

    def test_failed_directory_move_restores_backup(self):
        self.make_current()
        stage, record = self.transaction()
        replace = remote.os.replace
        def fail_new(source, target):
            if source == stage:
                raise OSError('rename failed')
            return replace(source, target)
        with mock.patch.object(remote.os, 'replace', side_effect=fail_new):
            with self.assertRaises(OSError):
                self.replace(stage, record)
        remote.recover_transaction()
        self.assertEqual((self.prefix / 'share/aviator/config/system.yaml').read_text(), 'field-config')

    def test_fixed_directory_is_recognized_for_next_update(self):
        self.make_current()
        self.assertEqual(remote.current_release(), self.prefix)


if __name__ == '__main__':
    unittest.main()
