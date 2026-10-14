#!/usr/bin/env python3
"""Release format, tamper detection, publication and explicit bench fallback."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import struct
import subprocess
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("release", Path(__file__).resolve().parents[1] / "scripts/aviator_release.py")
release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release)


class ReleaseTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "package"
        self.root.mkdir()
        self.make_package(self.root)

    def make_package(self, root):
        (root / "bin").mkdir(exist_ok=True)
        (root / "bin/app").write_bytes(b"abc")
        release.write_json(root / "release-version.json", {
            "product": "AVIATOR", "version": "1.2.15", "version_u16": 0x120f,
            "release_profile": "communication", "checksum_scope": release.scope()})
        release.seal(root)

    def test_version_vectors(self):
        for version, value, raw in [("0.0.0", 0, "0000"), ("1.0.3", 0x1003, "0310"),
                                    ("1.2.15", 0x120f, "0f12"), ("2.0.0", 0x2000, "0020"),
                                    ("15.15.255", 65535, "ffff")]:
            self.assertEqual(release.version_u16(version), value)
            self.assertEqual(struct.pack("<H", value).hex(), raw)
        for invalid in ["16.0.0", "1.16.0", "1.0.256", "01.2.3", "1.02.3", "1.2.003",
                        "1.0", "1.0.0-rc.1", "1.0.0+abc", "-1.0.0", "v1.2.3", None]:
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                release.version_u16(invalid)

    def test_checksum_vectors(self):
        self.assertEqual(release.checksum_u16(hashlib.sha256(b"abc").hexdigest()), 0xba78)
        self.assertEqual(struct.pack("<H", 0xba78), b"\x78\xba")
        self.assertEqual(release.checksum_u16("0" * 64), 0)
        self.assertEqual(release.checksum_u16("f" * 64), 65535)
        for invalid in ["A" * 64, "0" * 63, "g" * 64]:
            with self.assertRaises(ValueError):
                release.checksum_u16(invalid)

    def test_manifest_bytes_and_gnu(self):
        (self.root / "bin/中文 name").write_bytes(b"other\r\n")
        result = release.seal(self.root)
        raw = (self.root / "SHA256SUMS").read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), result["release_digest"])
        self.assertTrue(raw.startswith(hashlib.sha256(b"abc").hexdigest().encode() + b"  bin/app\n"))
        if shutil.which("sha256sum"):
            subprocess.run(["sha256sum", "--check", "--strict", "SHA256SUMS"], cwd=self.root,
                           check=True, stdout=subprocess.DEVNULL)

    def test_frame_checksum_is_independent(self):
        prefix = bytes(range(22)) + struct.pack("<HH", 0x0100, 0x9ca1)
        checksum = release.uplink_frame_checksum(prefix)
        self.assertEqual((sum(prefix) + checksum) & 255, 0)
        with self.assertRaises(ValueError):
            release.uplink_frame_checksum(prefix[:-1])

    def test_repeat_and_relocate(self):
        original = release.verify(self.root)
        self.assertEqual(release.seal(self.root), original)
        second = self.root.parent / "second"
        shutil.copytree(self.root, second)
        self.assertEqual(release.verify(second), original)

    def test_changes_fail(self):
        for change in ("modify", "delete", "rename", "add"):
            with self.subTest(change=change):
                root = self.root.parent / change
                shutil.copytree(self.root, root)
                app = root / "bin/app"
                if change == "modify":
                    app.write_bytes(b"abcd")
                elif change == "delete":
                    app.unlink()
                elif change == "rename":
                    app.rename(root / "bin/new")
                else:
                    (root / "bin/extra").write_bytes(b"x")
                with self.assertRaises(ValueError):
                    release.verify(root)
                with self.assertRaises(ValueError):
                    release.identity(root, True, "1.2.15")

    def test_trusted_digest_and_record(self):
        digest = release.verify(self.root)["release_digest"]
        self.assertTrue(release.verify(self.root, digest)["verified"])
        with self.assertRaises(ValueError):
            release.verify(self.root, "0" * 64)
        record = json.loads((self.root / "release-checksum.json").read_bytes())
        record["checksum_u16"] ^= 1
        release.write_json(self.root / "release-checksum.json", record)
        with self.assertRaises(ValueError):
            release.verify(self.root)

    def test_noncanonical_manifest(self):
        raw = (self.root / "SHA256SUMS").read_bytes()
        for bad in [b"\xef\xbb\xbf" + raw, raw.replace(b"\n", b"\r\n"), raw[:-1],
                    b"".join(reversed(raw.splitlines(keepends=True))), raw + raw]:
            (self.root / "SHA256SUMS").write_bytes(bad)
            with self.assertRaises(ValueError):
                release.verify(self.root)

    def test_excluded_runtime_data(self):
        original = release.verify(self.root)
        for path in ["logs/a", "cache/a", "tmp/a", "run/a", "test-output/a", "share/aviator/logs/a", "bin/__pycache__/a.pyc"]:
            entry = self.root / path
            entry.parent.mkdir(parents=True, exist_ok=True)
            entry.write_bytes(b"runtime")
        self.assertEqual(release.verify(self.root), original)
        (self.root / "config").mkdir()
        (self.root / "config/new.yaml").write_bytes(b"control: changed")
        with self.assertRaises(ValueError):
            release.verify(self.root)

    def test_paths_and_case_collision(self):
        for invalid in ["/abs", "../escape", "x/../y", "x\\y", "x\ny", "x\ry", "x//y"]:
            with self.assertRaises(ValueError):
                release.validate_path(invalid)
        (self.root / "bin/APP").write_bytes(b"other")
        with self.assertRaises(ValueError):
            release.seal(self.root)

    def test_symlinks(self):
        link = self.root / "bin/link"
        link.symlink_to("app")
        release.seal(self.root)
        self.assertTrue(release.verify(self.root)["verified"])
        link.unlink()
        link.symlink_to("/etc/hosts")
        with self.assertRaises(ValueError):
            release.verify(self.root)
        link.unlink()
        link.symlink_to(".", target_is_directory=True)
        with self.assertRaises(ValueError):
            release.seal(self.root)

    def test_metadata_symlink_rejected(self):
        record = self.root / "release-checksum.json"
        data = record.read_bytes()
        record.unlink()
        (self.root / "record").write_bytes(data)
        record.symlink_to("record")
        with self.assertRaises(ValueError):
            release.seal(self.root)

    def test_explicit_fallback(self):
        missing = self.root.parent / "missing"
        with self.assertRaises(ValueError):
            release.identity(missing)
        fallback = release.identity(missing, True, "0.1.0")
        self.assertFalse(fallback["verified"])
        self.assertEqual(fallback["mode"], "bench-test")
        self.assertNotIn("release_digest", fallback)
        packed = bytes.fromhex(fallback["rs422_bytes_22_25"])
        self.assertEqual(struct.unpack("<HH", packed), (0x0100, release.checksum_u16(hashlib.sha256(release.TEST_INPUT).hexdigest())))
        with self.assertRaises(ValueError):
            release.identity(missing, True, "0.1.0", "0" * 64)
        self.assertEqual(release.identity(self.root, True, "0.1.0")["mode"], "release")

    def test_partial_metadata_never_falls_back(self):
        for filename in release.IDENTITY_FILES:
            partial = self.root.parent / filename
            partial.mkdir()
            (partial / filename).write_text("{}")
            with self.assertRaises((ValueError, OSError)):
                release.identity(partial, True, "0.1.0")

    def test_publish_removes_only_owned_files(self):
        destination = self.root.parent / "installed"
        destination.mkdir()
        self.make_package(destination)
        (destination / "logs").mkdir()
        (destination / "logs/live").write_text("preserve")
        (self.root / "bin/app").rename(self.root / "bin/new")
        release.seal(self.root)
        release.publish(self.root, destination)
        self.assertFalse((destination / "bin/app").exists())
        self.assertEqual((destination / "logs/live").read_text(), "preserve")
        self.assertTrue(release.verify(destination)["verified"])
        (destination / "unknown").write_text("preserve")
        with self.assertRaises(ValueError):
            release.publish(self.root, destination)
        self.assertEqual((destination / "unknown").read_text(), "preserve")

    def test_installed_launcher_paths(self):
        repo = Path(__file__).resolve().parents[1]
        runtime = self.root / "share/aviator"
        (runtime / "scripts").mkdir(parents=True)
        (runtime / "rh56ftp_hand").mkdir()
        for name in ("start_aviator.sh", "start_camera_servo.sh"):
            launcher = runtime / "scripts" / name
            shutil.copy2(repo / "scripts" / name, launcher)
            for override in (None, str(self.root / "custom-bin")):
                with self.subTest(launcher=name, override=override):
                    env = dict(os.environ)
                    env.pop("AVIATOR_BIN", None)
                    if override:
                        env["AVIATOR_BIN"] = override
                    output = subprocess.check_output(["bash", str(launcher), "--dry-run", "--fake-hand"],
                                                     env=env, text=True)
                    bus = next(line for line in output.splitlines() if line.startswith("启动 bus:"))
                    command = shlex.split(bus.split(":", 1)[1])
                    self.assertEqual(Path(command[0]).resolve(), Path(override or self.root / "bin") / "aviator_bus")
                    self.assertIn("--fake-hand", output)
                    self.assertNotIn("fake_rh56ftp_hand.py", output)
                    self.assertNotIn("nodes/rh56ftp_hand/", output)
                    self.assertIn("nodes/camera/main.py", output)
                    if name == "start_aviator.sh":
                        tts_output = subprocess.check_output(
                            ["bash", str(launcher), "--dry-run", "--simulation", "--tts"], env=env, text=True)
                        line = next(line for line in tts_output.splitlines() if line.startswith("启动 tts:"))
                        tts = shlex.split(line.split(":", 1)[1])
                        self.assertEqual(Path(tts[0]).resolve(), Path(override or self.root / "bin") / "aviator_tts")
                        self.assertIn("config/system-simulation.yaml", tts)
                        self.assertLess(tts_output.index("启动 tts:"), tts_output.index("启动 core:"))


if __name__ == "__main__":
    unittest.main()
