#!/usr/bin/env python3
"""AVIATOR installation identity: canonical SHA256SUMS, verification and bench data."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile

ALGORITHM = "SHA256SUMS-v1"
METADATA = {"SHA256SUMS", "release-checksum.json"}
IDENTITY_FILES = METADATA | {"release-version.json"}
# Only these runtime locations are excluded, never arbitrary config/file extensions.
EXCLUDED_DIRS = {"logs", "cache", "tmp", "run", "test-output", "share/aviator/logs"}
TEST_INPUT = b"AVIATOR-RS422-BENCH-TEST-v1\n"


def version_u16(version):
    if not isinstance(version, str) or not re.fullmatch(r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)", version):
        raise ValueError("version must be canonical MAJOR.MINOR.PATCH without a suffix")
    major, minor, _patch = map(int, version.split("."))
    if major > 255 or minor > 255:
        raise ValueError("version exceeds RS422 8+8-bit capacity")
    # PATCH remains in the full release version; the ICD wire field carries MAJOR.MINOR.
    return (major << 8) | minor


def checksum_u16(digest):
    if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("expected a lowercase SHA-256 digest")
    return int(digest[:4], 16)


def uplink_frame_checksum(prefix):
    """RS422 byte 63: sum bytes 2..62 modulo 256, excluding the EB 90 header."""
    if not isinstance(prefix, bytes) or len(prefix) != 63:
        raise ValueError("RS422 uplink checksum requires exactly bytes 0..62")
    return sum(prefix[2:]) & 0xff


def validate_path(name):
    if not name or name.startswith("/") or re.match(r"^[a-zA-Z]:", name) or any(c in name for c in "\\\r\n\x00") or any(p in ("", ".", "..") for p in name.split("/")):
        raise ValueError(f"invalid release path: {name!r}")
    name.encode("utf-8", errors="strict")
    return name


def excluded(name):
    return "__pycache__" in name.split("/") or any(name == d or name.startswith(d + "/") for d in EXCLUDED_DIRS)


def inventory(root):
    root = Path(root).resolve(strict=True)
    files, names = {}, set()

    def visit(directory):
        for entry in directory.iterdir():
            name = validate_path(entry.relative_to(root).as_posix())
            if excluded(name):
                continue
            folded = name.casefold()
            if folded in names:
                raise ValueError(f"case-insensitive path collision: {name}")
            names.add(folded)
            if entry.is_symlink():
                target = entry.resolve(strict=True)
                if not target.is_relative_to(root) or not target.is_file():
                    raise ValueError(f"symlink must target an internal regular file: {name}")
            elif entry.is_dir():
                visit(entry)
                continue
            elif not entry.is_file():
                raise ValueError(f"not a regular release file: {name}")
            if name in IDENTITY_FILES and entry.is_symlink():
                raise ValueError(f"metadata cannot be a symlink: {name}")
            if name not in METADATA:
                files[name] = entry
    visit(root)
    return dict(sorted(files.items(), key=lambda item: item[0].encode("utf-8")))


def digest_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def manifest(root):
    return "".join(f"{digest_file(path)}  {name}\n" for name, path in inventory(root).items()).encode("utf-8")


def manifest_paths(raw):
    paths = []
    if not raw.endswith(b"\n"):
        raise ValueError("SHA256SUMS must end with LF")
    for line in raw[:-1].decode("utf-8").split("\n"):
        match = re.fullmatch(r"[0-9a-f]{64}  (.+)", line)
        if not match:
            raise ValueError("noncanonical SHA256SUMS")
        name = validate_path(match[1])
        if name in METADATA or excluded(name):
            raise ValueError("excluded file in SHA256SUMS")
        paths.append(name)
    if not paths or paths != sorted(paths, key=lambda p: p.encode("utf-8")) or len({p.casefold() for p in paths}) != len(paths):
        raise ValueError("empty, unsorted or duplicate SHA256SUMS")
    return set(paths)


def write_bytes(path, data):
    if path.is_symlink():
        raise ValueError(f"refusing to overwrite metadata symlink: {path}")
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(data)
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
        except BaseException:
            temporary.unlink()
            raise
    temporary.replace(path)


def write_json(path, value):
    write_bytes(path, (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8"))


def read_version(root, *, allow_legacy=False):
    value = json.loads((root / "release-version.json").read_bytes())
    if not isinstance(value, dict) or value.get("product") != "AVIATOR" or type(value.get("version_u16")) is not int:
        raise ValueError("invalid release version record")
    packed = version_u16(value.get("version"))
    accepted = {packed}
    if allow_legacy:
        # Only publication may replace an installation using the former 4+4+8
        # encoding. Runtime identity and newly generated packages stay strict.
        major, minor, patch = map(int, value["version"].split("."))
        if major <= 15 and minor <= 15 and patch <= 255:
            accepted.add((major << 12) | (minor << 8) | patch)
    if value["version_u16"] not in accepted:
        raise ValueError("invalid release version record")
    if not isinstance(value.get("release_profile"), str) or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]*", value["release_profile"]):
        raise ValueError("invalid release profile")
    if value.get("checksum_scope") != scope():
        raise ValueError("unsupported checksum scope")
    return value


def scope():
    return {"include": ["**"], "exclude_files": sorted(METADATA),
            "exclude_directories": sorted(EXCLUDED_DIRS), "exclude_directory_names": ["__pycache__"]}


def seal(root):
    value = read_version(root)
    raw = manifest(root)
    manifest_paths(raw)
    digest = hashlib.sha256(raw).hexdigest()
    result = {"algorithm": ALGORITHM, "release_digest": digest,
              "checksum_u16": checksum_u16(digest), "version": value["version"],
              "release_profile": value["release_profile"]}
    write_bytes(root / "SHA256SUMS", raw)
    write_json(root / "release-checksum.json", result)
    return verify(root)


def verify(root, expected_digest=None):
    root = Path(root)
    raw = (root / "SHA256SUMS").read_bytes()
    manifest_paths(raw)
    if raw != manifest(root):
        raise ValueError("release files or paths differ from SHA256SUMS")
    digest = hashlib.sha256(raw).hexdigest()
    if expected_digest is not None:
        checksum_u16(expected_digest)
        if digest != expected_digest:
            raise ValueError("release digest differs from trusted delivery record")
    value = read_version(root)
    result = json.loads((root / "release-checksum.json").read_bytes())
    expected = {"algorithm": ALGORITHM, "release_digest": digest,
                "checksum_u16": checksum_u16(digest), "version": value["version"],
                "release_profile": value["release_profile"]}
    if result != expected or type(result.get("checksum_u16")) is not int:
        raise ValueError("release checksum record mismatch")
    return dict(result, version_u16=value["version_u16"], mode="release", verified=True)


def identity(root, allow_test=False, version=None, expected_digest=None):
    root = Path(root)
    if any(os.path.lexists(root / name) for name in IDENTITY_FILES):
        result = verify(root, expected_digest)
    elif allow_test and expected_digest is None:
        # This digest identifies a public test vector, never deployed software.
        test_digest = hashlib.sha256(TEST_INPUT).hexdigest()
        result = {"mode": "bench-test", "verified": False, "version": version,
                  "version_u16": version_u16(version), "release_profile": "bench-test",
                  "checksum_u16": checksum_u16(test_digest), "test_vector_sha256": test_digest}
    else:
        raise ValueError("release metadata missing; bench fallback requires explicit --allow-test-fallback and --version")
    result["rs422_bytes_22_25"] = struct.pack("<HH", result["version_u16"], result["checksum_u16"]).hex(" ")
    return result


def configure(args):
    packed = version_u16(args.version)
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]*", args.profile):
        raise ValueError("invalid release profile")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    value = {"product": "AVIATOR", "version": args.version, "version_u16": packed,
             "release_profile": args.profile, "toolchain": {"cxx": args.compiler, "cmake": args.cmake_version},
             "features": dict(item.split("=", 1) for item in args.feature),
             "dependency_versions": dict(item.split("=", 1) for item in args.dependency), "checksum_scope": scope()}
    write_json(output, value)
    header = output.parent / "aviator/version.hpp"
    header.parent.mkdir(exist_ok=True)
    text = ("#pragma once\n#include <cstdint>\nnamespace aviator::release {\n"
            f'inline constexpr char version[] = "{args.version}";\n'
            f"inline constexpr std::uint16_t version_u16 = {packed};\n"
            f"inline constexpr std::uint16_t bench_checksum_u16 = {checksum_u16(hashlib.sha256(TEST_INPUT).hexdigest())};\n"
            "} // namespace aviator::release\n")
    if not header.exists() or header.read_text() != text:
        write_bytes(header, text.encode())


def generate(args):
    root, source = Path(args.root), Path(args.source)
    inventory(root)  # Validate existing paths before modifying anything.
    value = json.loads(Path(args.build_info).read_bytes())
    def git(*parts):
        return subprocess.check_output(["git", "-C", str(source), *parts], text=True).strip()
    value.update(git_commit=git("rev-parse", "HEAD"),
                 git_dirty=bool(git("status", "--porcelain", "--untracked-files=normal")),
                 build_type=args.config)
    value["vendored_sources"] = sorted(p.name for p in (source / "third_party").iterdir() if p.is_dir())
    # No timestamp or absolute build path: unchanged inputs give identical records.
    write_json(root / "release-version.json", value)
    return seal(root)


def publish(stage, root):
    """Replace only the previous package's recorded files; preserve runtime data."""
    verify(stage)
    current = inventory(root)
    if current or any(os.path.lexists(root / p) for p in METADATA):
        # Require a complete old record, but permit replacing locally modified bytes.
        read_version(root, allow_legacy=True)
        old = manifest_paths((root / "SHA256SUMS").read_bytes())
        json.loads((root / "release-checksum.json").read_bytes())
        if set(current) - old:
            raise ValueError("unmanaged files in installation; choose a clean prefix")
    else:
        old = set()
    new = inventory(stage)
    # Invalidate first, so an interrupted copy cannot retain a valid old identity.
    for name in METADATA:
        (root / name).unlink(missing_ok=True)
    for name in old - new.keys():
        (root / name).unlink(missing_ok=True)
        parent = (root / name).parent
        while parent != root:
            try:
                parent.rmdir()  # Remove only empty former package directories.
            except OSError:
                break
            parent = parent.parent
    for name, source in new.items():
        destination = root / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            destination.unlink()
        if source.is_symlink():
            destination.unlink(missing_ok=True)
            target = root / source.resolve().relative_to(stage.resolve())
            destination.symlink_to(os.path.relpath(target, destination.parent))
        else:
            shutil.copy2(source, destination)
    for name in METADATA:
        write_bytes(root / name, (stage / name).read_bytes())
    return verify(root)


def package(args):
    root = Path(args.prefix).absolute()
    if root.is_symlink():
        raise ValueError("installation root cannot be a symlink")
    root.mkdir(parents=True, exist_ok=True)
    # Directory lock also serializes different build trees sharing one prefix.
    fd = os.open(root, os.O_RDONLY)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with tempfile.TemporaryDirectory(prefix="aviator-release-", dir=args.build) as temporary:
            stage = Path(temporary)
            env = dict(os.environ)
            env.pop("DESTDIR", None)
            subprocess.run([args.cmake, "--install", args.build, "--prefix", str(stage), "--config", args.config],
                           check=True, env=env)
            return publish(stage, root)
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    conf = sub.add_parser("configure")
    for name in ("version", "profile", "output", "compiler", "cmake-version"):
        conf.add_argument("--" + name, required=True)
    conf.add_argument("--feature", action="append", default=[])
    conf.add_argument("--dependency", action="append", default=[])
    gen = sub.add_parser("generate")
    gen.add_argument("root")
    for name in ("source", "build-info", "config"):
        gen.add_argument("--" + name, required=True)
    pkg = sub.add_parser("package")
    for name in ("cmake", "build", "prefix", "config"):
        pkg.add_argument("--" + name, required=True)
    for name in ("verify", "identity"):
        command = sub.add_parser(name)
        command.add_argument("root")
        command.add_argument("--expected-digest")
        if name == "identity":
            command.add_argument("--allow-test-fallback", action="store_true")
            command.add_argument("--version")
    args = parser.parse_args()
    try:
        if args.command == "verify":
            result = verify(args.root, args.expected_digest)
        elif args.command == "identity":
            result = identity(args.root, args.allow_test_fallback, args.version, args.expected_digest)
        else:
            result = {"configure": configure, "generate": generate, "package": package}[args.command](args)
        if result:
            print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    except (ValueError, OSError, RuntimeError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"AVIATOR release error: {error}\n")


if __name__ == "__main__":
    main()
