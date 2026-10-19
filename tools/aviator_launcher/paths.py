"""Resolve only the selected distribution; never search unrelated checkouts."""
from dataclasses import dataclass
from pathlib import Path
import json
import os


@dataclass(frozen=True)
class Layout:
    root: Path
    kind: str
    prefix: Path

    @property
    def script(self):
        return self.root / 'scripts/start_aviator.sh'

    @property
    def bin(self):
        return self.prefix / 'bin' if self.kind == 'installed' else self.root / 'build/bin'

    def select_bin(self, required):
        """Choose one complete build, preferring Release, then recent binaries."""
        if self.kind == 'installed':
            return self.bin
        candidates = [self.root / relative for relative in
                      ('build/release/bin', 'build/debug/bin', 'build/bin')]
        ranked = []
        for directory in candidates:
            if not directory.is_dir():
                continue
            stamps = []
            for name in required:
                file = directory / name
                try:
                    if file.is_file() and os.access(file, os.X_OK):
                        stamps.append(file.stat().st_mtime_ns)
                except OSError:
                    pass  # A concurrent build may replace a binary; preflight checks again.
            ranked.append(((len(stamps) == len(required), directory == candidates[0],
                            max(stamps, default=0)), directory))
        # Incomplete trees remain available for preview and precise preflight errors.
        return max(ranked, key=lambda item: item[0])[1] if ranked else self.bin


def resolve_layout(location=None):
    selected = Path(location).expanduser().resolve() if location else Path(__file__).resolve().parents[2]
    # Exactly two allowed candidates: selected resource root or selected prefix.
    for root in (selected, selected / 'share/aviator'):
        if not (root / 'scripts/start_aviator.sh').is_file() or not (root / 'config').is_dir():
            continue
        if root.name == 'aviator' and root.parent.name == 'share':
            prefix = root.parent.parent
            try:
                meta = json.loads((prefix / 'release-version.json').read_text())
                if not isinstance(meta, dict) or not meta.get('version') or not (prefix / 'bin').is_dir():
                    raise ValueError('发布元数据或 bin 目录不完整')
            except (OSError, ValueError) as exc:
                raise ValueError(f'安装包不完整：{prefix}（{exc}）') from exc
            return Layout(root, 'installed', prefix)
        cmake = root / 'CMakeLists.txt'
        if cmake.is_file() and 'project(AVIATOR' in cmake.read_text():
            return Layout(root, 'source', root)
    raise ValueError(f'未找到同套 AVIATOR 的 scripts/start_aviator.sh 与 config：{selected}')


def user_dir(variable, fallback):
    value = Path(os.environ.get(variable) or Path.home() / fallback).expanduser()
    if not value.is_absolute():
        raise ValueError(f'{variable} 必须为绝对路径')
    return value


def runtime_dir():
    value = os.environ.get('XDG_RUNTIME_DIR', '')
    if not value:
        raise ValueError('缺少 XDG_RUNTIME_DIR，请在普通用户桌面会话中启动')
    base = Path(value)
    st = base.stat()
    if not base.is_absolute() or st.st_uid != os.getuid() or st.st_mode & 0o077:
        raise ValueError('XDG_RUNTIME_DIR 必须由当前用户拥有且权限为 0700')
    target = base / 'aviator-launcher'
    target.mkdir(mode=0o700, exist_ok=True)
    if target.is_symlink() or target.stat().st_uid != os.getuid() or target.stat().st_mode & 0o077:
        raise ValueError('启动器运行目录所有者或权限无效')
    return target
