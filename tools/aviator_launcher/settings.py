"""User state never modifies installation resources."""
from pathlib import Path
import json
import os
import tempfile
from paths import user_dir
from options import Options


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, temp = tempfile.mkstemp(dir=path.parent, prefix='.pending-')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


class Settings:
    def __init__(self):
        self.path = user_dir('XDG_CONFIG_HOME', '.config') / 'aviator-launcher/settings.json'
        self.data = {'schema_version': 1, 'sources': {}}
        self.warning = ''
        if self.path.exists():
            try:
                data = json.loads(self.path.read_text())
                if data.get('schema_version') != 1 or not isinstance(data.get('sources'), dict):
                    raise ValueError('不支持的配置格式')
                for value in data['sources'].values():
                    Options.from_dict(value)
                self.data = data
            except (ValueError, OSError, AttributeError, TypeError) as exc:
                self.warning = f'配置未加载，使用默认选项：{exc}'

    def get(self, layout):
        return Options.from_dict(self.data['sources'].get(str(layout.root), {}))

    def save(self, layout, options):
        self.data['sources'][str(layout.root)] = options.to_dict()
        atomic_json(self.path, self.data)
