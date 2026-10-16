"""Reject invalid configuration before SDL/tty startup; Python standard library only."""
from pathlib import Path
import subprocess
import sys
import tempfile

binary, template = map(Path, sys.argv[1:3])
base = template.read_text()
cases = [
    (base.replace('aviator-bench-v1', 'unknown'), 'profile must'),
    (base.replace('REPLACE_CHANNEL_B', 'REPLACE_CHANNEL_A'), 'distinct absolute'),
    (base + '\nunknown_option: true\n', 'unknown or duplicate'),
    (base + '\nprofile: aviator-bench-v1\n', 'unknown or duplicate'),
    (base.replace('roll_limit: 1.0', 'roll_limit: .nan'), 'finite and positive'),
    (base.replace('receive_timeout_ms: 50', 'receive_timeout_ms: 1'), '25..100'),
    (base.replace('pitch_axis: 1', 'pitch_axis: 0'), 'axes must differ'),
    (base.replace('send_a: true', 'send_a: yes'), 'true or false'),
]
with tempfile.TemporaryDirectory() as temp:
    path = Path(temp) / 'config.yaml'
    for config, expected in cases:
        path.write_text(config)
        result = subprocess.run([str(binary), '--config', str(path)], capture_output=True, text=True, timeout=3)
        assert result.returncode != 0 and expected in result.stderr, result.stderr
print('PASS: invalid profile, duplicate/unknown keys, paths, limits, axes, booleans')
