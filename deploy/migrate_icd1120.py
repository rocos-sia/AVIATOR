#!/usr/bin/env python3
"""Migrate preserved field settings to ICD 1120, inside the install transaction."""
import copy
import math
from pathlib import Path
import sys
import yaml


def migrate(data):
    if not isinstance(data, dict):
        return data
    result = copy.deepcopy(data)
    keyboard = result.get('keyboard')
    old_keys = {'roll_speed', 'pitch_speed', 'roll_limit', 'pitch_limit'}
    new_keys = {'roll_speed_deg_s', 'pitch_speed_mm_s', 'roll_limit_deg', 'pitch_min_mm', 'pitch_max_mm'}
    if isinstance(keyboard, dict) and old_keys.intersection(keyboard):
        if new_keys.intersection(keyboard) or set(keyboard) - old_keys:
            raise ValueError('Mixed/unknown keyboard units; resolve configuration before installing')
        old = dict(roll_speed=1., pitch_speed=1., roll_limit=1., pitch_limit=1.)
        old.update(keyboard)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0 for v in old.values()):
            raise ValueError('Invalid legacy keyboard configuration')
        if old['roll_limit'] > 1 or old['pitch_limit'] > 1:
            raise ValueError('Invalid legacy keyboard limits')
        result['keyboard'] = dict(roll_speed_deg_s=old['roll_speed'] * 50,
                                  pitch_speed_mm_s=old['pitch_speed'] * 75,
                                  roll_limit_deg=old['roll_limit'] * 50,
                                  pitch_min_mm=-old['pitch_limit'] * 80,
                                  pitch_max_mm=old['pitch_limit'] * 70)
    rs = result.get('rs422')
    if isinstance(rs, dict) and rs.get('profile') == 'aviator-bench-v1':
        rs['profile'] = 'aviator-icd-1120'
        step = rs.pop('switch_max_step', 100)
        if not isinstance(step, (int, float)) or not math.isfinite(step) or not 0 <= step <= 2000:
            raise ValueError('Invalid legacy RS422 switch threshold')
        rs['switch_max_roll_deg'] = step * 50 / 1000
        rs['switch_max_pitch_mm'] = step * 75 / 1000
    initial = result.get('wheel_initial')
    if isinstance(initial, dict):
        value = initial.get('displacement', 0)
        if value == -0.085:
            initial['displacement'] = -0.080
        elif not isinstance(value, (int, float)) or not math.isfinite(value) or not -0.150 <= value <= 0:
            raise ValueError('wheel_initial displacement outside new [-0.150,0] m range')
    # Legacy monitor calibration used 85 mm as neutral and physical pitch in [0,170].
    calibration = result.get('yoke_calibration')
    if isinstance(calibration, dict) and calibration.get('pitch_zero_mm') == 85:
        calibration['pitch_zero_mm'] = 0
        model = calibration.get('model', {})
        if 'pitch_offset_m' in model and 'pitch_sign' in model:
            model['pitch_offset_m'] += model['pitch_sign'] * .085 + .005
    return result


def main(root):
    paths = sorted((root / 'share/aviator/config').rglob('*.yaml'))
    extra = root / 'share/aviator/monitor/monitor.yaml'
    if extra.is_file():
        paths.append(extra)
    planned = []
    for path in paths:
        data = yaml.safe_load(path.read_text())
        updated = migrate(data)
        if updated != data:
            planned.append((path, updated))
    # Validate every migration first. Existing installation is untouched until transaction commit.
    for path, updated in planned:
        path.write_text(yaml.safe_dump(updated, allow_unicode=True, sort_keys=False))
        print('ICD 1120 configuration migrated:', path.relative_to(root), flush=True)


if __name__ == '__main__':
    main(Path(sys.argv[1]))
