#!/usr/bin/env python3
"""Apply an offline search result; regenerate the model and rebuild afterwards."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('selection', type=Path)
args = parser.parse_args()
config = Path(__file__).resolve().parents[1] / 'config'
selection = json.loads(args.selection.read_text())
grasp = json.loads((config / 'grasp.json').read_text())
posture = json.loads((config / 'posture.json').read_text())
for side in ['left', 'right']:
    chosen = selection[side]
    grasp[side] = {key: chosen[key] for key in ['position', 'quaternion']}
    posture[f'{side}_home_deg'] = chosen['home_deg']
    posture[f'{side}_approach_seed_deg'] = chosen['approach_seed_deg']
(config / 'grasp.json').write_text(json.dumps(grasp, indent=2) + '\n')
(config / 'posture.json').write_text(json.dumps(posture, indent=2) + '\n')
print('Updated grasp.json and posture.json; regenerate aviator.xml and rebuild both projects.')
