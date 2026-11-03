"""Opt-in checkpoint gate; historical training runs keep their original path."""
import json
from pathlib import Path
import time


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def evaluate_gate(reports, required=(20000, 40000), expected_episodes=10):
    """A missing/crashed evaluation is an error, never zero completion."""
    for step in required:
        report = reports.get(step)
        if report is None:
            return 'pending'
        if report.get('returncode') != 0 or report.get('instrumentation_unknown', 0):
            return 'instrumentation_error'
        if report.get('summary', {}).get('episodes') != expected_episodes:
            return 'instrumentation_error'
        rate = report.get('summary', {}).get('completion_rate')
        if not isinstance(rate, (int, float)) or not 0. <= rate <= 1.:
            return 'instrumentation_error'
    if all(reports[s]['summary']['completion_rate'] == 0. for s in required):
        return 'stop_both_zero'
    return 'continue'


class TrainingGate:
    def __init__(self, directory, pause_step=40000):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.pause_step = pause_step
        self.waiting = False
        self.stopped = False

    def checkpoint_ready(self, step, path):
        atomic_json(self.directory / f'ready_{step}.json',
                    {'step': step, 'checkpoint': str(path)})

    def stop_requested(self):
        return self.stopped or (self.directory / 'STOP').exists()

    def wait_for_decision(self):
        self.waiting = True
        try:
            while not self.stop_requested():
                decision = self.directory / 'decision.json'
                if decision.exists():
                    value = json.loads(decision.read_text())['decision']
                    if value == 'continue':
                        return True
                    self.stopped = True
                    return False
                time.sleep(.2)
            self.stopped = True
            return False
        finally:
            self.waiting = False
