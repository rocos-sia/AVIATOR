"""CLI regression for the offline SML executable; standard library only, no devices."""
import subprocess
import sys

exe = sys.argv[1]


def run(commands, *expected):
    result = subprocess.run([exe], input=commands, text=True, capture_output=True, timeout=3)
    assert result.returncode == 0, result.stderr
    for text in expected:
        assert text in result.stdout, (text, result.stdout)
    return result.stdout


run('done\nENTER_STANDBY\ndone\nSTART_CONTROL\nset following_authorized 0\nGRASP_WHEEL\n'
    'set following_authorized 1\nGRASP_WHEEL\nSTART_CONTROL\ndone\n'
    'set source_authorized 0\nSTART_CONTROL\nset source_authorized 1\nSTART_CONTROL\n'
    'LEAVE_WHEEL\nEXIT_CONTROL\nLEAVE_WHEEL\ndone\nquit\n',
    '[reply] START_CONTROL INVALID_STATE', '[reply] GRASP_WHEEL CAPABILITY_UNAVAILABLE',
    '[reply] START_CONTROL BUSY', '[reply] START_CONTROL CAPABILITY_UNAVAILABLE',
    '[event] START_CONTROL: FOLLOWING -> CONTROL', '[reply] LEAVE_WHEEL INVALID_STATE',
    '[event] done: RELEASING -> STANDBY')

run('done\nENTER_STANDBY\ndone\nGRASP_WHEEL\nfail\nlate_done\nset fault_cleared 0\nRESET_ERROR\n'
    'set fault_cleared 1\nRESET_ERROR\nENTER_STANDBY\nLEAVE_WHEEL\ndone\n',
    '[event] fail: GRASPING -> ERROR', '[event] late_done: ERROR -> ERROR',
    '[reply] RESET_ERROR INVALID_STATE', '[event] RESET_ERROR: ERROR -> SAFE',
    '[reply] ENTER_STANDBY INVALID_STATE', '[event] LEAVE_WHEEL: SAFE -> RELEASING',
    '[event] done: RELEASING -> STANDBY')

for commands, transition in [
    ('advance 30000\n', 'INITIALIZING -> ERROR'),
    ('done\nENTER_STANDBY\ndone\nGRASP_WHEEL\nadvance 180000\n', 'GRASPING -> ERROR'),
    ('done\nENTER_STANDBY\ndone\nGRASP_WHEEL\ndone\nLEAVE_WHEEL\nadvance 180000\n', 'RELEASING -> ERROR'),
]:
    run(commands + 'late_done\n', transition, 'Task deadline exceeded', '[event] late_done: ERROR -> ERROR')

run('done\nENTER_STANDBY\ndone\nGRASP_WHEEL\ndone\nSTART_CONTROL\nset input_ready 0\n'
    'set input_ready 1\nSTART_CONTROL\n',
    '[event] set input_ready 0: CONTROL -> SAFE', '[event] set input_ready 1: SAFE -> SAFE',
    '[reply] START_CONTROL INVALID_STATE', 'state=SAFE accepts_control=0')

run('emergency\nlate_done\nRESET_ERROR\nENTER_STANDBY\nGRASP_WHEEL\n',
    '[event] emergency: INITIALIZING -> EMERGENCY_STOP', '[mock] brake requested',
    '[event] late_done: EMERGENCY_STOP -> EMERGENCY_STOP', '[reply] RESET_ERROR INVALID_STATE')
run('set emergency_known 0\n', 'INITIALIZING -> EMERGENCY_STOP')
run('fault\nRESET_ERROR\nENTER_STANDBY\ndone\n',
    '[event] RESET_ERROR: ERROR -> SAFE', '[event] ENTER_STANDBY: SAFE -> HOMING',
    '[event] done: HOMING -> STANDBY')
for args in (['--config', 'config/system.yaml'], ['--safety-file', '/tmp/unused'], ['--fsm-simulation']):
    result = subprocess.run([exe, *args], capture_output=True, text=True, timeout=3)
    assert result.returncode == 1 and 'offline only' in result.stderr
for command in ('advance -1', 'advance 3600001', 'set ready maybe', 'done extra'):
    result = subprocess.run([exe], input=command+'\n', capture_output=True, text=True, timeout=3)
    assert result.returncode == 1
print('PASS offline CLI: guards, full cycle, recovery, input loss, timeouts, late events, emergency and no device options')

run('done\nGRASP_WHEEL\nstatus\nENTER_STANDBY\nstatus\ndone\n',
    '[event] done: INITIALIZING -> READY', '[reply] GRASP_WHEEL INVALID_STATE',
    '[event] ENTER_STANDBY: READY -> HOMING', '[event] done: HOMING -> STANDBY')
run('done\nENTER_STANDBY\nadvance 180000\n', 'HOMING -> ERROR', 'Task deadline exceeded')
