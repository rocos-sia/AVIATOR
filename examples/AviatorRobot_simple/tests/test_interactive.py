#!/usr/bin/env python3
"""顺序发送命令，检查解析、运动中 status/stop 和退出；不需要后台读线程。"""
import os
import select
import subprocess
import sys
import time

args = [sys.argv[1]]
if '--viewer' not in sys.argv:
    args.append('--headless')
proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, bufsize=0)
pending = ''
transcript = []


def wait_for(text, timeout=10):
    global pending
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        while '\n' in pending:
            line, pending = pending.split('\n', 1)
            transcript.append(line)
            if text in line:
                return
        if select.select([proc.stdout], [], [], 0.1)[0]:
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                break
            pending += chunk.decode(errors='replace')
    raise RuntimeError(f'No {text!r} in output')


def send(command):
    proc.stdin.write((command + '\n').encode())
    proc.stdin.flush()


try:
    wait_for('Backend: mujoco')
    send('unknown')
    wait_for('Unknown command')
    send('wheel 0 0 invalid')
    wait_for('Invalid speed ratio')
    send('servo 0 0 .5 extra')
    wait_for('Unexpected argument')
    send('enable')
    wait_for('[ok] ENABLED')
    send('approach')
    time.sleep(0.3)
    send('status')
    wait_for('state=APPROACHING', timeout=3)
    send('stop')
    wait_for('[error]', timeout=5)
    send('status')
    wait_for('state=FAULT')
    send('quit')
    assert proc.wait(timeout=5) == 0

    # EOF 前没有换行，最后一条命令仍应执行。
    eof = subprocess.run([sys.argv[1], '--headless'], input='status', text=True,
                         capture_output=True, timeout=10)
    assert eof.returncode == 0 and 'state=INITIALIZED' in eof.stdout
    print('Interactive parsing, status/stop, EOF and clean exit passed')
except Exception:
    print('\n'.join(transcript) + '\n' + pending)
    raise
finally:
    if proc.poll() is None:
        proc.kill()
        proc.wait()
