"""Production simulator with SDL virtual joystick; peer independently encodes uplinks."""
import os
from pathlib import Path
import pty
import re
import struct
import subprocess
import sys
import tempfile
import time
def frames(buffer, size):
    result = []
    while len(buffer) >= size:
        if buffer[:2] != b'\xeb\x90' or buffer[size-1] != sum(buffer[2:size-1]) & 255:
            del buffer[0]
            continue
        result.append(bytes(buffer[:size]))
        del buffer[:size]
    return result



def main():
    driver, config_template = map(Path, sys.argv[1:3])
    with tempfile.TemporaryDirectory(prefix='flight-simulator-') as temp:
        config = config_template.read_text()
        pairs = [pty.openpty(), pty.openpty()]
        for m, _ in pairs: os.set_blocking(m, False)
        # Always isolate tests from locally configured hardware and fault injection.
        for key, value in [('channel_a', os.ttyname(pairs[0][1])),
                           ('channel_b', os.ttyname(pairs[1][1])),
                           ('send_a', 'true'), ('send_b', 'true'),
                           ('require_joystick', 'true'), ('corrupt_every', '0')]:
            config = re.sub(rf'(?m)^(\s*{key}:)[^\n]*', rf'\g<1> {value}', config)
        path = Path(temp)/'simulator.yaml'; path.write_text(config)
        with (Path(temp)/'log').open('w+') as log:
            child = subprocess.Popen([str(driver), str(path)], stdout=log, stderr=log,
                                     env={**os.environ,'SDL_VIDEODRIVER':'dummy'})
            buffers=[bytearray(),bytearray()]; received=[[],[]]; seq=0
            start=time.monotonic()
            try:
                while child.poll() is None and time.monotonic()-start<8:
                    seq+=1
                    elapsed = time.monotonic() - start
                    up=bytearray(64); up[:2]=b'\xeb\x90';up[2]=seq%256
                    up[10]=7;up[11]=up[12]=255;up[22:26]=bytes.fromhex('01 02 cd ab')
                    struct.pack_into('<hh', up, 13, -32767, -32767)
                    struct.pack_into('<hh', up, 26, 32767, 32767)
                    for i,(m,_) in enumerate(pairs):
                        reply = bytearray(up)
                        # ICD 1127: FF emergency, FE healthy startup.
                        # B reports a new alarm, then legacy unknown faults must go STALE.
                        if i == 0:
                            reply[10] = 7 if elapsed < .6 else (0xff if elapsed < 1.6 else 0xfe)
                            if elapsed >= 1.6:
                                struct.pack_into('<h', reply, 15, -32768)
                        elif elapsed >= .6:
                            reply[10] = 0x10
                            struct.pack_into('<H', reply, 17, 7202)
                            reply[21] = 0x30
                            if elapsed >= 1.6:
                                struct.pack_into('<H', reply, 17, 0xffff)
                                reply[21] = 0xff
                        # A received reserved byte must survive the monitor/log capture.
                        reply[30] = 0xa5
                        reply[63] = sum(reply[2:63]) & 255
                        os.write(m,reply)
                        try: data=os.read(m,4096)
                        except BlockingIOError: continue
                        buffers[i].extend(data)
                        received[i].extend((time.monotonic()-start,f) for f in frames(buffers[i],32))
                    time.sleep(.006)
                assert child.wait(timeout=3)==0
                for collection in received:
                    assert len(collection)>70, len(collection)
                    operations=[f for _,f in collection if f[3]==0x10]
                    assert len(operations)==10, len(operations)
                    axes = {struct.unpack_from('<hh', f, 4) for _, f in collection}
                    assert {(0, 2184), (-16384, 17476), (16384, -15292)} <= axes, axes
                    assert max(t for t,_ in collection)<1.95, 'joystick disconnect must stop sending'
                # Both channels carry the same logical frames and counters.
                assert [f for _,f in received[0]] == [f for _,f in received[1]]
                log.flush();log.seek(0);text=log.read()
                # The first one-second sample may still be WAITING; intermediate
                # FOLLOWING frames need not appear in the sampled terminal log.
                assert 'checksum=ABCD' in text, text
                assert 'camera=(-50.000000 deg' in text and 'joint_fk=(50.000000 deg' in text, text
                assert '70.000000 mm' in text and '-80.000000 mm' in text, text
                lines = text.splitlines()
                captures = [bytes.fromhex(line.split('HEX=', 1)[1]) for line in lines if ' 64B HEX=' in line]
                assert captures and all(len(f) == 64 and f[30] == 0xa5 for f in captures), text
                assert any('SIM A LIVE' in line and 'state=EMERGENCY_STOP' in line for line in lines), text
                assert any('SIM A LIVE' in line and 'state=NOT_STARTED' in line
                           and 'camera=(-50.000000 deg, unknown)' in line
                           and 'fault=0 previous=0 severity=0' in line for line in lines), text
                assert any('SIM B LIVE' in line and 'fault=7202' in line and 'severity=48' in line for line in lines), text
                assert any('SIM B STALE' in line and 'state=ERROR' in line and 'fault=7202' in line for line in lines), text
                waiting = [line for line in lines if ' WAITING ' in line]
                assert waiting and all('state=' not in line for line in waiting), text
                print('PASS: production flight simulator, virtual joystick axes/button, dual identical frames, ten-cycle operation, emergency/healthy startup/alarm feedback, legacy unknown faults rejected, disconnect stops TX')
            except Exception:
                log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
            finally:
                if child.poll() is None: child.kill();child.wait()
                for pair in pairs:
                    for fd in pair: os.close(fd)


if __name__=='__main__': main()
