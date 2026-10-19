"""Production simulator with SDL virtual joystick; peer independently encodes uplinks."""
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import tempfile
import time
import yaml
from rs422_process_test import frames


def main():
    driver, root = map(Path, sys.argv[1:3])
    with tempfile.TemporaryDirectory(prefix='aviator-rs422-simulator-') as temp:
        config = yaml.safe_load((root/'config/flight-simulator.yaml').read_text())
        pairs = [pty.openpty(), pty.openpty()]
        for m, _ in pairs: os.set_blocking(m, False)
        config['rs422'].update(channel_a=os.ttyname(pairs[0][1]), channel_b=os.ttyname(pairs[1][1]))
        path = Path(temp)/'simulator.yaml'; path.write_text(yaml.safe_dump(config))
        with (Path(temp)/'log').open('w+') as log:
            child = subprocess.Popen([str(driver), str(path)], stdout=log, stderr=log,
                                     env={**os.environ,'SDL_VIDEODRIVER':'dummy'})
            buffers=[bytearray(),bytearray()]; received=[[],[]]; seq=0
            start=time.monotonic()
            try:
                while child.poll() is None and time.monotonic()-start<8:
                    seq+=1
                    up=bytearray(64); up[:2]=b'\xeb\x90';up[2]=seq%256
                    up[10]=7;up[11]=up[12]=255;up[22:26]=bytes.fromhex('03 10 cd ab')
                    up[63]=-sum(up[:63]) & 255
                    for i,(m,_) in enumerate(pairs):
                        os.write(m,up)
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
                    assert any(struct.unpack_from('<hh',f,4)==(500,-500) for _,f in collection)
                    assert max(t for t,_ in collection)<1.95, 'joystick disconnect must stop sending'
                # Both channels carry the same logical frames and counters.
                assert [f for _,f in received[0]] == [f for _,f in received[1]]
                log.flush();log.seek(0);text=log.read()
                assert 'FOLLOWING' in text and 'checksum=ABCD' in text, text
                print('PASS: production flight simulator, virtual joystick axes/button, dual identical frames, ten-cycle operation, uplink display, disconnect stops TX')
            except Exception:
                log.flush();log.seek(0);print(log.read(),file=sys.stderr);raise
            finally:
                if child.poll() is None: child.kill();child.wait()
                for pair in pairs:
                    for fd in pair: os.close(fd)


if __name__=='__main__': main()
