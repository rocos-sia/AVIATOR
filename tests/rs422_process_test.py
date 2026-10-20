"""Two independent PTYs exercise the real robot gateway and local ZMQ/Core contract."""
import json
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time

import yaml
import zmq


def down(counter, operation=0, roll=0, pitch=0):
    data = struct.pack('<BBBBhh', 0xeb, 0x90, counter % 256, operation, roll, pitch)
    data += bytes([0xa5]) + bytes(22)  # Nonzero reserved byte must survive observation.
    return data + bytes([sum(data[2:]) & 255])


def frames(buffer, size):
    found = []
    while len(buffer) >= size:
        if buffer[:2] != b'\xeb\x90':
            del buffer[0]
            continue
        candidate = bytes(buffer[:size])
        if candidate[-1] != sum(candidate[2:-1]) & 255:
            del buffer[0]
            continue
        found.append(candidate)
        del buffer[:size]
    return found


def main():
    binary, root = map(Path, sys.argv[1:3])
    with tempfile.TemporaryDirectory(prefix='aviator-rs422-') as temp:
        directory = Path(temp)
        context = zmq.Context()
        commands = context.socket(zmq.SUB)
        commands.subscribe(b'flight.command')
        status = context.socket(zmq.PUB)
        service = context.socket(zmq.ROUTER)
        endpoints = []
        for sock in (commands, status, service):
            endpoints.append(f'tcp://127.0.0.1:{sock.bind_to_random_port("tcp://127.0.0.1")}')
        pairs = [pty.openpty(), pty.openpty()]
        for master, _ in pairs:
            os.set_blocking(master, False)
        config = yaml.safe_load((root / 'config/flight-rs422.yaml').read_text())
        config.update(publish=endpoints[0], subscribe=endpoints[1], service=endpoints[2], lock_file=str(directory/'lock'))
        config['rs422'].update(channel_a=os.ttyname(pairs[0][1]), channel_b=os.ttyname(pairs[1][1]), recovery_frames=2)
        cfg = directory/'robot.yaml'
        cfg.write_text(yaml.safe_dump(config))
        log = (directory/'gateway.log').open('w+')
        child = subprocess.Popen([str(binary), '--config', str(cfg)], stdout=log, stderr=log,
                                 env={**os.environ, 'SDL_VIDEODRIVER': 'unavailable_driver'})
        clock = socket.gethostname() + '-' + Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        buffers = [bytearray(), bytearray()]
        received = [[], []]
        seen_commands, requests = [], []
        tick = sequence = 0
        batch_size = int(sys.argv[3]) if len(sys.argv) > 3 else 1
        pending_bytes = [bytearray(), bytearray()]
        next_tick = time.monotonic()
        last_request_count = 0

        def cycle(operation=0, a=True, b=True, roll=125, pitch=-250, malformed=False, core=True):
            nonlocal tick, sequence, next_tick
            now = time.monotonic()
            if next_tick > now:
                time.sleep(next_tick-now)
            next_tick = max(next_tick+.0125, time.monotonic())
            tick += 1
            data = down(tick, operation, roll, pitch)
            if malformed:
                data = data[:-1] + bytes([data[-1] ^ 1])
            for channel, (enabled, (master, _)) in enumerate(zip((a, b), pairs)):
                if enabled:
                    # Force fragmented reception, plus normal coalescing across scheduler delays.
                    pending_bytes[channel].extend(data)
                    if tick % batch_size == 0:
                        data_out = pending_bytes[channel]
                        os.write(master, data_out[:3]); os.write(master, data_out[3:])
                        pending_bytes[channel].clear()
                else:
                    pending_bytes[channel].clear()
            sequence += 1
            sampled = time.monotonic_ns()//1000
            if core:
                msg = dict(msg_type='FlightState', version='1.0', sequence=sequence,
                           timestamp=time.time_ns()//1000, sample_mono_us=sampled,
                           publisher_id='aviator_core', session_id='test-core', clock_id=clock, valid=True,
                           system=dict(state='FOLLOWING', current_error_code=0, last_error_code=1203,
                                       source_authorized=True, input_source='FLIGHT'),
                           wheel_measurement=dict(valid=True, sample_mono_us=sampled, angle=0., displacement=-.085))
                status.send_multipart([b'flight.state', json.dumps(msg).encode()])
            until = time.monotonic()+.004
            while time.monotonic() < until:
                for i, (master, _) in enumerate(pairs):
                    try:
                        data = os.read(master, 4096)
                    except BlockingIOError:
                        continue
                    buffers[i].extend(data)
                    received[i].extend(frames(buffers[i], 64))
                while commands.poll(0):
                    seen_commands.append(json.loads(commands.recv_multipart()[1]))
                while service.poll(0):
                    route, payload = service.recv_multipart()
                    req = json.loads(payload)
                    requests.append(req)
                    # Deliberately do not answer: timeout must not retry the operation.
                assert child.poll() is None, 'gateway exited'
                time.sleep(.0005)

        try:
            for _ in range(65): cycle()
            assert received[0] and received[1], 'both uplinks required'
            for collection in received:
                last = collection[-1]
                assert last[3:5] == bytes([0, 0]), last.hex()
                assert last[10] == 7 and last[11:13] == b'\xff\xff', last.hex()
                assert struct.unpack_from('<hh', last, 13) == (0, 0)
                assert struct.unpack_from('<H', last, 24)[0] == 0x9ca1
            valid = [m for m in seen_commands if m['valid']]
            assert valid and valid[-1]['source'] == 'FLIGHT' and 'input_state' not in valid[-1]
            assert valid[-1]['control'] == {'roll': .125, 'pitch': -.25}
            observation = valid[-1]['rs422']
            assert observation['selected_channel'] == 'A'
            for channel in observation['channels']:
                raw = bytes(channel['raw_bytes'])
                assert raw == down(raw[2], roll=125, pitch=-250), raw.hex()
                assert channel['healthy'] and channel['accepted_frames'] > 0
            assert observation['channels'][0]['received_mono_us'] == valid[-1]['sample_mono_us']
            for _ in range(10): cycle(0x20)
            for _ in range(12): cycle(0x20, a=False)
            assert len(requests) == 1 and requests[0]['operation'] == 'grasp_wheel', requests
            assert requests[0]['parameters'] == {'source': 'FLIGHT', 'operation_code': 0x20}
            assert received[1][-1][3] == 1 and received[1][-1][4] == 1, received[1][-1].hex()
            observation = seen_commands[-1]['rs422']
            assert observation['selected_channel'] == 'B'
            assert not observation['channels'][0]['healthy'] and observation['channels'][1]['healthy']
            assert observation['channels'][1]['received_mono_us'] == seen_commands[-1]['sample_mono_us']
            for _ in range(24): cycle()
            assert received[0][-1][3] == 1, 'primary recovery must not preempt'
            for _ in range(10): cycle(0x40)
            for _ in range(10): cycle()
            assert len(requests) == 2 and requests[1]['operation'] == 'exit_control'
            # Invalid CRC frames cannot renew either input lease.
            for _ in range(14): cycle(malformed=True)
            assert not seen_commands[-1]['valid'] and received[0][-1][4] == 3
            observation = seen_commands[-1]['rs422']
            for channel in observation['channels']:
                assert not channel['healthy'] and channel['rejected_bytes'] > 0
                raw = bytes(channel['raw_bytes'])
                assert raw[-1] == sum(raw[2:-1]) & 255, 'bad checksum replaced accepted frame'
                assert seen_commands[-1]['sample_mono_us'] - channel['received_mono_us'] >= 100000
            # Core silence must produce unknown state/measurements, not refreshed cached feedback.
            for _ in range(18): cycle(core=False)
            last = received[0][-1]
            assert last[10] == 255 and struct.unpack_from('<h',last,13)[0] == -32768
            # New samples recover; no old operation is replayed.
            for _ in range(25): cycle()
            assert seen_commands[-1]['valid'] and len(requests) == 2
            print('PASS: real dual-PTY robot, 80 Hz uplinks, FLIGHT input, single execution, failover, CRC loss, stale Core, recovery')
        except Exception:
            log.flush();log.seek(0);print(log.read(), file=sys.stderr)
            raise
        finally:
            child.terminate()
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: child.kill();child.wait()
            log.close()
            for pair in pairs:
                for fd in pair: os.close(fd)
            for sock in (commands,status,service): sock.close(0)
            context.term()


if __name__ == '__main__':
    main()
