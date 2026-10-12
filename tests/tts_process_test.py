"""Real libzmq transport, audio cache contracts and cancellable child playback.

Uses ctypes for the installed libzmq; no pyzmq/model/audio hardware required.
"""
import ctypes as C
import ctypes.util
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import yaml

binary = str(Path(sys.argv[1]).resolve())
repo = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("tts_generate", repo / "tools/tts/generate.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
lib = C.CDLL(ctypes.util.find_library("zmq"))
lib.zmq_ctx_new.restype = C.c_void_p
lib.zmq_socket.argtypes = [C.c_void_p, C.c_int]
lib.zmq_socket.restype = C.c_void_p
lib.zmq_bind.argtypes = [C.c_void_p, C.c_char_p]
lib.zmq_send.argtypes = [C.c_void_p, C.c_char_p, C.c_size_t, C.c_int]
lib.zmq_getsockopt.argtypes = [C.c_void_p, C.c_int, C.c_void_p, C.POINTER(C.c_size_t)]
lib.zmq_close.argtypes = [C.c_void_p]
lib.zmq_ctx_term.argtypes = [C.c_void_p]
context = lib.zmq_ctx_new()
publisher = lib.zmq_socket(context, 1)  # PUB
assert lib.zmq_bind(publisher, b"tcp://127.0.0.1:*") == 0
endpoint = C.create_string_buffer(128)
length = C.c_size_t(len(endpoint))
assert lib.zmq_getsockopt(publisher, 32, endpoint, C.byref(length)) == 0
clock = os.uname().nodename[:80] + "-" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
sequence = 0


def publish(state, code, valid=True):
    global sequence
    sequence += 1
    message = dict(msg_type="FlightState", version="1.0", sequence=sequence, timestamp=0,
                   sample_mono_us=time.monotonic_ns() // 1000, clock_id=clock,
                   publisher_id="aviator_core", session_id="process-test", valid=valid,
                   system=dict(state=state, state_code=code, generation=1, current_error=""))
    body = json.dumps(message).encode()
    assert lib.zmq_send(publisher, b"flight.state", 12, 2) == 12
    assert lib.zmq_send(publisher, body, len(body), 0) == len(body)


def wait_for(predicate, timeout=5, tick=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        if tick:
            tick()
        time.sleep(.02)
    raise AssertionError("condition timed out")


def dead(pid):
    try:
        os.kill(pid, 0)
        return False
    except ProcessLookupError:
        return True


try:
    with tempfile.TemporaryDirectory(prefix="aviator-tts-test-") as temporary:
        root = Path(temporary)
        config = yaml.safe_load((repo / "config/tts.yaml").read_text())
        config.update(subscribe=endpoint.value.decode(), audio_dir=str(root / "audio"), device="null",
                      freshness_ms=200, lost_after_ms=250, startup_grace_ms=250)
        cfg = root / "tts.yaml"
        cfg.write_text(yaml.safe_dump(config, allow_unicode=True))
        audio = root / "audio"
        # Synthetic PCM is a transport fixture, never installed as a voice pack.
        synthesize = lambda text: ([0.1] * 1600, 16000)
        original = generator.build_pack(config, audio, {"fixture": 1}, synthesize)
        generator.check_pack(config, audio)
        subprocess.run([binary, "--config", str(cfg), "--check"], check=True, capture_output=True)
        saved = (audio / "manifest.json").read_bytes()
        def broken(text):
            raise ValueError("inference failed")
        try:
            generator.build_pack(config, audio, {"fixture": 2}, broken)
            raise AssertionError("expected inference failure")
        except ValueError:
            pass
        assert (audio / "manifest.json").read_bytes() == saved, "failed generation replaced live pack"
        old_path = audio / original["cues"]["READY"]["file"]
        old_bytes = old_path.read_bytes()
        stochastic = generator.build_pack(config, audio, {"fixture": 1}, lambda text: ([0.2] * 1600, 16000))
        assert stochastic["cues"]["READY"]["file"] != original["cues"]["READY"]["file"]
        assert old_path.read_bytes() == old_bytes, "regeneration overwrote immutable audio"
        config["cues"]["READY"]["text"] = "新的初始化文案"
        cfg.write_text(yaml.safe_dump(config, allow_unicode=True))
        assert subprocess.run([binary, "--config", str(cfg), "--check"], capture_output=True).returncode == 1
        updated = generator.build_pack(config, audio, {"fixture": 1}, synthesize)
        assert original["cues"]["READY"]["file"] != updated["cues"]["READY"]["file"]
        # WAV structure accepted by the real playback tool, without a soundcard.
        subprocess.run([binary, "--config", str(cfg), "--test-cue", "READY", "--lock-file", str(root / "lock")],
                       check=True, capture_output=True, timeout=5)
        path = audio / updated["cues"]["READY"]["file"]
        raw = path.read_bytes()
        path.write_bytes(raw[:-1])
        assert subprocess.run([binary, "--config", str(cfg), "--check"], capture_output=True).returncode == 1
        path.write_bytes(raw)
        generator.check_pack(config, audio)

        bad_config = dict(config, device="aviator_tts_missing_test_device")
        bad_cfg = root / "bad-device.yaml"
        bad_cfg.write_text(yaml.safe_dump(bad_config, allow_unicode=True))
        failed = subprocess.run([binary, "--config", str(bad_cfg), "--test-cue", "READY",
                                 "--lock-file", str(root / "lock")], capture_output=True, timeout=5)
        assert failed.returncode == 1, "audio probe must report playback failure"

        # Fake aplay records its PID/argv and remains alive to exercise preemption.
        fake = root / "bin"
        fake.mkdir()
        player_log = root / "players.jsonl"
        (fake / "aplay").write_text(
            "#!/usr/bin/python3\nimport json, os, sys, time\n"
            "with open(os.environ['TTS_TEST_PLAYERS'], 'a') as f:\n"
            " f.write(json.dumps([os.getpid(), sys.argv[1:]]) + '\\n')\n"
            "time.sleep(30)\n")
        (fake / "aplay").chmod(0o755)
        env = dict(os.environ, PATH=str(fake) + ":" + os.environ["PATH"], TTS_TEST_PLAYERS=str(player_log))
        log = root / "node.log"
        players = lambda: [json.loads(line) for line in player_log.read_text().splitlines()] if player_log.exists() else []
        with log.open("w") as out:
            p = subprocess.Popen([binary, "--config", str(cfg), "--lock-file", str(root / "lock")],
                                 stdout=out, stderr=out, env=env)
            child_pids = []
            try:
                wait_for(lambda: "aviator_tts" in log.read_text())
                # PUB/SUB subscription propagation: send repeated fresh baselines.
                end = time.monotonic() + .4
                while time.monotonic() < end:
                    publish("READY", 11)
                    time.sleep(.02)
                assert not players(), "initial ordinary snapshot must stay silent"
                wait_for(lambda: len(players()) >= 1, tick=lambda: publish("CONTROL", 4))
                child_pids.append(players()[0][0])
                wait_for(lambda: len(players()) >= 2, tick=lambda: publish("EMERGENCY_STOP", 7))
                child_pids.append(players()[1][0])
                wait_for(lambda: dead(child_pids[0]))
                expected = str(audio / updated["cues"]["EMERGENCY_STOP"]["file"])
                assert players()[1][1] == ["-q", "-D", "null", expected], players()
                # Current cue heartbeats must not spawn new players.
                for _ in range(8):
                    publish("EMERGENCY_STOP", 7)
                    time.sleep(.02)
                assert len(players()) == 2
                duplicate = subprocess.run([binary, "--config", str(cfg), "--lock-file", str(root / "lock")],
                                           capture_output=True, timeout=5)
                assert duplicate.returncode == 1 and b"instance lock" in duplicate.stderr
                p.send_signal(signal.SIGTERM)
                assert p.wait(timeout=3) == 0
                wait_for(lambda: all(dead(pid) for pid in child_pids))
            finally:
                if p.poll() is None:
                    p.terminate()
                    p.wait(timeout=5)
                for pid, _ in players():
                    if not dead(pid):
                        os.kill(pid, signal.SIGKILL)
            if p.returncode:
                raise AssertionError(log.read_text())

        # No assets needed in dry-run; timeout/recovery operate on real messages.
        config["audio_dir"] = str(root / "missing")
        cfg.write_text(yaml.safe_dump(config, allow_unicode=True))
        with log.open("w") as out:
            p = subprocess.Popen([binary, "--config", str(cfg), "--dry-run", "--lock-file", str(root / "lock")], stdout=out, stderr=out)
            try:
                wait_for(lambda: "cue=COMM_LOST" in log.read_text())
                wait_for(lambda: "cue=ERROR" in log.read_text(), tick=lambda: publish("ERROR", 6))
                wait_for(lambda: "cue=COMM_RESTORED" in log.read_text(), tick=lambda: publish("ERROR", 6))
                p.send_signal(signal.SIGINT)
                assert p.wait(timeout=3) == 0
            finally:
                if p.poll() is None:
                    p.kill()
                    p.wait()
        print("TTS assets, ZMQ process, cancellation and signals passed")
finally:
    lib.zmq_close(publisher)
    lib.zmq_ctx_term(context)
