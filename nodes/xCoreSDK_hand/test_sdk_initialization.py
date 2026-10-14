"""Actual vendor SDK initialization, loopback only; never connects to a robot."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

library = Path(sys.argv[1]).resolve()
source = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='xcore-hand-sdk-test-') as folder:
    root = Path(folder)
    # Model a cwd whose SDK logs cannot be opened (e.g. root-owned arm logs).
    (root / 'logs').write_text('not a writable SDK log directory')
    code = '''
import ctypes
import os
from pathlib import Path
import sys
sys.path.insert(0, sys.argv[1])
from main import prepare_sdk_directory
library = Path(sys.argv[2]).resolve()
previous = Path.cwd()
directory = prepare_sdk_directory(previous / 'hand-sdk')
assert directory != previous and Path.cwd() == directory
sdk = ctypes.CDLL(str(library))
sdk.hand_sdk_connect.argtypes = [ctypes.c_char_p, ctypes.c_int]
sdk.hand_sdk_connect.restype = ctypes.c_void_p
sdk.hand_sdk_error.restype = ctypes.c_char_p
# Only loopback, with no robot motion/power/RS485 writes. The expected result
# is a caught connection error, rather than SIGABRT in logger initialization.
handle = sdk.hand_sdk_connect(b'127.0.0.1', 50)
assert not handle, 'Unexpected local SDK server'
message = sdk.hand_sdk_error().decode()
assert message, 'Missing SDK connection diagnostic'
assert list((directory / 'logs').glob('global_logger_*.log'))
assert (previous / 'logs').read_text() == 'not a writable SDK log directory'
print('PASS actual SDK initialized its private logger; loopback failure returned:', message, flush=True)
'''
    result = subprocess.run([sys.executable, '-c', code, str(source), str(library)],
                            cwd=root, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=20)
    print(result.stdout, end='')
    assert result.returncode == 0, f'SDK initialization subprocess exited {result.returncode}'
