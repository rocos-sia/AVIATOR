"""Shared console Logger for Python nodes; spdlog via the optional CMake bridge.

Source-only camera/hand deployments retain a stdlib fallback until CMake is built.
AVIATOR_LOGGING_LIBRARY can select a nonstandard build/install directory.
"""
import atexit
import ctypes
import ctypes.util
import datetime
import os
from pathlib import Path
import sys
import threading


class Logger:
    _lock = threading.RLock()
    _library = None
    _callback = None
    _initialized = False

    @staticmethod
    def _write(level, text, stream=None):
        stream = stream or (sys.stderr if level >= 3 else sys.stdout)
        colors = {2: "32", 3: "33", 4: "31", 5: "1;31"}
        if not os.environ.get("NO_COLOR") and getattr(stream, "isatty", lambda: False)():
            text = f"\033[{colors.get(level, '37')}m{text.rstrip()}\033[0m\n"
        stream.write(text)
        stream.flush()

    @classmethod
    def configure(cls, async_mode=False):
        with cls._lock:
            if not cls._initialized:
                root = Path(__file__).resolve().parent.parent
                candidates = [os.environ.get("AVIATOR_LOGGING_LIBRARY"),
                              ctypes.util.find_library("aviator_logging_python"),
                              str(root.parent.parent / "lib/libaviator_logging_python.so")]
                candidates += [str(p) for p in sorted(root.glob("build/*/lib/libaviator_logging_python.so"))]
                for candidate in filter(None, candidates):
                    try:
                        cls._library = ctypes.CDLL(candidate)
                        break
                    except OSError:
                        pass
                cls._initialized = True
                if cls._library:
                    callback_type = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p)
                    cls._callback = callback_type(lambda level, text: cls._write(level, text.decode("utf-8", errors="replace")))
                    cls._library.aviator_logger_configure.argtypes = [ctypes.c_int, callback_type]
                    cls._library.aviator_logger_configure.restype = ctypes.c_int
                    cls._library.aviator_logger_write.argtypes = [ctypes.c_int, ctypes.c_char_p]
                    cls._library.aviator_logger_write.restype = None
                    cls._library.aviator_logger_shutdown.argtypes = []
                    cls._library.aviator_logger_shutdown.restype = None
                    atexit.register(cls._library.aviator_logger_shutdown)
            if cls._library and cls._library.aviator_logger_configure(async_mode, cls._callback) != 0:
                raise RuntimeError("Cannot configure spdlog console logger")
            if async_mode and not cls._library:
                raise RuntimeError("Async logging requires the aviator_logging_python CMake target")

    @classmethod
    def _log(cls, level, message, *args, sep=" ", end="\n", file=None, flush=False):
        if not cls._initialized:
            cls.configure()
        text = sep.join(str(value) for value in (message, *args)).rstrip("\r\n")
        # Explicit stdout/stderr parameters preserve the existing CLI contracts.
        if cls._library and (file is None or file is (sys.stderr if level >= 3 else sys.stdout)):
            cls._library.aviator_logger_write(level, text.encode("utf-8"))
        else:
            names = {0: "trace", 1: "debug", 2: "info", 3: "warning", 4: "error", 5: "critical"}
            stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with cls._lock:
                cls._write(level, f"[{stamp}] [aviator] [{names[level]}] {text}\n", file)

    @classmethod
    def info(cls, message, *args, **kwargs):
        cls._log(2, message, *args, **kwargs)

    @classmethod
    def warn(cls, message, *args, **kwargs):
        cls._log(3, message, *args, **kwargs)

    @classmethod
    def error(cls, message, *args, **kwargs):
        cls._log(4, message, *args, **kwargs)

    @staticmethod
    def output(*args, **kwargs):
        """Unadorned machine-readable CLI output (e.g. --dry-run)."""
        print(*args, **kwargs)
