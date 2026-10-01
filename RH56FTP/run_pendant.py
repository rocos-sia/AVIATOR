#!/usr/bin/env python3
"""示教器启动器：从任意目录运行，不用先 cd 到 python/。

    python3 run_pendant.py            # 实机 192.168.11.210:6000
    python3 run_pendant.py --mock     # 无硬件
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "python"))

from pendant.main import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
