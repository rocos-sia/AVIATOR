#!/usr/bin/env python3
"""Build offline C ABI using the existing clearance tool's CMake flags/libraries."""
from pathlib import Path
import shlex,subprocess
root=Path(__file__).resolve().parents[2];b=root/'AviatorRobot/build'
f=(b/'CMakeFiles/aviator_clearance_trajectory.dir/flags.make').read_text();flags=[]
for l in f.splitlines():
 if l.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(l.split('=',1)[1])
link=shlex.split((b/'CMakeFiles/aviator_clearance_trajectory.dir/link.txt').read_text());libs=link[link.index('-o')+2:]
temporary=b/'lib/libpose_slack_bridge.next.so'
subprocess.run(['/usr/bin/c++',*flags,'-shared','-fPIC',str(root/'AviatorRobot/tools/pose_slack_bridge.cpp'),'-o',str(temporary),*libs],cwd=b,check=True)
temporary.replace(b/'lib/libpose_slack_bridge.so')
