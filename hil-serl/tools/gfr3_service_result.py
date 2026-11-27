"""Persist service termination even if the memory controller killed the learner."""
import argparse
import json
import os
from pathlib import Path
from tools.gfr1_remount_evaluate import dump


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--root",required=True)
    root=Path(p.parse_args().root)
    report={key:os.getenv(key) for key in ("SERVICE_RESULT","EXIT_CODE","EXIT_STATUS")}
    group=next(line.split(":",2)[2] for line in Path("/proc/self/cgroup").read_text().splitlines() if line.startswith("0::"))
    directory=Path("/sys/fs/cgroup")/group.lstrip("/")
    for name in ("memory.max","memory.peak","memory.events"):
        path=directory/name
        if path.exists():report[name]=path.read_text().strip()
    dump(root/"service_result.json",report)
    if report["SERVICE_RESULT"]!="success":
        status=root/"status.json"
        previous=json.loads(status.read_text()) if status.exists() else {}
        dump(status,dict(phase="stopped_on_service_error",execution_completed=False,
                         scientific_qualification=False,service=report,last_status=previous))
