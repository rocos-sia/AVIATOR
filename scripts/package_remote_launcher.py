#!/usr/bin/env python3
"""Build a portable Ubuntu UI bundle without the robot application or its source."""
import argparse
import hashlib
from pathlib import Path
import tarfile
import tempfile
import shutil

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description="打包 Ubuntu 22.04/24.04 独立远程启动界面")
    parser.add_argument("--output", type=Path, default=ROOT / "build/remote-launcher/aviator-remote-ui.tar.gz")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="aviator-ui-package-") as temp:
        bundle = Path(temp) / "aviator-remote-ui"
        source = ROOT / "tools/remote_launcher"
        target = bundle / "tools/remote_launcher"
        target.mkdir(parents=True)
        for name in ("main.py", "appearance.py", "backend.py", "remote_agent.py", "connection.json"):
            shutil.copy2(source / name, target / name)
        (bundle / "scripts").mkdir()
        shutil.copy2(ROOT / "scripts/start_remote_aviator.sh", bundle / "scripts/start_remote_aviator.sh")
        shutil.copy2(source / "install_dependencies.sh", bundle / "install_dependencies.sh")
        shutil.copy2(source / "README.md", bundle / "README.md")
        launcher = bundle / "start.sh"
        launcher.write_text('#!/usr/bin/env bash\nset -euo pipefail\nROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)\nexec bash "$ROOT/scripts/start_remote_aviator.sh" "$@"\n')
        for file in (launcher, bundle / "install_dependencies.sh", bundle / "scripts/start_remote_aviator.sh"):
            file.chmod(0o755)
        with tarfile.open(args.output, "w:gz") as archive:
            archive.add(bundle, arcname=bundle.name)
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_suffix(args.output.suffix + ".sha256").write_text(digest + "  " + args.output.name + "\n")
    print(args.output)
    print("包内包含固定 SSH 登录凭据，仅分发给可信控制机。")


if __name__ == "__main__":
    main()
