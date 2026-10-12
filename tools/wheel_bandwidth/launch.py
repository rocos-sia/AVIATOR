#!/usr/bin/env python3
"""Independent launcher; generated configuration and all outputs belong to one run."""
import argparse
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import yaml

ROOT = Path(__file__).resolve().parents[2]


def require_regular_user():
    if os.geteuid() == 0:
        raise RuntimeError("请去掉命令前的 sudo，直接运行 ./scripts/start_wheel_bandwidth.sh；"
                           "脚本会单独为 manipulator 请求 sudo 认证，其他节点使用当前用户环境")


def can_create_directory(path):
    parent = path.parent
    while not parent.exists() and parent != parent.parent:
        parent = parent.parent
    return parent.is_dir() and os.access(parent, os.W_OK | os.X_OK)


def select_output(requested, root=ROOT, home=None):
    """Select without writing, so dry-run can show the actual destination."""
    if requested is not None:
        output = Path(requested).expanduser().resolve()
    else:
        name = "wheel_bandwidth_" + datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
        output = (root / "logs" / name).resolve()
        if not can_create_directory(output):
            fallback = ((home or Path.home()) / "aviator_logs" / name).resolve()
            print(f"项目日志目录 {output.parent} 不可写，改用用户日志目录 {fallback.parent}。", flush=True)
            output = fallback
    if output.exists():
        raise FileExistsError(f"输出目录已存在：{output}；请使用新的 --output 目录")
    if not can_create_directory(output):
        raise PermissionError(f"输出目录不可写：{output}；请用 --output 指定当前用户可写的新目录")
    return output


def prepare_config(source, destination):
    """Pure transformation: resolve paths before relocating, never write source files."""
    source = Path(source).resolve()
    system = yaml.safe_load(source.read_text())
    robot_path = (source.parent / system["robot"]).resolve()
    robot = yaml.safe_load(robot_path.read_text())
    for key in ("model", "urdf", "grasp", "posture"):
        if key in robot:
            robot[key] = str((robot_path.parent / robot[key]).resolve())
    if "wheel_initial" not in robot:
        raise ValueError("robot.yaml 必须显式配置 wheel_initial")
    angle = float(robot["wheel_initial"]["angle"])
    displacement = float(robot["wheel_initial"]["displacement"])
    if not (math.isfinite(angle) and abs(angle) <= .87266 and
            math.isfinite(displacement) and -.170 <= displacement <= 0):
        raise ValueError("wheel_initial 超出轮盘范围")
    robot.setdefault("grasp_wheel", {})["source"] = "wheel_initial"
    system["robot"] = str(Path(destination).resolve() / "robot.yaml")
    return system, robot, robot_path


def parser():
    p = argparse.ArgumentParser(description="无相机轮盘带宽测试；自动抓握并输入 ±10° 正弦。")
    p.add_argument("--simulation", action="store_true")
    p.add_argument("--headless", action="store_true", help="仿真不打开窗口")
    p.add_argument("--fake-hand", action="store_true", help="真机机械臂搭配模拟手节点")
    p.add_argument("--no-monitor", action="store_true")
    p.add_argument("--logger", action="store_true", help="额外记录 MCAP；CSV 默认记录")
    p.add_argument("--dry-run", action="store_true", help="只校验和打印，不写文件、不启动节点")
    p.add_argument("--config", type=Path)
    p.add_argument("--output", type=Path)
    p.add_argument("--frequencies", default="0.1,0.2,0.3,0.5,0.7,1")
    p.add_argument("--amplitude-deg", type=float, default=10)
    p.add_argument("--settle-cycles", type=int, default=2)
    p.add_argument("--measure-cycles", type=int, default=5)
    p.add_argument("--ramp-cycles", type=int, default=1)
    return p


def main():
    args = parser().parse_args()
    require_regular_user()
    if args.headless and not args.simulation:
        raise ValueError("--headless 仅用于 --simulation")
    frequencies = [float(f) for f in args.frequencies.split(",")]
    if (not frequencies or any(not math.isfinite(f) or f < .01 or f > 5 for f in frequencies)
            or any(a >= b for a, b in zip(frequencies, frequencies[1:]))):
        raise ValueError("频率须严格递增，范围 0.01..5 Hz")
    if not (0 < args.amplitude_deg <= 10 and 0 <= args.settle_cycles <= 100
            and 2 <= args.measure_cycles <= 100 and 1 <= args.ramp_cycles <= 100):
        raise ValueError("幅值须 (0,10]°；稳定周期 0..100、测量周期 2..100、过渡周期 1..100")
    source = (args.config or ROOT / "config" / (
        "system-simulation.yaml" if args.simulation else "system.yaml")).resolve()
    output = select_output(args.output)
    system, robot, source_robot = prepare_config(source, output / "config")
    period = float(robot.get("servo_period", .01))
    if not (.001 <= period <= .05) or max(frequencies)*max(period, .01) > .05:
        raise ValueError("servo_period 须 0.001..0.05 s，且每周期至少20个目标采样点")
    bin_dir = Path(os.environ.get("AVIATOR_BIN", ROOT / "build/release/bin")).resolve()
    config = str(output / "config/system.yaml")
    plan = [("bus", [str(bin_dir / "aviator_bus"), "--config", config], False)]
    if args.simulation:
        command = [str(bin_dir / "simulation"), "--config", config, "--no-camera"]
        if args.headless:
            command += ["--headless"]
        plan.append(("simulation", command, False))
    else:
        plan.append(("manipulator", ["sudo", "-n", str(bin_dir / "manipulator"), "--config", config], True))
        conda = Path(os.environ.get("CONDA_ROOT", Path.home() / "miniconda3"))
        python = os.environ.get("HAND_PYTHON", str(conda / "envs/rh56-pendant/bin/python3"))
        hand = "fake_rh56ftp_hand.py" if args.fake_hand else "rh56ftp_node.py"
        plan.append(("hand", [python, str(ROOT / "nodes/rh56ftp_hand" / hand),
                              "--config", str(ROOT / "config/rh56ftp_hand.yaml")], False))
    if args.logger:
        plan.append(("logger", [str(bin_dir / "aviator_logger"), "--config", str(ROOT / "config/recording.yaml"),
                                "--output", str(output / "recording.mcap")], False))
    if not args.no_monitor:
        plan.append(("monitor", [os.environ.get("MONITOR_BIN", str(bin_dir / "aviator_monitor")),
                                 "--config", str(ROOT / "config/monitor.yaml"), "--log-dir", str(output)], False))
    core = [str(bin_dir / "aviator_core_wheel_bandwidth"), "--config", config, "--output", str(output),
            "--frequencies", args.frequencies, "--amplitude-deg", str(args.amplitude_deg),
            "--settle-cycles", str(args.settle_cycles), "--measure-cycles", str(args.measure_cycles),
            "--ramp-cycles", str(args.ramp_cycles)]
    plan.append(("core", core, False))
    print(f"抓取 wheel_initial={robot['wheel_initial']}；测试以角度 0° 为中心。", flush=True)
    print(f"输出目录：{output}", flush=True)
    for name, command, _ in plan:
        print(f"{name}: {shlex.join(command)}", flush=True)
    if args.dry_run:
        print("dry-run 完成：未写配置，未启动任何节点。")
        return 0
    # Fail before hardware startup if numerical/plotting dependencies are unavailable.
    import numpy  # noqa: F401
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot  # noqa: F401
    for name, command, elevated in plan:
        executable = command[2] if elevated else command[0]
        if not shutil.which(executable):
            raise RuntimeError(f"缺少可执行文件 {executable}；请先构建，见 tools/wheel_bandwidth/README.md")
    lock_path = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / f"aviator-start-{os.getuid()}.lock"
    with lock_path.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for endpoint in (system["bus"]["publish"], system["bus"]["subscribe"], system["manipulator_service"]):
            host, port = endpoint.removeprefix("tcp://").rsplit(":", 1)
            with socket.socket() as sock:
                sock.bind((host, int(port)))
        if not args.no_monitor:
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 8081))
        # Verify actual output creation before any privilege prompt or device startup.
        output.mkdir(parents=True, exist_ok=False)
        (output / "config").mkdir()
        (output / "config/system.yaml").write_text(yaml.safe_dump(system, sort_keys=False))
        (output / "config/robot.yaml").write_text(yaml.safe_dump(robot, sort_keys=False))
        # Preserve geometry and posture used by this run for reproducible offline analysis.
        for key in ("grasp", "posture"):
            copied = output / "config" / (key + ".json")
            shutil.copyfile(robot[key], copied)
            robot[key] = str(copied)
        (output / "config/robot.yaml").write_text(yaml.safe_dump(robot, sort_keys=False))
        (output / "launch.json").write_text(json.dumps({
            "source_system": str(source), "source_robot": str(source_robot),
            "simulation": args.simulation, "options": vars(args)}, default=str, indent=2))
        if not args.simulation:
            subprocess.run(["sudo", "-v"], check=True)
        children = []
        stopping = False
        result = 1

        def on_signal(_signum, _frame):
            nonlocal stopping
            stopping = True

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        def check_nodes():
            for name, proc, _, _ in children:
                if name != "core" and proc.poll() is not None:
                    raise RuntimeError(f"{name} 已退出，请查看 {output / (name + '.log')}")

        def stop(child, timeout):
            name, proc, elevated, _ = child
            if proc.poll() is not None:
                return
            def send(sig):
                if elevated:
                    subprocess.run(["sudo", "-n", "kill", f"-{sig}", "--", f"-{proc.pid}"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                else:
                    try:
                        os.killpg(proc.pid, getattr(signal, "SIG" + sig))
                    except ProcessLookupError:
                        pass
            send("TERM")
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print(f"{name} 停止超时，强制退出。", flush=True)
                send("KILL")
                proc.wait(timeout=5)

        try:
            for name, command, elevated in plan:
                if stopping:
                    break
                log = (output / (name + ".log")).open("w")
                # sudo keeps this terminal's credential cache; separate process group still
                # lets Core brake before the privileged device receives a shutdown signal.
                proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, start_new_session=not elevated,
                                        preexec_fn=os.setpgrp if elevated else None)
                children.append((name, proc, elevated, log))
                if name != "core":
                    until = time.monotonic() + 2
                    while time.monotonic() < until and not stopping:
                        check_nodes()
                        time.sleep(.1)
                if name == "monitor" and shutil.which("google-chrome-stable"):
                    browser_log = (output / "chrome.log").open("w")
                    subprocess.Popen(["google-chrome-stable", "--start-fullscreen", "http://127.0.0.1:8081"],
                                     stdout=browser_log, stderr=browser_log, start_new_session=True)
                    browser_log.close()
            print("测试已启动，进度见 core.log；Ctrl+C 先停止 Core，再清理其他节点。", flush=True)
            next_auth = time.monotonic() + 30
            while not stopping and children and children[-1][0] == "core":
                check_nodes()
                status = children[-1][1].poll()
                if status is not None:
                    result = status
                    break
                if not args.simulation and time.monotonic() >= next_auth:
                    subprocess.run(["sudo", "-n", "-v"], check=True)
                    next_auth = time.monotonic() + 30
                time.sleep(.1)
            if stopping:
                result = 130
        except Exception as error:
            print(f"运行失败：{error}", file=sys.stderr)
            result = 1
        finally:
            for child in reversed(children):
                try:
                    stop(child, 15 if child[0] == "core" else 5)
                except Exception as error:
                    print(f"清理 {child[0]} 失败：{error}", file=sys.stderr)
                    result = 1
                finally:
                    child[3].close()
        if (output / "samples.csv").exists():
            from analyze import analyze
            analyze(output)
        print(f"测试结束，退出码 {result}；结果：{output}", flush=True)
        return result


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"带宽测试失败：{error}", file=sys.stderr)
        sys.exit(1)
