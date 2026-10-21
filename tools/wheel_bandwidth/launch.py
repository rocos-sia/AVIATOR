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
    hand = system.setdefault("xcore_sdk_hand", {})
    hand["config"] = str((source.parent / hand.get("config", "rh56ftp_hand.yaml")).resolve())
    system["robot"] = str(Path(destination).resolve() / "robot.yaml")
    return system, robot, robot_path


def parser():
    p = argparse.ArgumentParser(description="无相机轮盘带宽测试；自动抓握并按所选轴输入正弦。")
    p.add_argument("--simulation", action="store_true")
    p.add_argument("--headless", action="store_true", help="仿真不打开窗口")
    p.add_argument("--fake-hand", action=argparse.BooleanOptionalAction, default=None,
                   help="真机机械臂搭配模拟手节点；--no-fake-hand 覆盖配置")
    p.add_argument("--no-monitor", action="store_true")
    p.add_argument("--logger", action="store_true", help="额外记录 MCAP；CSV 默认记录")
    p.add_argument("--dry-run", action="store_true", help="只校验和打印，不写文件、不启动节点")
    p.add_argument("--ignore-geometry", action=argparse.BooleanOptionalAction, default=None,
                   help="分析时允许几何超限频点参与带宽估计；保留告警，不改变设备保护")
    p.add_argument("--config", type=Path)
    p.add_argument("--sweep-config", type=Path, default=ROOT / "config/wheel_bandwidth.yaml",
                   help="测试参数 YAML，默认 config/wheel_bandwidth.yaml；命令行参数优先")
    p.add_argument("--output", type=Path)
    p.add_argument("--frequencies", help="逗号分隔的频率列表，覆盖测试配置")
    p.add_argument("--axis", choices=("rotation", "translation"),
                   help="rotation 转动（默认）；translation 推拉")
    amplitude = p.add_mutually_exclusive_group()
    amplitude.add_argument("--amplitude-deg", type=float, help="转动单边幅值，覆盖测试配置，上限 10°")
    amplitude.add_argument("--amplitude-mm", type=float, help="推拉单边幅值，覆盖测试配置")
    p.add_argument("--settle-cycles", type=int)
    p.add_argument("--measure-cycles", type=int)
    p.add_argument("--ramp-cycles", type=int)
    return p


def load_sweep_config(args):
    args.sweep_config = args.sweep_config.expanduser().resolve()
    settings = yaml.safe_load(args.sweep_config.read_text())
    required = {"axis", "amplitude_deg", "amplitude_mm", "frequencies_hz",
                "settle_cycles", "measure_cycles", "ramp_cycles", "fake_hand", "ignore_geometry"}
    if not isinstance(settings, dict) or set(settings) != required:
        raise ValueError(f"测试配置须包含且仅包含：{', '.join(sorted(required))}")
    if settings["axis"] not in ("rotation", "translation"):
        raise ValueError("测试配置 axis 须为 rotation 或 translation")
    for key in ("fake_hand", "ignore_geometry"):
        if type(settings[key]) is not bool:
            raise ValueError(f"测试配置 {key} 须为 true 或 false")
    for key in ("settle_cycles", "measure_cycles", "ramp_cycles"):
        if type(settings[key]) is not int:
            raise ValueError(f"测试配置 {key} 须为整数")
    for key in ("amplitude_deg", "amplitude_mm"):
        value = settings[key]
        limit = 10 if key == "amplitude_deg" else 85
        if type(value) not in (int, float) or not 0 < value <= limit:
            raise ValueError(f"测试配置 {key} 须为 (0,{limit}] 内的数值")
    frequencies = settings["frequencies_hz"]
    if not isinstance(frequencies, list) or not frequencies or any(
            type(f) not in (int, float) or not math.isfinite(f) for f in frequencies):
        raise ValueError("测试配置 frequencies_hz 须为有限数值列表")
    for key in ("axis", "settle_cycles", "measure_cycles", "ramp_cycles", "fake_hand", "ignore_geometry"):
        if getattr(args, key) is None:
            setattr(args, key, settings[key])
    amplitude_key = "amplitude_deg" if args.axis == "rotation" else "amplitude_mm"
    if getattr(args, amplitude_key) is None:
        setattr(args, amplitude_key, settings[amplitude_key])
    if args.frequencies is None:
        args.frequencies = ",".join(str(f) for f in frequencies)


def validate_motion(args, robot):
    """Validate the complete requested excursion before starting any nodes."""
    center = float(robot["wheel_initial"]["displacement"])
    if args.axis == "rotation":
        if args.amplitude_mm is not None:
            raise ValueError("--amplitude-mm 须配合 --axis translation")
        if not 0 < args.amplitude_deg <= 10:
            raise ValueError("转动幅值须 (0,10]°")
    else:
        if args.amplitude_deg is not None:
            raise ValueError("--axis translation 须使用 --amplitude-mm")
        amplitude = args.amplitude_mm / 1000
        if not (math.isfinite(amplitude) and amplitude > 0 and
                center - amplitude >= -.170 and center + amplitude <= 0):
            raise ValueError("推拉幅值须为有限正数，且中心 ± 幅值须在 [-170,0] mm 内")


def main():
    args = parser().parse_args()
    require_regular_user()
    load_sweep_config(args)
    if args.fake_hand and args.simulation:
        raise ValueError("--fake-hand 与 --simulation 不能同时使用")
    if args.headless and not args.simulation:
        raise ValueError("--headless 仅用于 --simulation")
    frequencies = [float(f) for f in args.frequencies.split(",")]
    if (not frequencies or any(not math.isfinite(f) or f < .01 or f > 5 for f in frequencies)
            or any(a >= b for a, b in zip(frequencies, frequencies[1:]))):
        raise ValueError("频率须严格递增，范围 0.01..5 Hz")
    if not (0 <= args.settle_cycles <= 100
            and 2 <= args.measure_cycles <= 100 and 1 <= args.ramp_cycles <= 100):
        raise ValueError("稳定周期 0..100、测量周期 2..100、过渡周期 1..100")
    source = (args.config or ROOT / "config" / (
        "system-simulation.yaml" if args.simulation else "system.yaml")).resolve()
    output = select_output(args.output)
    system, robot, source_robot = prepare_config(source, output / "config")
    validate_motion(args, robot)
    period = float(robot.get("servo_period", .01))
    if not (.001 <= period <= .05) or max(frequencies)*max(period, .01) > .05:
        raise ValueError("servo_period 须 0.001..0.05 s，且每周期至少20个目标采样点")
    bin_dir = Path(os.environ.get("AVIATOR_BIN", ROOT / "build/bin")).expanduser().resolve()
    config = str(output / "config/system.yaml")
    plan = [("bus", [str(bin_dir / "aviator_bus"), "--config", config], False)]
    if args.simulation:
        command = [str(bin_dir / "simulation"), "--config", config, "--no-camera"]
        if args.headless:
            command += ["--headless"]
        plan.append(("simulation", command, False))
    else:
        command = ["sudo", "-n", str(bin_dir / "manipulator"), "--config", config]
        plan.append(("manipulator", command, True))
        hand = [str(bin_dir / "xCoreSDK_hand"), "--system-config", config,
                "--sdk-work-dir", str(output / "hand-sdk")]
        if args.fake_hand:
            hand.append("--fake-hand")
        plan.append(("hand", hand, False))
    if args.logger:
        plan.append(("logger", [str(bin_dir / "aviator_logger"), "--config", str(ROOT / "config/recording.yaml"),
                                "--output", str(output / "recording.mcap")], False))
    if not args.no_monitor:
        plan.append(("monitor", [os.environ.get("MONITOR_BIN", str(bin_dir / "aviator_monitor")),
                                 "--config", str(ROOT / "config/monitor.yaml"), "--log-dir", str(output)], False))
    amplitude_option = (["--amplitude-deg", str(args.amplitude_deg)] if args.axis == "rotation"
                        else ["--amplitude-mm", str(args.amplitude_mm)])
    core = [str(bin_dir / "aviator_core_wheel_bandwidth"), "--config", config, "--output", str(output),
            "--frequencies", args.frequencies, "--axis", args.axis, *amplitude_option,
            "--settle-cycles", str(args.settle_cycles), "--measure-cycles", str(args.measure_cycles),
            "--ramp-cycles", str(args.ramp_cycles)]
    plan.append(("core", core, False))
    center_mm = float(robot["wheel_initial"]["displacement"]) * 1000
    print(f"抓取 wheel_initial={robot['wheel_initial']}。", flush=True)
    print(f"测试配置：{args.sweep_config}；频率 {args.frequencies} Hz；"
          f"等待/测量/过渡周期 {args.settle_cycles}/{args.measure_cycles}/{args.ramp_cycles}；"
          f"fake_hand={args.fake_hand}。", flush=True)
    if args.axis == "rotation":
        print(f"转动测试：0° ± {args.amplitude_deg:g}°；推拉保持 {center_mm:g} mm。", flush=True)
    else:
        print(f"推拉测试：{center_mm:g} ± {args.amplitude_mm:g} mm"
              f"（{center_mm-args.amplitude_mm:g}..{center_mm+args.amplitude_mm:g} mm）；角度保持 0°。",
              flush=True)
    print(f"输出目录：{output}", flush=True)
    if args.ignore_geometry:
        print("分析将忽略几何超限，结果带 _geometry_ignored 后缀；仅作关节反馈链路估计。", flush=True)
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
        shutil.copyfile(args.sweep_config, output / "config/wheel_bandwidth.yaml")
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
            analyze(output, ignore_geometry=args.ignore_geometry)
        print(f"测试结束，退出码 {result}；结果：{output}", flush=True)
        return result


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"带宽测试失败：{error}", file=sys.stderr)
        sys.exit(1)
