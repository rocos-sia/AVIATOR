#!/usr/bin/env python3
"""Read Rokae feedback through the installed SDK helper and save home angles."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def state_helper(name="aviator_grasp_tool_state"):
    if os.environ.get("AVIATOR_BIN"):
        directories = [Path(os.environ["AVIATOR_BIN"])]
    elif (ROOT.parent.parent / "release-version.json").is_file():
        directories = [ROOT.parent.parent / "bin"]
    else:
        directories = [ROOT / "build/bin", ROOT / "build/release/bin", ROOT / "build/debug/bin"]
    for directory in directories:
        helper = directory / name
        if helper.is_file() and os.access(helper, os.X_OK):
            return helper.resolve()
    raise RuntimeError(f"找不到 {name}；请构建启用 AVIATOR_BUILD_XCORE_SDK 的版本，"
                       "或通过 AVIATOR_BIN 指定二进制目录。部署机器请安装完整硬件部署包。")


def home_angles(sample, *, demo=False):
    if not isinstance(sample, dict) or sample.get("source") != ("offline" if demo else "robot"):
        raise ValueError("采样不是机械臂实测反馈")
    result = {}
    for side in ("left", "right"):
        arm = sample.get(side)
        values = arm.get("joint_position") if isinstance(arm, dict) else None
        if not isinstance(values, list) or len(values) != 7:
            raise ValueError(f"{side} 必须包含 7 个实测关节角")
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise ValueError(f"{side} 关节角必须为有限数值")
        degrees = [math.degrees(value) for value in values]
        if not all(math.isfinite(value) for value in degrees):
            raise ValueError(f"{side} 关节角转换为度失败")
        result[f"{side}_home_deg"] = degrees
    return result


def read_posture(robot_config):
    import yaml

    try:
        config = yaml.safe_load(robot_config.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"robot YAML 格式错误: {error}") from error
    if not isinstance(config, dict) or not isinstance(config.get("posture"), str) or not config["posture"]:
        raise ValueError("robot 配置缺少 posture 路径")
    posture = (robot_config.parent / config["posture"]).resolve(strict=True)
    original = posture.read_bytes()
    data = json.loads(original)
    if not isinstance(data, dict) or not all(key in data for key in ("left_home_deg", "right_home_deg")):
        raise ValueError("posture JSON 必须包含 left_home_deg 和 right_home_deg")
    return posture, original, data


def save_home(posture, original, sample):
    # Always validate the source here, including callers from the GUI.
    angles = home_angles(sample)
    data = json.loads(original)
    data.update(angles)
    updated = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    if posture.read_bytes() != original:
        raise RuntimeError("采样期间 posture 已被其他程序修改，请重试")

    mode = stat.S_IMODE(posture.stat().st_mode)
    # A unique byte-for-byte backup precedes an atomic replacement in the same directory.
    with tempfile.NamedTemporaryFile(dir=posture.parent, prefix=posture.name + ".",
                                     suffix=".bak", delete=False) as backup:
        backup.write(original)
        backup.flush()
        os.fsync(backup.fileno())
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=posture.parent, prefix="." + posture.name + ".",
                                         suffix=".tmp", delete=False) as output:
            temporary = Path(output.name)
            os.fchmod(output.fileno(), mode)
            output.write(updated)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, posture)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return Path(backup.name), angles


def capture(robot_config):
    posture, original, _ = read_posture(robot_config)
    helper = state_helper()
    print("读取双臂当前关节角；请先停止控制程序及拖动示教。SDK 断开连接会停止机械臂运动。",
          file=sys.stderr, flush=True)
    # The vendor SDK creates ./logs on connect, even in installed/read-only trees.
    with tempfile.TemporaryDirectory(prefix="aviator-home-read-") as work:
        (Path(work) / "logs").mkdir()
        completed = subprocess.run(
            [str(helper), "--robot-config", str(robot_config), "--read-robot",
             "--samples", "2", "--interval-ms", "50"], cwd=work,
            stdout=subprocess.PIPE, text=True, timeout=30, check=True)
    samples = [json.loads(line) for line in completed.stdout.splitlines() if line.strip()]
    if len(samples) != 2:
        raise ValueError("未收到完整的双臂采样，配置未修改")
    for sample in samples:
        home_angles(sample)
    backup, angles = save_home(posture, original, samples[-1])
    print(f"已更新: {posture}\n原文件备份: {backup}")
    for key, values in angles.items():
        print(f"{key}: {json.dumps(values)}")
    print("重新启动控制程序后生效。")


def main(argv=None):
    parser = argparse.ArgumentParser(description="读取机械臂当前关节角并直接写入双臂 home（度）。",
                                     epilog="需停止控制程序及拖动示教；读取超时为 30 秒。")
    parser.add_argument("--robot-config", type=Path, default=ROOT / "config/robot.yaml",
                        help="机器人配置，默认使用脚本所在源码/部署目录的 config/robot.yaml")
    parser.add_argument("--ui", action="store_true", help="打开双臂拖动与归位角记录界面")
    parser.add_argument("--dry-run", action="store_true", help="配合 --ui 演示操作，不连接设备、不写配置")
    args = parser.parse_args(argv)
    if args.dry_run and not args.ui:
        parser.error("--dry-run 需与 --ui 一起使用")
    try:
        # Share the normal launchers' lock; no robot process is started or stopped here.
        lock = Path(os.environ.get("XDG_RUNTIME_DIR") or "/tmp") / f"aviator-start-{os.getuid()}.lock"
        with lock.open("a") as guard:
            try:
                fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("控制程序或其他采集脚本正在运行，请先停止后重试") from None
            robot_config = args.robot_config.resolve(strict=True)
            if args.ui:
                from get_home_ui import run
                run(robot_config, args.dry_run)
            else:
                capture(robot_config)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, ImportError) as error:
        print(f"get_home: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
