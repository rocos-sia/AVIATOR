#!/usr/bin/env bash
# Reuse production device startup, readiness checks, locks and ordered shutdown.
set -Eeuo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
for arg in "$@"; do
    case "$arg" in
        -h|--help)
            echo '用法: ./scripts/start_wheel_accuracy_ui.sh [--fake-hand | --simulation] [--headless] [--logger] [--tts] [--dry-run]'
            echo '打开轮盘五点运动精度 UI；Home / 抓握 / 松手复用 Managed 完整流程。'
            echo '--fake-hand 仅模拟双手，机械臂仍真实运动。--simulation 使用 MuJoCo。'
            echo '首次使用先编译: cmake --build build --target flight_gateway -j4'
            echo 'UI_PYTHON 默认为 /usr/bin/python3，需要 tkinter 和可用的桌面显示。'
            echo '关闭窗口或 Ctrl+C 会按 start_aviator.sh 的顺序停止本次启动的节点。'
            exit 0 ;;
        --fake-hand|--simulation|--headless|--logger|--tts|--dry-run) ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done
exec "$ROOT/scripts/start_aviator.sh" --wheel-test "$@"
