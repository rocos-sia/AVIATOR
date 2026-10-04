#!/usr/bin/env bash
# 按 docs/启动流程.txt 启动；Ctrl+C 停止本次启动的节点。
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
DRY_RUN=false
SIMULATION=false
HEADLESS=false
LOGGER=false
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --simulation) SIMULATION=true ;;
        --headless) HEADLESS=true ;;
        --logger) LOGGER=true ;;
        -h|--help)
            echo '用法: ./scripts/start_aviator.sh [--simulation] [--headless] [--logger] [--dry-run]'
            echo '默认启动 Rokae + RH56FTP；--simulation 使用单个仿真进程替代设备节点。'
            echo '默认不开启 Logger；--logger 开启 MCAP 记录及相机图像录制通道。'
            echo '环境变量: AVIATOR_BIN, CONDA_ROOT, HAND_PYTHON, CAMERA_PYTHON, MONITOR_BIN, START_DELAY'
            exit 0 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done
$HEADLESS && ! $SIMULATION && { echo '--headless 仅适用于 --simulation' >&2; exit 2; }
AVIATOR_BIN=${AVIATOR_BIN:-$ROOT/build/bin}
recording_options=()
if $LOGGER; then
    recording_options+=(--recording-config config/recording.yaml)
fi

CONDA_ROOT=${CONDA_ROOT:-$HOME/miniconda3}
HAND_PYTHON=${HAND_PYTHON:-$CONDA_ROOT/envs/rh56-pendant/bin/python3}
CAMERA_PYTHON=${CAMERA_PYTHON:-$CONDA_ROOT/envs/apriltag_realsense/bin/python}
MONITOR_BIN=${MONITOR_BIN:-$AVIATOR_BIN/aviator_monitor}
START_DELAY=${START_DELAY:-2}
[[ $START_DELAY =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo 'START_DELAY 必须为非负秒数' >&2; exit 2; }
SESSION_FILE=/tmp/aviator_session.uuid
PIDS=()
NAMES=()
ROOT_PIDS=()
# 每次 sudo 调用直接认证，避免依赖跨 setsid 会话的认证缓存。
SUDO_PASSWORD='a'

cleanup() {
    trap - EXIT INT TERM
    echo '正在停止本次启动的节点…'
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        if [[ ${ROOT_PIDS[i]} == true ]]; then
            sudo -S -p '' kill -TERM -- "-${PIDS[i]}" <<< "$SUDO_PASSWORD" 2>/dev/null || true
        else
            kill -TERM -- "-${PIDS[i]}" 2>/dev/null || true
        fi
    done
    sleep 2
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        if [[ ${ROOT_PIDS[i]} == true ]]; then
            sudo -S -p '' kill -KILL -- "-${PIDS[i]}" <<< "$SUDO_PASSWORD" 2>/dev/null || true
        else
            kill -KILL -- "-${PIDS[i]}" 2>/dev/null || true
        fi
    done
}

check_nodes() {
    local i
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[i]}" 2>/dev/null; then
            echo "${NAMES[i]} 已退出，请查看 $LOG_DIR/${NAMES[i]}.log" >&2
            exit 1
        fi
    done
}

start() {
    local name=$1 elevated=false
    shift
    printf '启动 %s:' "$name"
    printf ' %q' "$@"
    printf '\n'
    $DRY_RUN && return 0
    [[ $1 == sudo ]] && elevated=true
    if $elevated; then
        setsid "$@" >"$LOG_DIR/$name.log" 2>&1 <<< "$SUDO_PASSWORD" &
    else
        setsid "$@" >"$LOG_DIR/$name.log" 2>&1 < /dev/null &
    fi
    PIDS+=("$!")
    NAMES+=("$name")
    ROOT_PIDS+=("$elevated")
    sleep "$START_DELAY"
    check_nodes
}

if ! $DRY_RUN; then
    [[ $EUID != 0 ]] || { echo '请以普通用户执行，脚本仅对 manipulator 使用 sudo。' >&2; exit 1; }
    commands=(setsid flock google-chrome-stable curl)
    executables=("$AVIATOR_BIN/aviator_bus" "$AVIATOR_BIN/flight_gateway"
        "$AVIATOR_BIN/aviator_core_managed" "$MONITOR_BIN")
    files=(config/system.yaml config/robot.yaml config/camera.yaml)
    if $LOGGER; then
        executables+=("$AVIATOR_BIN/aviator_logger")
        files+=(config/recording.yaml)
    fi
    if $SIMULATION; then
        executables+=("$AVIATOR_BIN/simulation")
    else
        commands+=(sudo)
        executables+=("$AVIATOR_BIN/manipulator" "$HAND_PYTHON" "$CAMERA_PYTHON")
        files+=(config/camera.yaml nodes/rh56ftp_hand/rh56ftp_node.py nodes/camera/main.py)
    fi
    for command in "${commands[@]}"; do
        command -v "$command" >/dev/null || { echo "缺少命令: $command" >&2; exit 1; }
    done
    for file in "${executables[@]}"; do
        [[ -x $file ]] || { echo "可执行文件不存在: $file" >&2; exit 1; }
    done
    for file in "${files[@]}"; do
        [[ -r $file ]] || { echo "文件不可读: $file" >&2; exit 1; }
    done
    exec 9>"${XDG_RUNTIME_DIR:-/tmp}/aviator-start-${UID}.lock"
    flock -n 9 || { echo '一键启动脚本已在运行。' >&2; exit 1; }
    if ! $SIMULATION; then sudo -S -p '' -v <<< "$SUDO_PASSWORD"; fi
    LOG_DIR=$(mktemp -d "${TMPDIR:-/tmp}/aviator-start.XXXXXXXX")
    echo "日志目录: $LOG_DIR"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    if [[ ! -s $SESSION_FILE ]]; then
        cat /proc/sys/kernel/random/uuid > "$SESSION_FILE"
    fi
    SESSION=$(cat "$SESSION_FILE")
else
    SESSION='<读取 /tmp/aviator_session.uuid；不存在时生成 UUID>'
fi

start bus "$AVIATOR_BIN/aviator_bus" --config config/system.yaml
if $SIMULATION; then
    simulation_options=()
    $HEADLESS && simulation_options+=(--headless)
    start simulation "$AVIATOR_BIN/simulation" --config config/system.yaml \
        --camera-config config/camera.yaml "${recording_options[@]}" \
        --camera-id cockpit "${simulation_options[@]}"
else
    start manipulator sudo -S -p '' "$AVIATOR_BIN/manipulator" --config config/system.yaml
    start rh56ftp env PATH="$(dirname "$HAND_PYTHON"):$PATH" "$HAND_PYTHON" \
        nodes/rh56ftp_hand/rh56ftp_node.py --right-host 192.168.21.210 \
        --left-host 192.168.11.210 --speed 500 --force 500
fi
start gateway "$AVIATOR_BIN/flight_gateway"
start core "$AVIATOR_BIN/aviator_core_managed" --config config/system.yaml
if ! $SIMULATION; then
    start camera env PATH="$(dirname "$CAMERA_PYTHON"):$PATH" "$CAMERA_PYTHON" \
        nodes/camera/main.py --config config/camera.yaml "${recording_options[@]}" \
        --camera-id cockpit --session "$SESSION" --show --print-pose
fi
if $SIMULATION; then
    if $DRY_RUN; then
        SIM_MONITOR_CONFIG='<日志目录>/monitor-simulation.yaml'
    else
        SIM_MONITOR_CONFIG="$LOG_DIR/monitor-simulation.yaml"
        sed -e 's/camera.detection: "camera"/camera.detection: "simulation"/' \
            -e 's/publisher_id: "camera"/publisher_id: "simulation"/' config/monitor.yaml > "$SIM_MONITOR_CONFIG"
    fi
    start monitor "$MONITOR_BIN" --config "$SIM_MONITOR_CONFIG"
else
    start monitor "$MONITOR_BIN"
fi

if ! $DRY_RUN; then
    ready=false
    for ((attempt=0; attempt<30; attempt++)); do
        check_nodes
        if curl --silent --fail --max-time 1 http://127.0.0.1:8081/ >/dev/null; then
            ready=true
            break
        fi
        sleep 1
    done
    $ready || { echo 'Monitor 在 30 次检查后仍未就绪。' >&2; exit 1; }
fi
printf '启动 Chrome: google-chrome-stable --start-fullscreen http://127.0.0.1:8081\n'
# Chrome 可能复用已打开的浏览器，不将其纳入节点退出监控。
if ! $DRY_RUN; then
    google-chrome-stable --start-fullscreen http://127.0.0.1:8081 >"$LOG_DIR/chrome.log" 2>&1 < /dev/null 9>&- &
fi
if $LOGGER; then
    RECORDING_FILE="recording_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S_%N).mcap"
    start logger "$AVIATOR_BIN/aviator_logger" --config config/recording.yaml --output "$RECORDING_FILE"
fi
$DRY_RUN && exit 0
echo '所有节点已启动；保持此终端打开，按 Ctrl+C 停止。'
while true; do
    check_nodes
    sleep 1
done
