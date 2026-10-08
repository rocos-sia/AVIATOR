#!/usr/bin/env bash
# 相机方向盘随动测试；独立于 start_aviator.sh。
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
DRY_RUN=false
FAKE_HAND=false
LOGGER=false
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --fake-hand) FAKE_HAND=true ;;
        --logger) LOGGER=true ;;
        -h|--help)
            echo '用法: ./scripts/start_camera_servo.sh [--fake-hand] [--logger] [--dry-run]'
            echo '启动真实机械臂、相机、手节点、Monitor 和相机随动测试 Core；无需 Gateway。'
            echo 'Core 自动使能、接近并锁定把手，随后在关节阻抗模式下随动。'
            echo '--fake-hand 使用模拟双手；--logger 开启 MCAP 与相机图像记录。'
            echo '默认二进制目录 build/release/bin；Ctrl+C 先停止 Core，再停止其他节点。'
            echo '环境变量: AVIATOR_BIN, CONDA_ROOT, HAND_PYTHON, CAMERA_PYTHON, MONITOR_BIN,'
            echo '          CAMERA_TIMEOUT_MS (默认200), START_DELAY (默认2秒)'
            exit 0 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done

AVIATOR_BIN=${AVIATOR_BIN:-$ROOT/build/release/bin}
CONDA_ROOT=${CONDA_ROOT:-$HOME/miniconda3}
HAND_PYTHON=${HAND_PYTHON:-$CONDA_ROOT/envs/rh56-pendant/bin/python3}
CAMERA_PYTHON=${CAMERA_PYTHON:-$CONDA_ROOT/envs/apriltag_realsense/bin/python}
MONITOR_BIN=${MONITOR_BIN:-$AVIATOR_BIN/aviator_monitor}
START_DELAY=${START_DELAY:-2}
CAMERA_TIMEOUT_MS=${CAMERA_TIMEOUT_MS:-200}
[[ $START_DELAY =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo 'START_DELAY 必须为非负秒数' >&2; exit 2; }
[[ $CAMERA_TIMEOUT_MS =~ ^[0-9]{1,4}$ ]] || { echo 'CAMERA_TIMEOUT_MS 必须为 20..1000 的整数' >&2; exit 2; }
CAMERA_TIMEOUT_MS=$((10#$CAMERA_TIMEOUT_MS))
(( CAMERA_TIMEOUT_MS >= 20 && CAMERA_TIMEOUT_MS <= 1000 )) || {
    echo 'CAMERA_TIMEOUT_MS 必须为 20..1000 的整数' >&2; exit 2;
}
SYSTEM_CONFIG=config/system.yaml
HAND_NODE=nodes/rh56ftp_hand/rh56ftp_node.py
if $FAKE_HAND; then HAND_NODE=nodes/rh56ftp_hand/fake_rh56ftp_hand.py; fi
recording_options=()
if $LOGGER; then recording_options+=(--recording-config config/recording.yaml); fi

PIDS=()
NAMES=()
ROOT_PIDS=()
CORE_PID=
# 与现有启动脚本一致：跨 setsid 会话直接认证，不依赖 sudo 缓存。
SUDO_PASSWORD='a'

cleanup() {
    trap - EXIT INT TERM
    set +e
    echo '正在停止相机随动测试…'
    # Keep bus, hand and manipulator alive while Core brakes, unlocks and disables.
    if [[ -n $CORE_PID ]] && kill -0 "$CORE_PID" 2>/dev/null; then
        kill -TERM -- "-$CORE_PID" 2>/dev/null
        for ((attempt=0; attempt<150; attempt++)); do
            kill -0 "$CORE_PID" 2>/dev/null || break
            sleep .1
        done
        if kill -0 "$CORE_PID" 2>/dev/null; then
            echo "Core 15秒内未退出，强制停止；请查看 $LOG_DIR/core.log" >&2
            kill -KILL -- "-$CORE_PID" 2>/dev/null
        fi
    fi
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        [[ ${PIDS[i]} == "$CORE_PID" ]] && continue
        if [[ ${ROOT_PIDS[i]} == true ]]; then
            sudo -S -p '' kill -TERM -- "-${PIDS[i]}" <<< "$SUDO_PASSWORD" 2>/dev/null
        else
            kill -TERM -- "-${PIDS[i]}" 2>/dev/null
        fi
    done
    sleep 2
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        if [[ ${ROOT_PIDS[i]} == true ]]; then
            sudo -S -p '' kill -KILL -- "-${PIDS[i]}" <<< "$SUDO_PASSWORD" 2>/dev/null
        else
            kill -KILL -- "-${PIDS[i]}" 2>/dev/null
        fi
    done
    # The browser may stay open; only reap the node processes owned by this run.
    for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null; done
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
    if [[ $name == core ]]; then CORE_PID=${PIDS[-1]}; fi
    sleep "$START_DELAY"
    check_nodes
}

if ! $DRY_RUN; then
    [[ $EUID != 0 ]] || { echo '请以普通用户执行，脚本仅对 manipulator 使用 sudo。' >&2; exit 1; }
    for command in setsid flock sudo google-chrome-stable curl; do
        command -v "$command" >/dev/null || { echo "缺少命令: $command" >&2; exit 1; }
    done
    executables=("$AVIATOR_BIN/aviator_bus" "$AVIATOR_BIN/manipulator"
        "$AVIATOR_BIN/aviator_core_camera_servo" "$HAND_PYTHON" "$CAMERA_PYTHON" "$MONITOR_BIN")
    files=("$SYSTEM_CONFIG" config/robot.yaml config/camera.yaml config/monitor.yaml
        config/rh56ftp_hand.yaml "$HAND_NODE" nodes/rh56ftp_hand/rh56ftp_node.py nodes/camera/main.py)
    if $LOGGER; then
        executables+=("$AVIATOR_BIN/aviator_logger")
        files+=(config/recording.yaml)
    fi
    for file in "${executables[@]}"; do
        [[ -x $file ]] || { echo "可执行文件不存在: $file" >&2; exit 1; }
    done
    for file in "${files[@]}"; do
        [[ -r $file ]] || { echo "文件不可读: $file" >&2; exit 1; }
    done
    # Share the normal launcher's lock to exclude concurrent Core owners.
    exec 9>"${XDG_RUNTIME_DIR:-/tmp}/aviator-start-${UID}.lock"
    flock -n 9 || { echo 'AVIATOR 启动脚本已在运行，请先停止。' >&2; exit 1; }
    sudo -S -p '' -v <<< "$SUDO_PASSWORD"
    LOG_DIR=$(mktemp -d "${TMPDIR:-/tmp}/aviator-camera-servo.XXXXXXXX")
    echo "日志目录: $LOG_DIR"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    # A new marker prevents camera sequence resets from looking like old frames.
    cat /proc/sys/kernel/random/uuid > "$LOG_DIR/session.uuid"
    SESSION=$(cat "$LOG_DIR/session.uuid")
else
    LOG_DIR='<本次运行日志目录>'
    SESSION='<本次生成的新 UUID>'
fi

start bus "$AVIATOR_BIN/aviator_bus" --config "$SYSTEM_CONFIG"
start manipulator sudo -S -p '' "$AVIATOR_BIN/manipulator" --config "$SYSTEM_CONFIG"
start rh56ftp env PATH="$(dirname "$HAND_PYTHON"):$PATH" "$HAND_PYTHON" \
    "$HAND_NODE" --config config/rh56ftp_hand.yaml
if $LOGGER; then
    if $DRY_RUN; then
        RECORDING_FILE="$ROOT/logs/camera_servo_<北京时间>.mcap"
    else
        mkdir -p "$ROOT/logs"
        RECORDING_FILE="$ROOT/logs/camera_servo_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S_%N).mcap"
    fi
    start logger "$AVIATOR_BIN/aviator_logger" --config config/recording.yaml --output "$RECORDING_FILE"
fi
start camera env PATH="$(dirname "$CAMERA_PYTHON"):$PATH" "$CAMERA_PYTHON" \
    nodes/camera/main.py --config config/camera.yaml "${recording_options[@]}" \
    --camera-id cockpit --session "$SESSION" --show --print-pose
start monitor "$MONITOR_BIN" --config config/monitor.yaml --log-dir "$LOG_DIR"
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
if ! $DRY_RUN; then
    google-chrome-stable --start-fullscreen http://127.0.0.1:8081 >"$LOG_DIR/chrome.log" 2>&1 < /dev/null 9>&- &
fi
# Start the motion owner last, once its devices and observation UI are running.
start core "$AVIATOR_BIN/aviator_core_camera_servo" --config "$SYSTEM_CONFIG" \
    --camera-id cockpit --camera-timeout-ms "$CAMERA_TIMEOUT_MS"
$DRY_RUN && exit 0
echo '相机随动测试已启动；Core 自动接近并锁定把手。Ctrl+C 停止并下使能。'
while true; do
    check_nodes
    sleep 1
done
