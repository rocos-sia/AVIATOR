#!/usr/bin/env bash
# 相机方向盘随动测试；独立于 start_aviator.sh。
set -Eeuo pipefail
if [[ -f $(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh ]]; then
    source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
fi

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
            echo '默认二进制目录：源码运行 build/release/bin，安装运行使用安装目录 bin。'
            echo 'Ctrl+C 先停止 Core，再停止其他节点。'
            echo '环境变量: AVIATOR_BIN, CONDA_ROOT, CAMERA_PYTHON, MONITOR_BIN,'
            echo '          CAMERA_TIMEOUT_MS (默认200), START_DELAY (默认2秒)'
            exit 0 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done

if [[ -f $ROOT/../../release-version.json ]]; then
    AVIATOR_BIN=${AVIATOR_BIN:-$ROOT/../../bin}
else
    AVIATOR_BIN=${AVIATOR_BIN:-$ROOT/build/release/bin}
fi
hand_options=()
$FAKE_HAND && hand_options+=(--fake-hand)
CONDA_ROOT=${CONDA_ROOT:-$HOME/miniconda3}
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
recording_options=()
if $LOGGER; then recording_options+=(--recording-config config/recording.yaml); fi

PIDS=()
NAMES=()
ROOT_PIDS=()
CORE_PID=
HAND_PID=
# 与现有启动脚本一致：跨 setsid 会话直接认证，不依赖 sudo 缓存。
SUDO_PASSWORD='a'

# sudo may put the command in a separate session (for example with use_pty).
# Signal both the actual node group and the launcher group, even if their leaders exited.
signal_node() {
    local index=$1 signal=$2 node_group=
    local groups=("-${PIDS[index]}")
    if [[ ${ROOT_PIDS[index]} == true ]]; then
        if [[ -r $LOG_DIR/${NAMES[index]}.pid ]]; then
            read -r node_group < "$LOG_DIR/${NAMES[index]}.pid" || true
            if [[ $node_group =~ ^[0-9]+$ ]] && ((node_group > 1)); then
                groups=("-$node_group" "${groups[@]}")
            fi
        fi
        sudo -S -p '' kill "-$signal" -- "${groups[@]}" <<< "$SUDO_PASSWORD" 2>/dev/null || true
    else
        kill "-$signal" -- "${groups[@]}" 2>/dev/null || true
    fi
}

# Verify bounded cleanup without waiting forever on an uninterruptible SDK process.
reap_nodes() {
    local index node_group groups="${PIDS[*]}" remaining= pid state attempt
    for index in "${!PIDS[@]}"; do
        if [[ ${ROOT_PIDS[index]} == true && -r $LOG_DIR/${NAMES[index]}.pid ]]; then
            read -r node_group < "$LOG_DIR/${NAMES[index]}.pid" || true
            if [[ $node_group =~ ^[0-9]+$ ]] && ((node_group > 1)); then groups+=" $node_group"; fi
        fi
    done
    for ((attempt=0; attempt<20; attempt++)); do
        remaining=$(ps -eo pid=,pgid=,stat=,comm= | awk -v groups="$groups" '
            BEGIN { n=split(groups,a," "); for(i=1;i<=n;i++) owned[a[i]]=1 }
            ($2 in owned) && $3 !~ /^Z/ { print }
        ')
        [[ -n $remaining ]] || break
        sleep .1
    done
    if [[ -n $remaining ]]; then
        printf '以下本次启动的进程在 SIGKILL 后仍未退出（PID/PGID/状态/名称）：\n%s\n' "$remaining" >&2
    fi
    for pid in "${PIDS[@]}"; do
        state=$(ps -o stat= -p "$pid")
        case "$state" in ''|Z*) wait "$pid" 2>/dev/null || true ;; esac
    done
}

cleanup() {
    # Repeated Ctrl+C must not interrupt the ordered shutdown or its KILL fallback.
    trap '' INT TERM HUP
    trap - EXIT
    set +e
    echo '正在停止相机随动测试…'
    # Keep bus, hand and manipulator alive while Core brakes, unlocks and disables.
    if [[ -n $CORE_PID ]] && kill -0 -- "-$CORE_PID" 2>/dev/null; then
        kill -TERM -- "-$CORE_PID" 2>/dev/null
        for ((attempt=0; attempt<150; attempt++)); do
            kill -0 -- "-$CORE_PID" 2>/dev/null || break
            sleep .1
        done
        if kill -0 -- "-$CORE_PID" 2>/dev/null; then
            echo "Core 15秒内未退出，强制停止；请查看 $LOG_DIR/core.log" >&2
            kill -KILL -- "-$CORE_PID" 2>/dev/null
        fi
    fi
    # Finish hand shutdown writes before releasing the arm controller connection.
    if [[ -n $HAND_PID ]] && kill -0 -- "-$HAND_PID" 2>/dev/null; then
        kill -TERM -- "-$HAND_PID" 2>/dev/null || true
        for ((attempt=0; attempt<80; attempt++)); do
            kill -0 -- "-$HAND_PID" 2>/dev/null || break
            sleep .1
        done
        if kill -0 -- "-$HAND_PID" 2>/dev/null; then
            echo "手节点8秒内未退出，强制停止；请查看 $LOG_DIR/hand.log" >&2
            kill -KILL -- "-$HAND_PID" 2>/dev/null || true
        fi
    fi
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        [[ ${PIDS[i]} == "$CORE_PID" ]] && continue
        signal_node "$i" TERM
    done
    sleep 2
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        signal_node "$i" KILL
    done
    reap_nodes
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
        # The only elevated launch uses: sudo -S -p '' executable [arguments].
        # Create the node session *inside* sudo, record its leader, and keep sudo
        # waiting even when setsid must fork. Positional arguments preserve quoting.
        setsid "${@:1:4}" setsid --wait bash -c '
            printf "%s\n" "$$" > "$1"
            shift
            exec "$@"
        ' bash "$LOG_DIR/$name.pid" "${@:5}" >"$LOG_DIR/$name.log" 2>&1 <<< "$SUDO_PASSWORD" &
    else
        setsid "$@" >"$LOG_DIR/$name.log" 2>&1 < /dev/null &
    fi
    PIDS+=("$!")
    NAMES+=("$name")
    ROOT_PIDS+=("$elevated")
    if [[ $name == core ]]; then CORE_PID=${PIDS[-1]}; fi
    if [[ $name == hand ]]; then HAND_PID=${PIDS[-1]}; fi
    sleep "$START_DELAY"
    check_nodes
}

if ! $DRY_RUN; then
    [[ $EUID != 0 ]] || { echo '请以普通用户执行，脚本仅对 manipulator 使用 sudo。' >&2; exit 1; }
    for command in setsid flock ps awk sudo google-chrome-stable curl; do
        command -v "$command" >/dev/null || { echo "缺少命令: $command" >&2; exit 1; }
    done
    executables=("$AVIATOR_BIN/aviator_bus" "$AVIATOR_BIN/manipulator" "$AVIATOR_BIN/xCoreSDK_hand"
        "$AVIATOR_BIN/aviator_core_camera_servo" "$CAMERA_PYTHON" "$MONITOR_BIN")
    files=("$SYSTEM_CONFIG" config/robot.yaml config/camera.yaml config/monitor.yaml
        config/rh56ftp_hand.yaml nodes/camera/main.py)
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
    flock -n 9 || { echo "启动脚本或上次残留节点仍持有启动锁，请先停止。锁文件: ${XDG_RUNTIME_DIR:-/tmp}/aviator-start-${UID}.lock（不要删除锁文件绕过检查）" >&2; exit 1; }
    sudo -S -p '' -v <<< "$SUDO_PASSWORD"
    LOG_DIR=$(mktemp -d "${TMPDIR:-/tmp}/aviator-camera-servo.XXXXXXXX")
    echo "日志目录: $LOG_DIR"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
    # A new marker prevents camera sequence resets from looking like old frames.
    cat /proc/sys/kernel/random/uuid > "$LOG_DIR/session.uuid"
    SESSION=$(cat "$LOG_DIR/session.uuid")
else
    LOG_DIR='<本次运行日志目录>'
    SESSION='<本次生成的新 UUID>'
fi

start bus "$AVIATOR_BIN/aviator_bus" --config "$SYSTEM_CONFIG"
start manipulator sudo -S -p '' "$AVIATOR_BIN/manipulator" --config "$SYSTEM_CONFIG"
start hand "$AVIATOR_BIN/xCoreSDK_hand" --system-config "$SYSTEM_CONFIG" \
        --sdk-work-dir "$LOG_DIR/hand-sdk" "${hand_options[@]}"
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
