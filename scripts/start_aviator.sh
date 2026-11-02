#!/usr/bin/env bash
# 按 docs/启动流程.txt 启动；Ctrl+C 停止本次启动的节点。
set -Eeuo pipefail
if [[ -f $(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh ]]; then
    source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
fi

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
DRY_RUN=false
SIMULATION=false
RS422=false
FAKE_HAND=false
HEADLESS=false
NO_BROWSER=false
LOGGER=false
TTS=false
WHEEL_TEST=false
REMOTE_UI=${AVIATOR_REMOTE_UI:-0}
REMOTE_KEYBOARD=${AVIATOR_REMOTE_KEYBOARD:-0}
[[ $REMOTE_UI == 0 || $REMOTE_UI == 1 ]] || { echo 'AVIATOR_REMOTE_UI 必须为 0 或 1' >&2; exit 2; }
[[ $REMOTE_KEYBOARD == 0 || $REMOTE_KEYBOARD == 1 ]] || { echo 'AVIATOR_REMOTE_KEYBOARD 必须为 0 或 1' >&2; exit 2; }
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --simulation) SIMULATION=true ;;
        --rs422) RS422=true ;;
        --fake-hand) FAKE_HAND=true ;;
        --headless) HEADLESS=true ;;
        --no-browser) NO_BROWSER=true ;;
        --logger) LOGGER=true ;;
        --tts) TTS=true ;;
        --wheel-test) WHEEL_TEST=true ;;
        -h|--help)
            echo '用法: ./scripts/start_aviator.sh [--simulation | --fake-hand] [--headless] [--no-browser] [--rs422 | --wheel-test] [--logger] [--tts] [--dry-run]'
            echo '默认启动 Rokae + RH56FTP，加载 config/system.yaml。'
            echo '真机先启动 Bus 和 X1 光电节点，再启动设备和 Core；Core 收到有效 1 后才允许初始化。'
            echo '--simulation 使用单个仿真进程替代设备节点，加载 config/system-simulation.yaml。'
            echo '--no-browser 不自动打开浏览器；Monitor 仍可通过 http://127.0.0.1:8081 访问。'
            echo '--rs422 选择双路串口配置，Core 输入来源为 FLIGHT；先编辑 flight-rs422.yaml。'
            echo '--fake-hand 仅将 RH56FTP 替换为模拟双手，机械臂照常启动。'
            echo '真机仅在 grasp_wheel.source=camera 时启动相机；wheel_initial 或省略时跳过相机。'
            echo '--wheel-test 打开轮盘五点精度测试 UI，独占网关输入；UI_PYTHON 可指定含 tkinter 的 Python。'
            echo '默认不开启 Logger；--logger 开启 MCAP 记录及相机图像录制通道。'
            echo '--tts 开启离线语音提醒；先生成音频包。TTS 退出不停止控制节点。'
            echo '环境变量: AVIATOR_BIN, CONDA_ROOT, CAMERA_PYTHON, HAND_PYTHON, MONITOR_BIN, TTS_CONFIG, PHOTOELECTRIC_CONFIG, START_DELAY, AVIATOR_RECORDING_DIR, DEVICE_READY_TIMEOUT'
            echo 'DEVICE_READY_TIMEOUT: 等待设备服务 READY 的秒数（默认60，范围1..3600）。'
            echo 'AVIATOR_REMOTE_UI=1：远程界面模式，不打开工控机相机窗口和浏览器。'
            echo 'AVIATOR_REMOTE_KEYBOARD=1：远程模式下将原网关键盘窗口通过 SSH X11 显示到本机，与 --rs422 互斥。'
            exit 0 ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done
$FAKE_HAND && $SIMULATION && { echo '--fake-hand 与 --simulation 不能同时使用' >&2; exit 2; }
$HEADLESS && ! $SIMULATION && { echo '--headless 仅适用于 --simulation' >&2; exit 2; }
$WHEEL_TEST && $RS422 && { echo '--wheel-test 与 --rs422 不能同时使用' >&2; exit 2; }
if [[ $REMOTE_KEYBOARD == 1 ]]; then
    [[ $REMOTE_UI == 1 ]] || { echo '本机键盘控制需要 AVIATOR_REMOTE_UI=1' >&2; exit 2; }
    ! $RS422 || { echo '本机键盘控制与 --rs422 不能同时使用' >&2; exit 2; }
    ! $WHEEL_TEST || { echo '本机键盘控制与 --wheel-test 不能同时使用' >&2; exit 2; }
    $DRY_RUN || [[ -n ${DISPLAY:-} ]] || { echo '本机键盘控制缺少 SSH X11 DISPLAY，未启动设备。' >&2; exit 2; }
fi
SYSTEM_CONFIG=config/system.yaml
if $SIMULATION; then
    SYSTEM_CONFIG=config/system-simulation.yaml
fi
FLIGHT_CONFIG=config/flight.yaml
if $RS422; then
    FLIGHT_CONFIG=config/flight-rs422.yaml
    SYSTEM_CONFIG=config/system-rs422.yaml
    $SIMULATION && SYSTEM_CONFIG=config/system-simulation-rs422.yaml
fi
if [[ -f $ROOT/../../release-version.json ]]; then
    AVIATOR_BIN=${AVIATOR_BIN:-$ROOT/../../bin}
else
    AVIATOR_BIN=${AVIATOR_BIN:-$ROOT/build/bin}
fi
recording_options=()
if $LOGGER; then
    recording_options+=(--recording-config config/recording.yaml)
fi

CONDA_ROOT=${CONDA_ROOT:-$HOME/miniconda3}
HAND_PYTHON=${HAND_PYTHON:-$CONDA_ROOT/envs/rh56-pendant/bin/python}
HAND_LAUNCHER="$ROOT/nodes/rh56ftp_hand/launch.py"
[[ ! -d $ROOT/rh56ftp_hand ]] || HAND_LAUNCHER="$ROOT/rh56ftp_hand/launch.py"
hand_options=()
$FAKE_HAND && hand_options+=(--fake-hand)
CAMERA_PYTHON=${CAMERA_PYTHON:-$CONDA_ROOT/envs/apriltag_realsense/bin/python}
UI_PYTHON=${UI_PYTHON:-/usr/bin/python3}
MONITOR_BIN=${MONITOR_BIN:-$AVIATOR_BIN/aviator_monitor}
TTS_CONFIG=${TTS_CONFIG:-$ROOT/config/tts.yaml}
PHOTOELECTRIC_CONFIG=${PHOTOELECTRIC_CONFIG:-$ROOT/config/photoelectric.yaml}
# Resolve the same robot file as Core. Parse YAML before starting any devices;
# malformed settings must not silently disable camera acquisition.
START_CAMERA=false
ROBOT_CONFIG=config/robot.yaml
if ! $SIMULATION; then
    camera_settings=$(/usr/bin/python3 - "$SYSTEM_CONFIG" <<'PY'
import pathlib
import sys
try:
    import yaml
    system_path = pathlib.Path(sys.argv[1])
    system = yaml.safe_load(system_path.read_text())
    robot_path = pathlib.Path(system['robot'])
    if not robot_path.is_absolute():
        robot_path = system_path.parent / robot_path
    robot = yaml.safe_load(robot_path.read_text())
    grasp = robot.get('grasp_wheel', {})
    if not isinstance(grasp, dict):
        raise ValueError('grasp_wheel must be a map')
    source = grasp.get('source', 'wheel_initial')
    if source not in ('wheel_initial', 'camera'):
        raise ValueError('grasp_wheel.source must be wheel_initial or camera')
    print(source)
    print(robot_path)
except Exception as error:
    sys.exit(f'无法读取抓握相机配置，未启动设备: {error}')
PY
    ) || exit 2
    GRASP_WHEEL_SOURCE=${camera_settings%%$'\n'*}
    ROBOT_CONFIG=${camera_settings#*$'\n'}
    if [[ $GRASP_WHEEL_SOURCE == camera ]]; then
        START_CAMERA=true
    else
        echo "grasp_wheel.source=$GRASP_WHEEL_SOURCE：跳过真实相机节点及预热。"
    fi
fi
START_DELAY=${START_DELAY:-2}
AVIATOR_RECORDING_DIR=${AVIATOR_RECORDING_DIR:-$ROOT/logs}
[[ $AVIATOR_RECORDING_DIR == /* ]] || { echo 'AVIATOR_RECORDING_DIR 必须为绝对目录' >&2; exit 2; }
[[ $START_DELAY =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo 'START_DELAY 必须为非负秒数' >&2; exit 2; }
DEVICE_READY_TIMEOUT=${DEVICE_READY_TIMEOUT:-60}
[[ $DEVICE_READY_TIMEOUT =~ ^[1-9][0-9]{0,3}$ ]] && ((DEVICE_READY_TIMEOUT <= 3600)) || {
    echo 'DEVICE_READY_TIMEOUT 必须为 1..3600 的整数秒数' >&2; exit 2;
}
SESSION_FILE=/tmp/aviator_session.uuid
PIDS=()
NAMES=()
ROOT_PIDS=()
CORE_PID=
HAND_PID=
OPTIONAL_EXIT_REPORTED=()
# 每次 sudo 调用直接认证，避免依赖跨 setsid 会话的认证缓存。
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
    echo '正在停止本次启动的节点…'
    # Keep arm/hand, photoelectric and bus alive while Core releases and disables.
    if [[ -n $CORE_PID ]] && kill -0 -- "-$CORE_PID" 2>/dev/null; then
        kill -TERM -- "-$CORE_PID" 2>/dev/null || true
        for ((attempt=0; attempt<150; attempt++)); do
            kill -0 -- "-$CORE_PID" 2>/dev/null || break
            sleep .1
        done
        if kill -0 -- "-$CORE_PID" 2>/dev/null; then
            echo 'Core 15秒内未退出，强制停止；请查看 core.log' >&2
            kill -KILL -- "-$CORE_PID" 2>/dev/null || true
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
            if [[ ${NAMES[i]} == wheel_test_ui ]] && wait "${PIDS[i]}"; then
                echo '测试窗口已关闭，正在有序停止本次启动。'
                exit 0
            fi
            if [[ ${NAMES[i]} == tts ]]; then
                if [[ ${OPTIONAL_EXIT_REPORTED[i]:-false} != true ]]; then
                    echo "TTS 已退出，控制节点继续运行；请查看 $LOG_DIR/tts.log" >&2
                    OPTIONAL_EXIT_REPORTED[i]=true
                fi
                continue
            fi
            echo "${NAMES[i]} 已退出，请查看 $LOG_DIR/${NAMES[i]}.log" >&2
            tail -n 30 "$LOG_DIR/${NAMES[i]}.log" >&2 || true
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

# runDeviceServer emits this marker after device initialization and service bind.
# Read only this launch's log; process liveness alone does not mean SDK readiness.
wait_device_ready() {
    local name=$1 deadline=$((SECONDS + DEVICE_READY_TIMEOUT))
    echo "等待 $name 设备服务 READY（最多 ${DEVICE_READY_TIMEOUT} 秒）…"
    $DRY_RUN && return 0
    while true; do
        check_nodes
        if awk -v name="$name" '
            index($0, "READY " name " session=") && index($0, " service=") { ready=1 }
            END { exit !ready }
        ' "$LOG_DIR/$name.log"; then
            echo "$name 设备服务已就绪。"
            return 0
        fi
        if ((SECONDS >= deadline)); then
            echo "$name 在 ${DEVICE_READY_TIMEOUT} 秒内未报告 READY；设备初始化或 SDK 通信尚未完成，未启动 Core。请查看 $LOG_DIR/$name.log" >&2
            tail -n 30 "$LOG_DIR/$name.log" >&2 || true
            exit 1
        fi
        sleep .1
    done
}

# The monitor browser is optional and is not a control-node dependency.
browser_command=()
if ! $NO_BROWSER && ! $WHEEL_TEST && [[ $REMOTE_UI == 0 ]]; then
    for browser in google-chrome-stable google-chrome chromium chromium-browser firefox xdg-open; do
        if command -v "$browser" >/dev/null; then
            browser_command=("$browser")
            case "$browser" in
                google-chrome*|chromium*) browser_command+=(--start-fullscreen) ;;
                firefox) browser_command+=(--new-window) ;;
            esac
            browser_command+=(http://127.0.0.1:8081)
            break
        fi
    done
    if ((${#browser_command[@]} == 0)); then
        echo '未找到可用浏览器，继续启动；请手动访问 Monitor: http://127.0.0.1:8081' >&2
    fi
fi

if ! $DRY_RUN; then
    [[ $EUID != 0 ]] || { echo '请以普通用户执行，脚本仅对 manipulator 使用 sudo。' >&2; exit 1; }
    commands=(setsid flock ps awk tail curl)
    executables=("$AVIATOR_BIN/aviator_bus" "$AVIATOR_BIN/flight_gateway"
        "$AVIATOR_BIN/aviator_core_managed" "$MONITOR_BIN")
    files=("$SYSTEM_CONFIG" "$FLIGHT_CONFIG" "$ROBOT_CONFIG")
    if $WHEEL_TEST; then
        executables+=("$UI_PYTHON")
        files+=(tools/wheel_accuracy/ui.py)
    fi
    if $TTS; then
        executables+=("$AVIATOR_BIN/aviator_tts")
        files+=("$TTS_CONFIG")
        commands+=(aplay)
    fi
    if $LOGGER; then
        executables+=("$AVIATOR_BIN/aviator_logger")
        files+=(config/recording.yaml)
    fi
    if $SIMULATION; then
        executables+=("$AVIATOR_BIN/simulation")
        files+=(config/camera.yaml)
    else
        commands+=(sudo)
        executables+=("$AVIATOR_BIN/aviator_photoelectric" "$AVIATOR_BIN/manipulator" "$HAND_PYTHON")
        files+=("$PHOTOELECTRIC_CONFIG" config/rh56ftp_hand.yaml "$HAND_LAUNCHER")
    fi
    if $START_CAMERA; then
        executables+=("$CAMERA_PYTHON")
        files+=(config/camera.yaml nodes/camera/main.py)
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
    if ! $SIMULATION; then
        "$HAND_PYTHON" "$HAND_LAUNCHER" --system-config "$SYSTEM_CONFIG" "${hand_options[@]}" --check-config
    fi
    if $WHEEL_TEST; then
        "$AVIATOR_BIN/flight_gateway" --help 2>&1 | awk '/--test-ui-socket/ { found=1 } END { exit !found }' || {
            echo '请先编译新网关: cmake --build build --target flight_gateway -j4' >&2; exit 1;
        }
        "$UI_PYTHON" -c 'import tkinter as tk; w=tk.Tk(); w.withdraw(); w.destroy()' || {
            echo '测试 UI 需要可用的桌面显示和 tkinter；可设置 UI_PYTHON。' >&2; exit 1;
        }
    fi
    # Validate recording storage before starting any node (including hardware).
    if $LOGGER; then
        mkdir -p -- "$AVIATOR_RECORDING_DIR"
        recording_probe=$(mktemp "$AVIATOR_RECORDING_DIR/.aviator-write-check.XXXXXXXX") || {
            echo "录制目录不可写: $AVIATOR_RECORDING_DIR" >&2; exit 1;
        }
        rm -- "$recording_probe"
    fi
    # Fail preparation before starting hardware; runtime audio failure is optional.
    if $TTS; then "$AVIATOR_BIN/aviator_tts" --config "$TTS_CONFIG" --check; fi
    exec 9>"${XDG_RUNTIME_DIR:-/tmp}/aviator-start-${UID}.lock"
    flock -n 9 || { echo "启动脚本或上次残留节点仍持有启动锁，请先停止。锁文件: ${XDG_RUNTIME_DIR:-/tmp}/aviator-start-${UID}.lock（不要删除锁文件绕过检查）" >&2; exit 1; }
    # Validate without opening the serial port, before any device process starts.
    if ! $SIMULATION; then
        "$AVIATOR_BIN/aviator_photoelectric" --config "$PHOTOELECTRIC_CONFIG" --check-config
    fi
    if ! $SIMULATION; then
        sudo -S -p '' -v <<< "$SUDO_PASSWORD"
    fi
    LOG_DIR=$(mktemp -d "${TMPDIR:-/tmp}/aviator-start.XXXXXXXX")
    echo "日志目录: $LOG_DIR"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
    if [[ ! -s $SESSION_FILE ]]; then
        cat /proc/sys/kernel/random/uuid > "$SESSION_FILE"
    fi
    SESSION=$(cat "$SESSION_FILE")
else
    SESSION='<读取 /tmp/aviator_session.uuid；不存在时生成 UUID>'
    LOG_DIR='<本次运行日志目录>'
fi

start bus "$AVIATOR_BIN/aviator_bus" --config "$SYSTEM_CONFIG"
if ! $SIMULATION; then
    start photoelectric "$AVIATOR_BIN/aviator_photoelectric" --config "$PHOTOELECTRIC_CONFIG"
    echo 'Core 初始化条件：收到有效、新鲜的 X1=1。'
fi
if $SIMULATION; then
    simulation_options=()
    $HEADLESS && simulation_options+=(--headless)
    start simulation "$AVIATOR_BIN/simulation" --config "$SYSTEM_CONFIG" \
        --camera-config config/camera.yaml "${recording_options[@]}" \
        --camera-id cockpit "${simulation_options[@]}"
    wait_device_ready simulation
else
    start manipulator sudo -S -p '' "$AVIATOR_BIN/manipulator" --config "$SYSTEM_CONFIG"
    wait_device_ready manipulator
    start hand "$HAND_PYTHON" "$HAND_LAUNCHER" --system-config "$SYSTEM_CONFIG" "${hand_options[@]}"
fi
gateway_options=()
if $WHEEL_TEST; then gateway_options+=(--test-ui-socket "$LOG_DIR/wheel-test.sock"); fi
gateway_command=("$AVIATOR_BIN/flight_gateway")
if [[ $REMOTE_UI == 1 ]]; then
    gateway_video_driver=dummy
    [[ $REMOTE_KEYBOARD == 1 ]] && gateway_video_driver=x11
    gateway_command=(env SDL_VIDEODRIVER="$gateway_video_driver" "${gateway_command[@]}")
fi
start gateway "${gateway_command[@]}" --config "$FLIGHT_CONFIG" "${gateway_options[@]}"
if $WHEEL_TEST; then
    wheel_test_mode='真机 + 真实双手'
    if $FAKE_HAND; then wheel_test_mode='真机 + fake-hand（模拟双手）'; fi
    if $SIMULATION; then wheel_test_mode='MuJoCo 仿真'; fi
    start wheel_test_ui "$UI_PYTHON" "$ROOT/tools/wheel_accuracy/ui.py" \
        --socket "$LOG_DIR/wheel-test.sock" --mode "$wheel_test_mode"
fi
if $TTS; then
    start tts "$AVIATOR_BIN/aviator_tts" --config "$TTS_CONFIG" --system-config "$SYSTEM_CONFIG"
fi
if $START_CAMERA; then
    camera_view_options=(--show --print-pose)
    if [[ $REMOTE_UI == 1 ]]; then camera_view_options=(--no-show --print-pose); fi
    # Use the dedicated warmup below instead of the normal inter-node delay.
    START_DELAY=0 start camera env PATH="$(dirname "$CAMERA_PYTHON"):$PATH" "$CAMERA_PYTHON" \
        nodes/camera/main.py --config config/camera.yaml "${recording_options[@]}" \
        --camera-id cockpit --session "$SESSION" "${camera_view_options[@]}"
    echo '等待相机预热 5 秒后启动 Core…'
    if ! $DRY_RUN; then
        sleep 5
        check_nodes
    fi
fi
start core "$AVIATOR_BIN/aviator_core_managed" --config "$SYSTEM_CONFIG"
if $SIMULATION; then
    if $DRY_RUN; then
        SIM_MONITOR_CONFIG='<日志目录>/monitor-simulation.yaml'
    else
        SIM_MONITOR_CONFIG="$LOG_DIR/monitor-simulation.yaml"
        sed -e 's/camera.detection: "camera"/camera.detection: "simulation"/' \
            -e 's/publisher_id: "camera"/publisher_id: "simulation"/' config/monitor.yaml > "$SIM_MONITOR_CONFIG"
    fi
    start monitor "$MONITOR_BIN" --config "$SIM_MONITOR_CONFIG" --log-dir "$LOG_DIR"
else
    start monitor "$MONITOR_BIN" --log-dir "$LOG_DIR"
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
if ((${#browser_command[@]} > 0)); then
    printf '启动浏览器:'
    printf ' %q' "${browser_command[@]}"
    printf '\n'
    # 浏览器可能复用已有进程，不将其纳入节点退出监控，也不持有启动锁。
    if ! $DRY_RUN; then
        (
            "${browser_command[@]}" || echo '浏览器启动失败，请手动访问 Monitor: http://127.0.0.1:8081' >&2
        ) >"$LOG_DIR/browser.log" 2>&1 < /dev/null 9>&- &
    fi
fi
if $LOGGER; then
    RECORDING_FILE="$AVIATOR_RECORDING_DIR/recording_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S_%N).mcap"
    logger_output_options=()
    # Keep this remote run's data and images together for transfer; leave site config intact.
    if [[ $REMOTE_UI == 1 ]]; then logger_output_options+=(--image-output "${RECORDING_FILE%.mcap}.images.mcap"); fi
    start logger "$AVIATOR_BIN/aviator_logger" --config config/recording.yaml --output "$RECORDING_FILE" "${logger_output_options[@]}"
fi
$DRY_RUN && exit 0
echo '所有节点已启动；保持此终端打开，按 Ctrl+C 停止。'
while true; do
    check_nodes
    sleep 1
done
