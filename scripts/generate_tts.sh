#!/usr/bin/env bash
# Generate the configured voice pack from local models; never download/install.
set -Eeuo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
CONFIG="$ROOT/config/tts.yaml"
MODEL_DIR="$ROOT/models/tts/matcha-icefall-zh-baker"
VOCODER="$ROOT/models/tts/vocos-22khz-univ.onnx"
MODEL_SOURCE='官方 Matcha Baker 模型，仅用于非商业开发测试；来源见 models/tts/sources.json'
OUTPUT=''
THREADS=2
SPEED=1.0
CHECK=false
DRY_RUN=false
CUSTOM_MODEL=false
CUSTOM_SOURCE=false

usage() {
    cat <<'HELP'
用法: scripts/generate_tts.sh [选项]
默认使用项目内 Matcha Baker + Vocos，输出目录由 config/tts.yaml 的 audio_dir 决定。
  --config PATH       提醒配置 YAML
  --output PATH       覆盖音频输出目录
  --threads N         CPU 推理线程数 1..4，默认 2
  --speed N           语速 0.5..2，默认 1.0
  --model-dir PATH    本地 Matcha 模型目录
  --vocoder PATH      本地 Vocos ONNX
  --model-source TEXT 自定义模型来源和许可说明（更换模型时必填）
  --check             只校验已有音频，不加载模型
  --dry-run           只打印命令，不生成、不检查、不写文件
  -h, --help          显示帮助
环境变量 TTS_PYTHON 指定 Python；默认优先使用项目 .cache/tts-venv/bin/python，
不存在时使用 PATH 中的 python3。依赖见 tools/tts/requirements.txt。
可从任意目录调用；显式传入的相对路径以调用时的工作目录为准。
生成成功后自动检查文案、WAV 格式和 SHA256；不会启动机器人或播放声音。
HELP
}

while (( $# )); do
    case "$1" in
        --check) CHECK=true; shift ;;
        --dry-run) DRY_RUN=true; shift ;;
        -h|--help) usage; exit 0 ;;
        --config|--output|--model-dir|--vocoder|--model-source|--threads|--speed)
            (( $# >= 2 )) && [[ -n $2 ]] || { echo "参数缺少值: $1" >&2; exit 2; }
            case "$1" in
                --config) CONFIG=$2 ;;
                --output) OUTPUT=$2 ;;
                --model-dir) MODEL_DIR=$2; CUSTOM_MODEL=true ;;
                --vocoder) VOCODER=$2; CUSTOM_MODEL=true ;;
                --model-source) MODEL_SOURCE=$2; CUSTOM_SOURCE=true ;;
                --threads) THREADS=$2 ;;
                --speed) SPEED=$2 ;;
            esac
            shift 2 ;;
        *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
    esac
done
if ! $CHECK && $CUSTOM_MODEL && ! $CUSTOM_SOURCE; then
    echo '更换模型时请用 --model-source 记录实际来源和许可说明。' >&2
    exit 2
fi
if [[ -z ${TTS_PYTHON:-} ]]; then
    if [[ -x $ROOT/.cache/tts-venv/bin/python ]]; then
        TTS_PYTHON="$ROOT/.cache/tts-venv/bin/python"
    else
        TTS_PYTHON=python3
    fi
fi

base=("$TTS_PYTHON" "$ROOT/tools/tts/generate.py" --config "$CONFIG")
[[ -z $OUTPUT ]] || base+=(--output "$OUTPUT")
generate=("${base[@]}" --model-dir "$MODEL_DIR" --vocoder "$VOCODER"
    --model-source "$MODEL_SOURCE" --threads "$THREADS" --speed "$SPEED")

run() {
    printf '执行:'
    printf ' %q' "$@"
    printf '\n'
    if ! $DRY_RUN; then "$@"; fi
}

if ! $DRY_RUN; then
    command -v "$TTS_PYTHON" >/dev/null || { echo "Python 不存在: $TTS_PYTHON" >&2; exit 1; }
    [[ -r $CONFIG ]] || { echo "配置不可读: $CONFIG" >&2; exit 1; }
    if $CHECK; then
        modules='import yaml'
    else
        modules='import yaml, numpy, sherpa_onnx'
    fi
    if ! "$TTS_PYTHON" -c "$modules"; then
        echo '请先创建 .cache/tts-venv 并安装 tools/tts/requirements.txt，或设置 TTS_PYTHON。' >&2
        exit 1
    fi
fi

if ! $CHECK; then run "${generate[@]}"; fi
run "${base[@]}" --check
