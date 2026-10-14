#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
binary="$project_dir/build/xCoreSDK_RH56FTP_485"
if [[ ! -x "$binary" ]]; then
    echo "请先按 README 使用 CMake 构建：$binary" >&2
    exit 1
fi
exec "$binary" "$@"
