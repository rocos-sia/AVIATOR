#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
resource_root=$(cd -- "$project_dir/../.." && pwd)
if [[ -f $resource_root/../../release-version.json ]]; then
    binary="$resource_root/../../bin/xCoreSDK_RH56FTP_485"
    # Explicit caller options follow this default and can override it.
    set -- --config "$project_dir/config.ini" "$@"
else
    binary="$project_dir/build/xCoreSDK_RH56FTP_485"
fi
if [[ ! -x "$binary" ]]; then
    echo "请先按 README 使用 CMake 构建：$binary" >&2
    exit 1
fi
exec "$binary" "$@"
