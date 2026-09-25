#!/usr/bin/env bash
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
version=1.24.4
root="$here/.deps/onnxruntime"
if [[ -f "$root/include/onnxruntime_cxx_api.h" && -f "$root/lib/libonnxruntime.so" ]]; then
  echo "$root"
  exit 0
fi
mkdir -p "$root"
archive="$(mktemp)"
trap 'rm -f "$archive"' EXIT
curl -fL --retry 3 "https://github.com/microsoft/onnxruntime/releases/download/v${version}/onnxruntime-linux-x64-${version}.tgz" -o "$archive"
tar -xzf "$archive" --strip-components=1 -C "$root"
echo "$root"
