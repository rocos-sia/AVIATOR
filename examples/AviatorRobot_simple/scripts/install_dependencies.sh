#!/usr/bin/env bash
# Compatibility entry point; keep the package list in the root scripts directory.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${script_dir}/../../../scripts/install_dependencies.sh" "$@"
