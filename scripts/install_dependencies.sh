#!/usr/bin/env bash
# Install apt dependencies for the root AVIATOR build on Ubuntu 22.04.
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: install_dependencies.sh [--dry-run] [--with-examples]

Install the full root project's system libraries and GCC/CMake/Ninja/LLVM/GDB tools.
  --dry-run        Print apt commands without sudo, downloads or system changes.
  --with-examples  Also install OpenCV and Protobuf for standalone Foxglove examples.
  -h, --help      Show this help.

Supported system: Ubuntu 22.04 (jammy), with the universe repository enabled.
Vendored libraries are built by CMake; ONNX Runtime and Python environments are
not installed by this script. No repositories or PPAs are added automatically.
HELP
}

dry_run=false
with_examples=false
for arg in "$@"; do
  case "$arg" in
    --dry-run) dry_run=true ;;
    --with-examples) with_examples=true ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
  esac
done

source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
  echo "This script supports Ubuntu 22.04 (jammy)." >&2
  exit 1
fi
command -v apt-get >/dev/null || { echo "apt-get is required." >&2; exit 1; }

packages=(
  # Build tools, editor language services, formatter, static analysis and debugger.
  build-essential cmake ninja-build pkg-config python3
  clangd clang-format clang-tidy gdb
  # common/ transport and JSON protocol.
  libzmq3-dev nlohmann-json3-dev
  # cmake/ThirdPartyDependencies.cmake: robotics and system libraries.
  libassimp-dev
  libboost-date-time-dev libboost-filesystem-dev libboost-serialization-dev
  libboost-system-dev libboost-thread-dev
  libconsole-bridge-dev libeigen3-dev
  libnlopt-dev libnlopt-cxx-dev
  liboctomap-dev libtinyxml-dev libtinyxml2-dev
  liburdfdom-dev liburdfdom-headers-dev libyaml-cpp-dev zlib1g-dev
  # Vendored MuJoCo, viewer and nodes/simulation EGL camera rendering.
  libccd-dev libqhull-dev libtinyobjloader-dev
  libglfw3-dev libgl1-mesa-dev libegl1-mesa-dev
)
if "$with_examples"; then
  # examples/foxglove_bridge, including regeneration of protobuf sources.
  packages+=(libopencv-dev libprotobuf-dev protobuf-compiler)
fi

if "$dry_run"; then
  printf 'apt-get update\napt-get install -y --no-install-recommends --no-remove'
  printf ' %q' "${packages[@]}"
  printf '\n'
  exit 0
fi

privilege=()
if (( EUID != 0 )); then
  command -v sudo >/dev/null || { echo "Run as root or install sudo." >&2; exit 1; }
  privilege=(sudo)
fi
"${privilege[@]}" apt-get update
for package in "${packages[@]}"; do
  if ! apt-cache show "$package" >/dev/null 2>&1; then
    echo "Missing package: $package. Check the Ubuntu jammy universe repository." >&2
    exit 1
  fi
done
"${privilege[@]}" apt-get install -y --no-install-recommends --no-remove "${packages[@]}"
echo "APT dependencies installed. Next: cmake --preset debug && cmake --build --preset debug"
