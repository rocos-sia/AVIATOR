#!/usr/bin/env bash
# Install build dependencies for examples/AviatorRobot_simple on Ubuntu 22.04.
set -euo pipefail
case "${1:-}" in
  --help|-h) echo "Usage: $0 [--dry-run]"; exit 0 ;;
  ""|--dry-run) ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo "Too many arguments" >&2; exit 2; fi
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
  echo "This script supports Ubuntu 22.04 (jammy)." >&2
  exit 1
fi
packages=(
  build-essential cmake ninja-build pkg-config python3
  libassimp-dev
  libboost-date-time-dev
  libboost-filesystem-dev
  libboost-serialization-dev
  libboost-system-dev
  libboost-thread-dev
  libccd-dev
  libconsole-bridge-dev
  libeigen3-dev
  libglfw3-dev
  libnlopt-cxx-dev
  libnlopt-dev
  liboctomap-dev
  libqhull-dev
  libtinyobjloader-dev
  libtinyxml-dev
  libtinyxml2-dev
  liburdfdom-dev
  liburdfdom-headers-dev
  libyaml-cpp-dev
  zlib1g-dev
  libgl1-mesa-dev
)
if [[ "${1:-}" == --dry-run ]]; then
  printf 'apt-get update\napt-get install -y --no-install-recommends'
  printf ' %q' "${packages[@]}"
  printf '\n'
  exit 0
fi
privilege=()
if (( EUID != 0 )); then
  command -v sudo >/dev/null || { echo "Run as root or install sudo." >&2; exit 1; }
  privilege=(sudo)
fi
# Several packages are in Ubuntu's universe component.
# Do not add PPAs or change the user's repository configuration automatically.
"${privilege[@]}" apt-get update
for package in "${packages[@]}"; do
  if ! apt-cache show "$package" >/dev/null 2>&1; then
    echo "Missing package: $package. Enable Ubuntu jammy universe and retry." >&2
    exit 1
  fi
done
"${privilege[@]}" apt-get install -y --no-install-recommends "${packages[@]}"
