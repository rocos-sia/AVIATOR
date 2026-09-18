#!/usr/bin/env bash
# Reproducible vendor build of hpp-fcl (installs as "coal") and Pinocchio.
#
# Vendored sources live in third_party/{hpp-fcl,pinocchio}; each repo's
# jrl-cmakemodules CMake submodule has been inlined into its cmake/ directory
# so no network access is required.  Artifacts are installed into
# third_party/_install (headers + CMake configs + shared libraries) and are
# NOT committed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="$ROOT/_install"
GENERATOR="${GENERATOR:-Ninja}"

# hpp-fcl 3.x installs as "coal"; the backward-compatibility option makes it
# also install hpp-fclConfig.cmake -> coal::coal, which Pinocchio expects.
cmake -S "$ROOT/hpp-fcl" -B "$ROOT/build/hpp-fcl" -G "$GENERATOR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DBUILD_TESTING=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON \
    -DCOAL_HAS_QHULL=OFF

cmake --build "$ROOT/build/hpp-fcl" --target install

cmake -S "$ROOT/pinocchio" -B "$ROOT/build/pinocchio" -G "$GENERATOR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DCMAKE_PREFIX_PATH="$PREFIX" \
    -DBUILD_TESTING=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARK=OFF \
    -DBUILD_UTILS=OFF \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_COLLISION_SUPPORT=ON \
    -DBUILD_WITH_SDF_SUPPORT=OFF \
    -DBUILD_WITH_AUTODIFF_SUPPORT=OFF \
    -DBUILD_WITH_CASADI_SUPPORT=OFF \
    -DBUILD_WITH_CODEGEN_SUPPORT=OFF \
    -DBUILD_WITH_OPENMP_SUPPORT=OFF \
    -DBUILD_WITH_EXTRA_SUPPORT=OFF

cmake --build "$ROOT/build/pinocchio" --target install

echo "Installed hpp-fcl (coal) and Pinocchio into $PREFIX"
