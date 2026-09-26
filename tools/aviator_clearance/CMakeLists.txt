# Standalone build for the `build_manifold_phi` manifold builder (ported from the
# grasp-conditioned branch).  This is a minimal-footprint, independent target:
# it is NOT added to dev's main build tree (no add_subdirectory from the root
# CMakeLists.txt) -- configure it directly:
#
#   cmake -S tools/aviator_clearance -B build/aviator_clearance
#   cmake --build build/aviator_clearance -j
#
# It reuses the dev repo's vendored third-party build (Coal, Pinocchio, PIN-IK,
# MuJoCo) via cmake/ThirdPartyDependencies.cmake, but installs those libraries
# into THIS project's build directory so dev's own build tree is left untouched.
#
# The port replaces KDL / kdl_parser / TRAC-IK with Pinocchio + PIN-IK.  The
# KDL-shaped math surface the tool uses is re-provided by kdl_compat.hpp (thin
# Eigen/Pinocchio wrappers), so only the IK backend and the URDF-chain setup
# changed in clearance_trajectory.cpp.
cmake_minimum_required(VERSION 3.22)
# MuJoCo is a C + C++ project (its libccd precision check compiles C), so the
# C language must be enabled here or the vendored ExternalProject configure
# fails with "No CMAKE_C_COMPILER could be found".
project(aviator_clearance LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Repo root (this file lives at <root>/tools/aviator_clearance).
get_filename_component(AVIATOR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

# Reuse the dev repo's vendored third-party build.  Pin the vendor dir at the
# dev repo's sources; install deps into THIS project's build dir (minimal
# footprint -- dev's own build tree is never modified).
set(AVIATOR_VENDOR_DIR "${AVIATOR_ROOT}/third_party" CACHE PATH "Vendored third-party sources" FORCE)
set(AVIATOR_DEPS_DIR "${CMAKE_BINARY_DIR}/third_party/install" CACHE PATH "Third-party install dir" FORCE)

# Only the robotics stack and MuJoCo are needed; disable the dev-only extras.
set(AVIATOR_BUILD_ROBOTICS_LIBS ON CACHE BOOL "Build Coal/Pinocchio/PIN-IK" FORCE)
set(AVIATOR_BUILD_MUJOCO ON CACHE BOOL "Build MuJoCo" FORCE)
set(AVIATOR_BUILD_XCORE_SDK OFF CACHE BOOL "" FORCE)
set(AVIATOR_BUILD_VIEWER OFF CACHE BOOL "" FORCE)

include("${AVIATOR_ROOT}/cmake/ThirdPartyDependencies.cmake")

find_package(Threads REQUIRED)

add_executable(build_manifold_phi clearance_trajectory.cpp)
target_link_libraries(build_manifold_phi PRIVATE
    aviator_robotics_deps   # Coal/Pinocchio/PIN-IK + system deps + ${AVIATOR_DEPS_DIR}/include
    mujoco_imported
    Threads::Threads
)
