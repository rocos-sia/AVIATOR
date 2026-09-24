# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Build configuration for third party libraries used in MuJoCo.

set(MUJOCO_DEP_VERSION_lodepng
    17d08dd26cac4d63f43af217ebd70318bfb8189c
    CACHE STRING "Version of `lodepng` to be fetched."
)
set(MUJOCO_DEP_VERSION_MarchingCubeCpp
    f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d
    CACHE STRING "Version of `MarchingCubeCpp` to be fetched."
)
set(MUJOCO_DEP_VERSION_Eigen3
    49623d0c4e1af3c680845191948d10f6d3e92f8a
    CACHE STRING "Version of `Eigen3` to be fetched."
)

set(MUJOCO_DEP_VERSION_abseil
    d38452e1ee03523a208362186fd42248ff2609f6 # LTS 20250814.1
    CACHE STRING "Version of `abseil` to be fetched."
)

set(MUJOCO_DEP_VERSION_gtest
    52eb8108c5bdec04579160ae17225d66034bd723 # v1.17.0
    CACHE STRING "Version of `gtest` to be fetched."
)

set(MUJOCO_DEP_VERSION_benchmark
    5f7d66929fb66869d96dfcbacf0d8a586b33766d
    CACHE STRING "Version of `benchmark` to be fetched."
)

set(MUJOCO_DEP_VERSION_TriangleMeshDistance
    2cb643de1436e1ba8e2be49b07ec5491ac604457
    CACHE STRING "Version of `TriangleMeshDistance` to be fetched."
)

mark_as_advanced(MUJOCO_DEP_VERSION_lodepng)
mark_as_advanced(MUJOCO_DEP_VERSION_MarchingCubeCpp)
mark_as_advanced(MUJOCO_DEP_VERSION_Eigen3)
mark_as_advanced(MUJOCO_DEP_VERSION_abseil)
mark_as_advanced(MUJOCO_DEP_VERSION_gtest)
mark_as_advanced(MUJOCO_DEP_VERSION_benchmark)
mark_as_advanced(MUJOCO_DEP_VERSION_TriangleMeshDistance)

include(FetchContent)
include(FindOrFetch)

# Build the remaining vendored dependencies statically. Ubuntu dependencies below
# are imported shared libraries and are unaffected by BUILD_SHARED_LIBS.
set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS
    OFF
    CACHE INTERNAL "Build SHARED libraries"
)

if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    if(NOT EMSCRIPTEN)
      target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
    else()
      target_include_directories(lodepng PUBLIC  $<BUILD_INTERFACE:${lodepng_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
    endif()
  endif()
endif()

if(NOT TARGET marchingcubecpp)
  FetchContent_Declare(
    marchingcubecpp
    GIT_REPOSITORY https://github.com/aparis69/MarchingCubeCpp.git
    GIT_TAG ${MUJOCO_DEP_VERSION_MarchingCubeCpp}
  )

  FetchContent_GetProperties(marchingcubecpp)
  if(NOT marchingcubecpp_POPULATED)
    FetchContent_Populate(marchingcubecpp)
    include_directories(${marchingcubecpp_SOURCE_DIR})
  endif()
endif()

# AVIATOR: use Ubuntu 22.04 development packages, with no source downloads.
find_package(Qhull REQUIRED)
find_path(AVIATOR_QHULL_INCLUDE_DIR qhull_ra.h PATH_SUFFIXES libqhull_r REQUIRED)
add_library(qhullstatic_r INTERFACE IMPORTED)
set_target_properties(qhullstatic_r PROPERTIES
  INTERFACE_LINK_LIBRARIES Qhull::qhull_r
  INTERFACE_INCLUDE_DIRECTORIES "${AVIATOR_QHULL_INCLUDE_DIR}")
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
add_library(tinyxml2 ALIAS PkgConfig::TINYXML2)
find_package(tinyobjloader REQUIRED)
add_library(tinyobjloader ALIAS tinyobjloader::tinyobjloader)

if(NOT TARGET trianglemeshdistance)
  FetchContent_Declare(
    trianglemeshdistance
    GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/TriangleMeshDistance.git
    GIT_TAG ${MUJOCO_DEP_VERSION_TriangleMeshDistance}
  )

  FetchContent_GetProperties(trianglemeshdistance)
  if(NOT trianglemeshdistance_POPULATED)
    FetchContent_Populate(trianglemeshdistance)
    # Patch the source code to silence a warning/error related to a loop variable creating a copy.
    # Since this is a header only library this fix is less intrusive than disabling the warning for
    # any target including the header.
    # Aviator: this upstream fix is already applied to the vendored header.
    include_directories(${trianglemeshdistance_SOURCE_DIR})
  endif()
endif()

# Ubuntu libccd is built with double precision, matching MuJoCo's ABI.
find_package(ccd REQUIRED)
include(CheckCSourceCompiles)
get_target_property(CMAKE_REQUIRED_INCLUDES ccd INTERFACE_INCLUDE_DIRECTORIES)
check_c_source_compiles("#include <ccd/config.h>
#ifndef CCD_DOUBLE
#error libccd must use double precision
#endif
int main(void) { return 0; }" AVIATOR_CCD_DOUBLE)
unset(CMAKE_REQUIRED_INCLUDES)
if(NOT AVIATOR_CCD_DOUBLE)
  message(FATAL_ERROR "MuJoCo requires a double-precision libccd")
endif()

if(MUJOCO_BUILD_TESTS OR MUJOCO_BUILD_STUDIO OR MUJOCO_USE_FILAMENT)
  set(ABSL_PROPAGATE_CXX_STD ON)

  # This specific version of Abseil does not have the following variable. We need to work with BUILD_TESTING
  set(BUILD_TESTING_OLD ${BUILD_TESTING})
  set(BUILD_TESTING
      OFF
      CACHE INTERNAL "Build tests."
  )

  set(ABSL_BUILD_TESTING OFF)
  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    absl
    LIBRARY_NAME
    abseil-cpp
    GIT_REPO
    https://github.com/abseil/abseil-cpp.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_abseil}
    TARGETS
    absl::core_headers
    EXCLUDE_FROM_ALL
  )

  set(BUILD_TESTING
      ${BUILD_TESTING_OLD}
      CACHE BOOL "Build tests." FORCE
  )
endif()

if(MUJOCO_BUILD_TESTS)

  # Avoid linking errors on Windows by dynamically linking to the C runtime.
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE
  )

  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    GTest
    LIBRARY_NAME
    googletest
    GIT_REPO
    https://github.com/google/googletest.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_gtest}
    TARGETS
    gtest
    gmock
    gtest_main
    EXCLUDE_FROM_ALL
  )

  set(BENCHMARK_EXTRA_FETCH_ARGS "")
  if(WIN32 AND NOT MSVC)
    set(BENCHMARK_EXTRA_FETCH_ARGS
        PATCH_COMMAND
        "sed"
        "-i"
        "-e"
        "s/-std=c++11/-std=c++14/g"
        "-e"
        "s/HAVE_CXX_FLAG_STD_CXX11/HAVE_CXX_FLAG_STD_CXX14/g"
        "${CMAKE_BINARY_DIR}/_deps/benchmark-src/CMakeLists.txt"
    )
  endif()

  set(BENCHMARK_ENABLE_TESTING OFF)

  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    benchmark
    LIBRARY_NAME
    benchmark
    GIT_REPO
    https://github.com/google/benchmark.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_benchmark}
    TARGETS
    benchmark::benchmark
    benchmark::benchmark_main
    ${BENCHMARK_EXTRA_FETCH_ARGS}
    EXCLUDE_FROM_ALL
  )
endif()

if(MUJOCO_TEST_PYTHON_UTIL)
  add_compile_definitions(EIGEN_MPL2_ONLY)
  if(NOT TARGET eigen)
    # Support new IN_LIST if() operator.
    set(CMAKE_POLICY_DEFAULT_CMP0057 NEW)

    FetchContent_Declare(
      Eigen3
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG ${MUJOCO_DEP_VERSION_Eigen3}
    )

    FetchContent_GetProperties(Eigen3)
    if(NOT Eigen3_POPULATED)
      FetchContent_Populate(Eigen3)

      # Mark the library as IMPORTED as a workaround for https://gitlab.kitware.com/cmake/cmake/-/issues/15415
      add_library(Eigen3::Eigen INTERFACE IMPORTED)
      set_target_properties(
        Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen3_SOURCE_DIR}"
      )
    endif()
  endif()
endif()

# Reset BUILD_SHARED_LIBS to its previous value
set(BUILD_SHARED_LIBS
    ${BUILD_SHARED_LIBS_OLD}
    CACHE BOOL "Build MuJoCo as a shared library" FORCE
)
