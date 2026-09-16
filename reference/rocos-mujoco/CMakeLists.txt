# Copyright 2026, Yang Luo
# SPDX-License-Identifier: GPL-3.0-or-later
#
# CMakeLists.txt
# Build system for the rocos-mujoco MuJoCo simulation environment.

cmake_minimum_required(VERSION 3.16)
project(rocos_mujoco VERSION 1.0.0
        DESCRIPTION "ROCOS MuJoCo Simulation Environment")

# ---- C++ standard --------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(ROCOS_MUJOCO_ENABLE_VISUALIZATION
       "Build the GLFW/OpenGL MuJoCo visualization window" ON)

# ==========================================================================
# Third-party libraries (仿照 RoboChargeX 的 third_party 结构)
# ==========================================================================

# ---- MuJoCo 3.4.0 --------------------------------------------------------
set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco-3.4.0")
if(NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so" AND
   NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so.3.4.0")
    set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.4.0")
endif()
message(STATUS "Third-party MuJoCo:  ${MUJOCO_DIR}")

if(EXISTS "${MUJOCO_DIR}/lib/libmujoco.so.3.4.0")
    # Prefer the real versioned library. On Windows-mounted WSL checkouts,
    # libmujoco.so can be a plain-text symlink placeholder.
    set(MUJOCO_LIB "${MUJOCO_DIR}/lib/libmujoco.so.3.4.0")
elseif(EXISTS "${MUJOCO_DIR}/lib/libmujoco.so")
    set(MUJOCO_LIB "${MUJOCO_DIR}/lib/libmujoco.so")
else()
    find_library(MUJOCO_LIB mujoco
                 PATHS "${MUJOCO_DIR}/lib"
                 NO_DEFAULT_PATH REQUIRED)
endif()

add_library(third_party::mujoco SHARED IMPORTED)
set_target_properties(third_party::mujoco PROPERTIES
    IMPORTED_LOCATION              "${MUJOCO_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES  "${MUJOCO_DIR}/include")

# ---- GLFW / OpenGL (optional visualization) --------------------------------
if(ROCOS_MUJOCO_ENABLE_VISUALIZATION)
    find_package(glfw3 QUIET)
    if(NOT glfw3_FOUND)
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(GLFW3 glfw3)
        endif()
    endif()
    if(glfw3_FOUND)
        message(STATUS "Third-party GLFW:   system (cmake)")
    elseif(GLFW3_FOUND)
        message(STATUS "Third-party GLFW:   system (pkg-config)")
    else()
        message(FATAL_ERROR "GLFW not found; use -DROCOS_MUJOCO_ENABLE_VISUALIZATION=OFF for headless servers")
    endif()

    find_package(OpenGL REQUIRED)
    message(STATUS "Visualization:      enabled")
else()
    message(STATUS "Visualization:      disabled (headless)")
endif()

# ---- yaml-cpp -------------------------------------------------------------
find_package(yaml-cpp REQUIRED)
message(STATUS "Third-party yaml-cpp: system")

# ---- Threads --------------------------------------------------------------
find_package(Threads REQUIRED)

# ==========================================================================
# Build targets
# ==========================================================================

# ---- Shared library: librocos_mujoco.so -----------------------------------
add_library(rocos_mujoco SHARED
    src/mujoco_simulator.cpp
    src/docking.cpp
    src/crawling.cpp
    src/aviator_grasp.cpp
)
if(ROCOS_MUJOCO_ENABLE_VISUALIZATION)
    target_sources(rocos_mujoco PRIVATE src/hardware_mj.cpp)
endif()

target_include_directories(rocos_mujoco
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(rocos_mujoco
    PUBLIC
        third_party::mujoco
        yaml-cpp
    PRIVATE
        Threads::Threads
        rt
        pthread
)
if(ROCOS_MUJOCO_ENABLE_VISUALIZATION)
    target_compile_definitions(rocos_mujoco
        PUBLIC ROCOS_MUJOCO_ENABLE_VISUALIZATION=1)
    target_link_libraries(rocos_mujoco PRIVATE OpenGL::GL)
    # GLFW: link conditionally based on how it was found
    if(glfw3_FOUND)
        target_link_libraries(rocos_mujoco PRIVATE glfw)
    else()
        target_link_libraries(rocos_mujoco PRIVATE ${GLFW3_LIBRARIES})
        target_include_directories(rocos_mujoco PRIVATE ${GLFW3_INCLUDE_DIRS})
    endif()
endif()

# ---- Executable: rocos_mujoco_sim -----------------------------------------
add_executable(rocos_mujoco_sim
    src/main.cpp
)
target_link_libraries(rocos_mujoco_sim PRIVATE rocos_mujoco)

set(SPARC_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Model" CACHE PATH
    "SPARC Model directory containing urdf/ and meshes/")
get_filename_component(SPARC_MODEL_DIR "${SPARC_MODEL_DIR}" ABSOLUTE)
set(HAVE_SPARC_MODEL ON)
foreach(model_file urdf/scene.xml urdf/SPARC.xml urdf/SPARC.urdf)
    if(NOT EXISTS "${SPARC_MODEL_DIR}/${model_file}")
        set(HAVE_SPARC_MODEL OFF)
    endif()
endforeach()
if(NOT HAVE_SPARC_MODEL)
    message(STATUS "Optional SPARC assets not found; building AVIATOR without SPARC tests/copies")
endif()
target_compile_definitions(rocos_mujoco_sim PRIVATE
    ROCOS_MUJOCO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    SPARC_MODEL_DIR="${SPARC_MODEL_DIR}")

include(CTest)
if(BUILD_TESTING)
    add_executable(aviator_control_test tests/aviator_control_test.cpp)
    target_link_libraries(aviator_control_test PRIVATE rocos_mujoco rt pthread)
    add_test(NAME aviator_control
        COMMAND aviator_control_test
            "${CMAKE_CURRENT_SOURCE_DIR}/model/aviator.xml"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/hardware_aviator_config.yaml")
endif()
if(BUILD_TESTING AND HAVE_SPARC_MODEL)
    add_executable(sparc_control_test tests/sparc_control_test.cpp)
    target_link_libraries(sparc_control_test PRIVATE rocos_mujoco rt pthread)
    foreach(model_file scene.xml SPARC.urdf)
        add_test(NAME sparc_${model_file}
            COMMAND sparc_control_test "${SPARC_MODEL_DIR}/urdf/${model_file}"
                "${CMAKE_CURRENT_SOURCE_DIR}/config/hardware_SPARC_config.yaml")
    endforeach()
endif()

# ==========================================================================
# Install
# ==========================================================================
include(GNUInstallDirs)

install(TARGETS rocos_mujoco rocos_mujoco_sim
    EXPORT  rocos_mujoco-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# ---- Copy config & model files to build directory -------------------------
# Always synchronize assets, including when only XML/YAML/meshes changed and
# the executable does not need relinking.
add_custom_target(rocos_mujoco_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/config"
        "${CMAKE_BINARY_DIR}/bin/config"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/model"
        "${CMAKE_BINARY_DIR}/bin/model"
)
add_dependencies(rocos_mujoco_sim rocos_mujoco_assets)
if(HAVE_SPARC_MODEL)
    add_custom_target(rocos_mujoco_sparc_assets
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SPARC_MODEL_DIR}/urdf"
            "${CMAKE_BINARY_DIR}/bin/model/SPARC/urdf"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SPARC_MODEL_DIR}/meshes"
            "${CMAKE_BINARY_DIR}/bin/model/SPARC/meshes"
    )
    add_dependencies(rocos_mujoco_assets rocos_mujoco_sparc_assets)
endif()
