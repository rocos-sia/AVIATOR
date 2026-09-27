cmake_minimum_required(VERSION 3.16)
project(aviator_t6_joystick LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Repo root, for resolving the MuJoCo build + t6 sources + LUT paths.
set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

# --- MuJoCo 3.4.0 (built as a dependency of tools/aviator_clearance) ---
set(MUJOCO_PREFIX "${ROOT}/build/aviator_clearance/third_party/install" CACHE PATH
    "MuJoCo install prefix (include/ + lib/)")
find_library(MUJOCO_LIB NAMES mujoco HINTS "${MUJOCO_PREFIX}/lib" REQUIRED)
find_path(MUJOCO_INC mujoco/mujoco.h HINTS "${MUJOCO_PREFIX}/include" REQUIRED)

# --- aviator_t6 (prebuilt static lib) + ONNX Runtime (runtime shared lib) ---
set(T6_DIR "${ROOT}/examples/t6")
set(ORT_PREFIX "${ROOT}/methods/t6/.deps/onnxruntime" CACHE PATH
    "ONNX Runtime release root (include/ + lib/)")
find_library(ONNXRUNTIME_LIB NAMES onnxruntime HINTS "${ORT_PREFIX}/lib" REQUIRED)

# --- joystick (prebuilt static lib, examples/joystick) ---
set(JOYSTICK_DIR "${ROOT}/examples/joystick")
set(JOYSTICK_LIB "${JOYSTICK_DIR}/build/libjoystick.a")

# --- GLFW (system) + the existing aviator Viewer component ---
find_library(GLFW_LIB NAMES glfw REQUIRED)
find_path(GLFW_INC GLFW/glfw3.h REQUIRED)
set(VIEWER_SRC "${ROOT}/examples/AviatorRobot_simple/src")

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIB}")
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${ONNXRUNTIME_LIB}")

# Milestone 1: kinematic LUT-start vs MJCF TCP alignment check.
add_executable(align_check align_check.cpp)
target_include_directories(align_check PRIVATE "${MUJOCO_INC}" "${T6_DIR}")
target_link_libraries(align_check PRIVATE
    "${T6_DIR}/build/libaviator_t6.a" mujoco onnxruntime)
set_target_properties(align_check PROPERTIES
    BUILD_RPATH "${ORT_PREFIX}/lib;${MUJOCO_PREFIX}/lib")

# Milestone 2+3: passive-wheel + grasp-weld + T6 arm-servo closed loop, joystick
# wired in (scripted-sine fallback when no device is present), plus the reusable
# GLFW viewer from AviatorRobot_simple (main-thread render, worker-thread control).
add_executable(t6_joystick_sim main.cpp sim_backend.cpp "${VIEWER_SRC}/Viewer.cpp")
target_include_directories(t6_joystick_sim PRIVATE
    "${MUJOCO_INC}" "${T6_DIR}" "${JOYSTICK_DIR}/include" "${VIEWER_SRC}" "${GLFW_INC}")
target_link_libraries(t6_joystick_sim PRIVATE
    "${T6_DIR}/build/libaviator_t6.a" "${JOYSTICK_LIB}" mujoco onnxruntime
    "${GLFW_LIB}" pthread)
set_target_properties(t6_joystick_sim PROPERTIES
    BUILD_RPATH "${ORT_PREFIX}/lib;${MUJOCO_PREFIX}/lib")
