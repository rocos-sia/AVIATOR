cmake_minimum_required(VERSION 3.16)
project(aviator_t6 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.deps/onnxruntime" CACHE PATH
    "ONNX Runtime C++ release root (include/ and lib/)")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
          HINTS "${ONNXRUNTIME_ROOT}/include" REQUIRED)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
             HINTS "${ONNXRUNTIME_ROOT}/lib" REQUIRED)
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}")
add_library(aviator_t6 STATIC t6.cpp feedback.cpp onnx_actor.cpp)
target_include_directories(aviator_t6 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(aviator_t6 PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(aviator_t6 PUBLIC onnxruntime)
add_executable(t6_smoke smoke.cpp)
target_link_libraries(t6_smoke PRIVATE aviator_t6)
add_executable(t6_feedback_smoke feedback_smoke.cpp)
target_link_libraries(t6_feedback_smoke PRIVATE aviator_t6)
add_executable(t6_lut_probe lut_probe.cpp)
target_link_libraries(t6_lut_probe PRIVATE aviator_t6)
enable_testing()
add_test(NAME t6_feedback_smoke COMMAND t6_feedback_smoke
         ${CMAKE_CURRENT_SOURCE_DIR}/actor_60k.onnx
         ${CMAKE_CURRENT_SOURCE_DIR}/../../hil-serl/data/aviator/manifold_phi)
