cmake_minimum_required(VERSION 3.16)
project(aviator_foxglove_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies (system + vendored cppzmq headers), matching common/CMakeLists.txt.
find_package(Threads REQUIRED)
find_package(Boost 1.74 REQUIRED)
find_package(OpenCV REQUIRED)

# nlohmann/json is header-only. Prefer the installed CMake config (as the main
# project does); fall back to a bare include path (e.g. a conda prefix) so the
# example still builds standalone.
find_package(nlohmann_json 3.10 QUIET CONFIG)
if(NOT nlohmann_json_FOUND)
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    HINTS "$ENV{CONDA_PREFIX}/include"
          "/opt/conda/include"
          "$ENV{HOME}/miniconda3/include")
  if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR
      "nlohmann/json not found; set -DCMAKE_PREFIX_PATH=<prefix> or install nlohmann-json3-dev")
  endif()
  add_library(nlohmann_json INTERFACE)
  target_include_directories(nlohmann_json INTERFACE ${NLOHMANN_JSON_INCLUDE_DIR})
  add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
endif()

find_path(AVIATOR_ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(AVIATOR_ZMQ_LIBRARY NAMES zmq REQUIRED)

set(CPPZMQ_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/cppzmq-4.11.0")

# --- camera_pub: OpenCV capture -> JPEG -> ZMQ PUB (dedicated image channel) ---
add_executable(camera_pub src/camera_pub.cpp)
target_include_directories(camera_pub PRIVATE ${CPPZMQ_INCLUDE_DIR})
target_link_libraries(camera_pub PRIVATE
  ${AVIATOR_ZMQ_LIBRARY} ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

# --- foxglove_bridge: ZMQ SUB (data + image) -> Boost.Beast WebSocket ---
add_executable(foxglove_bridge src/foxglove_bridge.cpp)
target_include_directories(foxglove_bridge PRIVATE
  ${CPPZMQ_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(foxglove_bridge PRIVATE
  ${AVIATOR_ZMQ_LIBRARY} nlohmann_json::nlohmann_json
  Boost::headers Threads::Threads)

# Smoke tests: --help must parse and exit 0.
enable_testing()
add_test(NAME camera_pub_help COMMAND camera_pub --help)
add_test(NAME foxglove_bridge_help COMMAND foxglove_bridge --help)
