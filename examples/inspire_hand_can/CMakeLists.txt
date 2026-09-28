cmake_minimum_required(VERSION 3.16)
project(aviator_inspire_hand LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Repo root, for the vendored cppzmq headers.
set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

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

set(CPPZMQ_INCLUDE_DIR "${ROOT}/third_party/cppzmq-4.11.0")

add_executable(inspire_hand_node hand_node.cpp)
target_include_directories(inspire_hand_node PRIVATE
  ${CPPZMQ_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(inspire_hand_node PRIVATE
  ${AVIATOR_ZMQ_LIBRARY} yaml-cpp nlohmann_json::nlohmann_json Threads::Threads)

# Smoke test: --help must parse and exit 0.
enable_testing()
add_test(NAME inspire_hand_node_help COMMAND inspire_hand_node --help)
