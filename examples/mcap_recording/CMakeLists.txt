# Standalone build for the MCAP full-recording example.
#
# Build & run:
#   cmake -S . -B build
#   cmake --build build
#   ./build/mcap_recording_example [output.mcap]
#
# This example is intentionally not wired into the root build so it can be
# tuned independently; promote it into nodes/aviator_logger when ready.

cmake_minimum_required(VERSION 3.16)
project(aviator_mcap_recording_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(mcap_recording_example src/main.cpp)

target_include_directories(mcap_recording_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mcap/include)

# Header-only MCAP: MCAP_IMPLEMENTATION is defined in exactly one TU (main.cpp).
# Disable zstd/lz4 so no external compression libraries are required.
target_compile_definitions(mcap_recording_example PRIVATE
    MCAP_COMPRESSION_NO_ZSTD
    MCAP_COMPRESSION_NO_LZ4)

if(MSVC)
    target_compile_options(mcap_recording_example PRIVATE /W4)
else()
    target_compile_options(mcap_recording_example PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_test(NAME mcap_recording_example COMMAND mcap_recording_example)
