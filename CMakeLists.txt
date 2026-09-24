cmake_minimum_required(VERSION 3.22)
project(AVIATOR LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(FATAL_ERROR "Use an out-of-source build: cmake --preset linux-debug")
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
include(CTest)
include(GNUInstallDirs)

# Directory scaffold only. Add target_compile_features(... cxx_std_17)
# and target-scoped dependencies when each implementation is introduced.
add_subdirectory(common)
add_subdirectory(nodes)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# Draft configuration and deployment files are deliberately not installed.
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(DIRECTORY docs/ DESTINATION ${CMAKE_INSTALL_DOCDIR})
