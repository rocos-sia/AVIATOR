cmake_minimum_required(VERSION 3.22)
project(AVIATOR VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(FATAL_ERROR "Use an out-of-source build: cmake --preset linux-debug")
endif()

# Project-wide settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(CTest)
include(GNUInstallDirs)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# RPATH settings for installed binaries
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

#------------------------------------------------------------------------------
# Third-party dependencies
#------------------------------------------------------------------------------
include(cmake/ThirdPartyDependencies.cmake)

#------------------------------------------------------------------------------
# Project components
#------------------------------------------------------------------------------
# Directory scaffold only. Add target_compile_features(... cxx_std_17)
# and target-scoped dependencies when each implementation is introduced.
add_subdirectory(common)
add_subdirectory(nodes)

#------------------------------------------------------------------------------
# Examples
#------------------------------------------------------------------------------
option(AVIATOR_BUILD_EXAMPLES "Build example projects" ON)

if(AVIATOR_BUILD_EXAMPLES)
    message(STATUS "Building examples...")
    add_subdirectory(examples)
endif()

#------------------------------------------------------------------------------
# Tests
#------------------------------------------------------------------------------
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#------------------------------------------------------------------------------
# Installation
#------------------------------------------------------------------------------
# Draft configuration and deployment files are deliberately not installed.
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(DIRECTORY docs/ DESTINATION ${CMAKE_INSTALL_DOCDIR})

# Install third-party libraries
if(AVIATOR_BUILD_ROBOTICS_LIBS OR AVIATOR_BUILD_MUJOCO)
    install(DIRECTORY "${AVIATOR_DEPS_DIR}/lib/" DESTINATION ${CMAKE_INSTALL_LIBDIR}
        FILES_MATCHING PATTERN "*.so*")
endif()
