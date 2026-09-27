# Shared source-build support for full and simulation-only builds.
include_guard(GLOBAL)

include(ExternalProject)

# Path to vendored third-party libraries
set(AVIATOR_VENDOR_DIR "${CMAKE_SOURCE_DIR}/third_party" CACHE PATH "Third-party vendor directory")

# Build output directory for third-party libraries
set(AVIATOR_DEPS_DIR "${CMAKE_BINARY_DIR}/third_party/install" CACHE PATH "Third-party install directory")
file(MAKE_DIRECTORY "${AVIATOR_DEPS_DIR}/include" "${AVIATOR_DEPS_DIR}/lib")

# Parallel build jobs for third-party projects
set(AVIATOR_DEPENDENCY_JOBS 2 CACHE STRING "Parallel compiler jobs per third-party project")

# Custom target to build all third-party dependencies
add_custom_target(aviator_third_party_all)

#------------------------------------------------------------------------------
# Helper function to build source-based third-party libraries
#------------------------------------------------------------------------------
function(aviator_add_third_party_source name)
    cmake_parse_arguments(ARG "" "SOURCE_DIR" "DEPENDS;CMAKE_ARGS;LIBRARIES" ${ARGN})

    if(NOT ARG_SOURCE_DIR)
        set(ARG_SOURCE_DIR "${AVIATOR_VENDOR_DIR}/${name}")
    endif()

    if(NOT EXISTS "${ARG_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "Third-party ${name} source not found at ${ARG_SOURCE_DIR}")
    endif()

    # Build list of library outputs for dependency tracking
    set(outputs "")
    foreach(lib IN LISTS ARG_LIBRARIES)
        list(APPEND outputs "${AVIATOR_DEPS_DIR}/lib/lib${lib}.so")
    endforeach()

    # Prepare prefix path for nested dependencies
    set(dependency_prefixes "${AVIATOR_DEPS_DIR};${CMAKE_PREFIX_PATH}")
    list(REMOVE_DUPLICATES dependency_prefixes)
    string(REPLACE ";" "|" dependency_prefixes "${dependency_prefixes}")

    ExternalProject_Add(third_party_${name}
        LIST_SEPARATOR |
        SOURCE_DIR "${ARG_SOURCE_DIR}"
        BINARY_DIR "${CMAKE_BINARY_DIR}/third_party/build/${name}"
        PREFIX "${CMAKE_BINARY_DIR}/third_party/stamps/${name}"
        DOWNLOAD_COMMAND ""
        UPDATE_COMMAND ""
        PATCH_COMMAND ""
        DEPENDS ${ARG_DEPENDS}
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_INSTALL_PREFIX=${AVIATOR_DEPS_DIR}
            -DCMAKE_INSTALL_LIBDIR=lib
            "-DCMAKE_PREFIX_PATH=${dependency_prefixes}"
            -DCMAKE_INSTALL_RPATH=$ORIGIN
            -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF
            -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
            -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
            -DFETCHCONTENT_FULLY_DISCONNECTED=ON
            -DBUILD_SHARED_LIBS=ON
            -DBUILD_TESTING=OFF
            ${ARG_CMAKE_ARGS}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --parallel ${AVIATOR_DEPENDENCY_JOBS}
            COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
        INSTALL_COMMAND ""
        BUILD_ALWAYS ON
        BUILD_BYPRODUCTS ${outputs}
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
        LOG_OUTPUT_ON_FAILURE ON
    )

    add_dependencies(aviator_third_party_all third_party_${name})
endfunction()

