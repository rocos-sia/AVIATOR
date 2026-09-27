include_guard(GLOBAL)
include(${CMAKE_CURRENT_LIST_DIR}/ThirdPartySource.cmake)

# Build only the vendored MuJoCo, without pulling in robotics/SDK dependencies.
function(aviator_add_mujoco)
    if(TARGET mujoco_imported)
        return()
    endif()
    if(NOT EXISTS "${AVIATOR_VENDOR_DIR}/mujoco-3.4.0/CMakeLists.txt")
        message(FATAL_ERROR "Missing vendored MuJoCo: ${AVIATOR_VENDOR_DIR}/mujoco-3.4.0")
    endif()
    message(STATUS "Building MuJoCo simulation library...")

    find_package(Qhull REQUIRED)
    find_package(ccd REQUIRED)
    find_package(tinyobjloader REQUIRED)

    # Prepare FetchContent source directories for MuJoCo dependencies
    set(mujoco_cmake_args "")
    foreach(dep lodepng marchingcubecpp trianglemeshdistance)
        string(TOUPPER "${dep}" dep_upper)
        list(APPEND mujoco_cmake_args
            "-DFETCHCONTENT_SOURCE_DIR_${dep_upper}=${AVIATOR_VENDOR_DIR}/${dep}"
        )
    endforeach()

    aviator_add_third_party_source(mujoco-3.4.0
        LIBRARIES mujoco
        CMAKE_ARGS
            ${mujoco_cmake_args}
            -DMUJOCO_BUILD_EXAMPLES=OFF
            -DMUJOCO_BUILD_SIMULATE=OFF
            -DMUJOCO_BUILD_STUDIO=OFF
            -DMUJOCO_BUILD_TESTS=OFF
            -DMUJOCO_TEST_PYTHON_UTIL=OFF
            -DMUJOCO_WITH_USD=OFF
            -DMUJOCO_USE_FILAMENT=OFF
    )

    # Create imported target for MuJoCo
    add_library(mujoco_imported SHARED IMPORTED GLOBAL)
    set_target_properties(mujoco_imported PROPERTIES
        IMPORTED_LOCATION "${AVIATOR_DEPS_DIR}/lib/libmujoco.so"
        INTERFACE_INCLUDE_DIRECTORIES "${AVIATOR_DEPS_DIR}/include"
    )
    add_dependencies(mujoco_imported third_party_mujoco-3.4.0)
endfunction()
