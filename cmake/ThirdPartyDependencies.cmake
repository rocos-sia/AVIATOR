# Third-party dependencies management for AVIATOR project
# This file handles both system packages and vendored libraries

include(${CMAKE_CURRENT_LIST_DIR}/ThirdPartySource.cmake)

#------------------------------------------------------------------------------
# System packages - required for all builds
#------------------------------------------------------------------------------
message(STATUS "Checking system dependencies...")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Boost 1.74 REQUIRED COMPONENTS filesystem system thread date_time serialization)
find_package(yaml-cpp REQUIRED)
find_package(NLopt REQUIRED)
find_package(assimp REQUIRED)
find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(console_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
find_package(ZLIB REQUIRED)
find_library(AVIATOR_TINYXML_LIBRARY NAMES tinyxml REQUIRED)

#------------------------------------------------------------------------------
# Create interface library for common system dependencies
#------------------------------------------------------------------------------
add_library(aviator_system_deps INTERFACE)
target_include_directories(aviator_system_deps SYSTEM INTERFACE
    ${urdfdom_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
)
target_link_libraries(aviator_system_deps INTERFACE
    Eigen3::Eigen
    yaml-cpp
    ${NLOPT_LIBRARIES}
    ${AVIATOR_TINYXML_LIBRARY}
    PkgConfig::TINYXML2
    ${urdfdom_LIBRARIES}
    ${console_bridge_LIBRARIES}
    Boost::filesystem
    Boost::system
    Boost::thread
    Boost::date_time
    Boost::serialization
)

#------------------------------------------------------------------------------
# Build vendored robotics libraries (Coal, Pinocchio, PIN-IK)
#------------------------------------------------------------------------------
option(AVIATOR_BUILD_ROBOTICS_LIBS "Build vendored robotics libraries (Coal, Pinocchio, PIN-IK)" ON)

if(AVIATOR_BUILD_ROBOTICS_LIBS)
    message(STATUS "Building vendored robotics libraries...")

    # Boost arguments shared by multiple libraries
    set(boost_cmake_args -DBoost_NO_BOOST_CMAKE=ON)

    # Coal (collision detection)
    aviator_add_third_party_source(coal-3.0.4
        LIBRARIES coal
        CMAKE_ARGS
            ${boost_cmake_args}
            -DBUILD_PYTHON_INTERFACE=OFF
            -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON
            -DCOAL_HAS_QHULL=OFF
    )

    # Pinocchio (rigid body dynamics)
    aviator_add_third_party_source(pinocchio-3.9.0
        DEPENDS third_party_coal-3.0.4
        LIBRARIES pinocchio_default pinocchio_collision pinocchio_parsers
        CMAKE_ARGS
            ${boost_cmake_args}
            -DBUILD_PYTHON_INTERFACE=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_BENCHMARK=OFF
            -DBUILD_UTILS=OFF
            -DBUILD_WITH_URDF_SUPPORT=ON
            -DBUILD_WITH_COLLISION_SUPPORT=ON
            -DBUILD_WITH_SDF_SUPPORT=OFF
            -DBUILD_WITH_AUTODIFF_SUPPORT=OFF
            -DBUILD_WITH_CASADI_SUPPORT=OFF
            -DBUILD_WITH_CODEGEN_SUPPORT=OFF
            -DBUILD_WITH_OPENMP_SUPPORT=OFF
            -DBUILD_WITH_EXTRA_SUPPORT=OFF
    )

    # PIN-IK (inverse kinematics)
    aviator_add_third_party_source(pin_ik-2.2.0
        DEPENDS third_party_pinocchio-3.9.0
        LIBRARIES pin_ik
        CMAKE_ARGS
            ${boost_cmake_args}
            -DPIN_IK_BUILD_EXAMPLES=OFF
    )

    # Create interface library for robotics stack
    add_library(aviator_robotics_deps INTERFACE)
    add_dependencies(aviator_robotics_deps aviator_third_party_all)
    target_include_directories(aviator_robotics_deps SYSTEM INTERFACE
        "${AVIATOR_DEPS_DIR}/include"
    )
    target_link_libraries(aviator_robotics_deps INTERFACE
        aviator_system_deps
    )

    # Import built libraries
    foreach(lib_name coal pinocchio_default pinocchio_collision pinocchio_parsers pin_ik)
        add_library(${lib_name}_imported SHARED IMPORTED GLOBAL)
        set_target_properties(${lib_name}_imported PROPERTIES
            IMPORTED_LOCATION "${AVIATOR_DEPS_DIR}/lib/lib${lib_name}.so"
        )
        add_dependencies(${lib_name}_imported third_party_pinocchio-3.9.0 third_party_pin_ik-2.2.0)
        target_link_libraries(aviator_robotics_deps INTERFACE ${lib_name}_imported)
    endforeach()

    # Add compile definitions
    target_compile_definitions(aviator_robotics_deps INTERFACE
        BOOST_MPL_LIMIT_LIST_SIZE=30
        BOOST_MPL_LIMIT_VECTOR_SIZE=30
        PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
        PINOCCHIO_WITH_HPP_FCL
        PINOCCHIO_WITH_URDFDOM
        COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
        COAL_DISABLE_HPP_FCL_WARNINGS
        COAL_HAS_OCTOMAP
        COAL_HAVE_OCTOMAP
        OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
        OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
        OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION}
    )
endif()

#------------------------------------------------------------------------------
# MuJoCo simulation (optional)
#------------------------------------------------------------------------------
option(AVIATOR_BUILD_MUJOCO "Build MuJoCo simulation library" ON)

if(AVIATOR_BUILD_MUJOCO)
    include(${CMAKE_CURRENT_LIST_DIR}/MujocoDependency.cmake)
    aviator_add_mujoco()
endif()

#------------------------------------------------------------------------------
# xCoreSDK (Rokae robot control, optional)
#------------------------------------------------------------------------------
option(AVIATOR_BUILD_XCORE_SDK "Include xCoreSDK for Rokae robot control" ON)

if(AVIATOR_BUILD_XCORE_SDK)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        message(WARNING "xCore SDK only supports Linux x86_64, disabling AVIATOR_BUILD_XCORE_SDK")
        set(AVIATOR_BUILD_XCORE_SDK OFF CACHE BOOL "Include xCoreSDK for Rokae robot control" FORCE)
    else()
        message(STATUS "Including xCoreSDK...")

        # Configure to use static libraries
        set(XCORE_LINK_SHARED_LIBS OFF CACHE BOOL "Use static xCoreSDK libraries" FORCE)
        add_subdirectory("${AVIATOR_VENDOR_DIR}/xCoreSDK-0.7.1" "${CMAKE_BINARY_DIR}/third_party/xCoreSDK" EXCLUDE_FROM_ALL)
    endif()
endif()

#------------------------------------------------------------------------------
# Viewer support (optional)
#------------------------------------------------------------------------------
option(AVIATOR_BUILD_VIEWER "Build GLFW viewer support" ON)

if(AVIATOR_BUILD_VIEWER AND AVIATOR_BUILD_MUJOCO)
    find_package(glfw3 3.3 REQUIRED)
    message(STATUS "GLFW viewer support enabled")
endif()

message(STATUS "Third-party dependencies configuration complete")
