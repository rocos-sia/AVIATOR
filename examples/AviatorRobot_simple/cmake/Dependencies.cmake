# Ubuntu 22.04 system packages plus the remaining vendored robotics libraries.
include(ExternalProject)
get_filename_component(AVIATOR_VENDOR "${CMAKE_CURRENT_LIST_DIR}/../../../third_party" ABSOLUTE)
# Versioned source directories; keep logical CMake target names stable.
set(AVIATOR_SOURCE_coal "${AVIATOR_VENDOR}/coal-3.0.4")
set(AVIATOR_SOURCE_mujoco "${AVIATOR_VENDOR}/mujoco-3.4.0")
set(AVIATOR_SOURCE_pinocchio "${AVIATOR_VENDOR}/pinocchio-3.9.0")
set(AVIATOR_SOURCE_pin_ik "${AVIATOR_VENDOR}/pin_ik-2.2.0")
set(AVIATOR_SOURCE_xcore "${AVIATOR_VENDOR}/xcore-0.7.1")
set(AVIATOR_SOURCE_lodepng "${AVIATOR_VENDOR}/lodepng-17d08dd26cac")
set(AVIATOR_SOURCE_marchingcubecpp "${AVIATOR_VENDOR}/marchingcubecpp-f03a1b3ec29b")
set(AVIATOR_SOURCE_trianglemeshdistance "${AVIATOR_VENDOR}/trianglemeshdistance-2cb643de1436")
set(AVIATOR_DEPS "${CMAKE_BINARY_DIR}/dependencies")
set(AVIATOR_DEPENDENCY_JOBS 2 CACHE STRING "Parallel compiler jobs per third-party project")
file(MAKE_DIRECTORY "${AVIATOR_DEPS}/include" "${AVIATOR_DEPS}/include/eigen3" "${AVIATOR_DEPS}/lib")
add_custom_target(aviator_third_party)

# Separate CMake caches avoid upstream projects overwriting each other's options.
function(aviator_source name)
    cmake_parse_arguments(SRC "" "DIRECTORY" "DEPENDS;ARGS;LIBRARIES" ${ARGN})
    if(NOT SRC_DIRECTORY)
        set(SRC_DIRECTORY "${AVIATOR_SOURCE_${name}}")
    endif()
    set(outputs "")
    foreach(library IN LISTS SRC_LIBRARIES)
        list(APPEND outputs "${AVIATOR_DEPS}/lib/lib${library}.so")
    endforeach()
    set(dependency_prefixes "${AVIATOR_DEPS};${CMAKE_PREFIX_PATH}")
    list(REMOVE_DUPLICATES dependency_prefixes)
    string(REPLACE ";" "|" dependency_prefixes "${dependency_prefixes}")
    ExternalProject_Add(source_${name}
        LIST_SEPARATOR |
        SOURCE_DIR "${SRC_DIRECTORY}" BINARY_DIR "${CMAKE_BINARY_DIR}/third_party/${name}"
        PREFIX "${CMAKE_BINARY_DIR}/third_party/stamps/${name}"
        DOWNLOAD_COMMAND "" UPDATE_COMMAND "" PATCH_COMMAND ""
        DEPENDS ${SRC_DEPENDS}
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_INSTALL_PREFIX=${AVIATOR_DEPS} -DCMAKE_INSTALL_LIBDIR=lib
            "-DCMAKE_PREFIX_PATH=${dependency_prefixes}"
            -DCMAKE_INSTALL_RPATH=$ORIGIN -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF
            -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
            -DFETCHCONTENT_FULLY_DISCONNECTED=ON
            -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF ${SRC_ARGS}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --parallel ${AVIATOR_DEPENDENCY_JOBS}
            COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
        INSTALL_COMMAND ""
        BUILD_ALWAYS ON
        BUILD_BYPRODUCTS ${outputs}
        LOG_CONFIGURE ON LOG_BUILD ON LOG_INSTALL ON LOG_OUTPUT_ON_FAILURE ON)
    add_dependencies(aviator_third_party source_${name})
endfunction()

# Fail early with a missing-package diagnostic before starting long source builds.
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
if(AVIATOR_WITH_SIMULATION)
    find_package(Qhull REQUIRED)
    find_package(ccd REQUIRED)
    find_package(tinyobjloader REQUIRED)
endif()
if(AVIATOR_WITH_SIMULATION AND AVIATOR_WITH_VIEWER)
    find_package(glfw3 3.3 REQUIRED)
endif()
# Share one system Boost/Eigen ABI with the source-built robotics stack.
set(boost_args -DBoost_NO_BOOST_CMAKE=ON)
aviator_source(coal
    LIBRARIES coal ARGS ${boost_args} -DBUILD_PYTHON_INTERFACE=OFF
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON -DCOAL_HAS_QHULL=OFF)
aviator_source(pinocchio DEPENDS source_coal LIBRARIES
    pinocchio_default pinocchio_collision pinocchio_parsers
    ARGS ${boost_args} -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_EXAMPLES=OFF
    -DBUILD_BENCHMARK=OFF -DBUILD_UTILS=OFF -DBUILD_WITH_URDF_SUPPORT=ON
    -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_WITH_SDF_SUPPORT=OFF
    -DBUILD_WITH_AUTODIFF_SUPPORT=OFF -DBUILD_WITH_CASADI_SUPPORT=OFF
    -DBUILD_WITH_CODEGEN_SUPPORT=OFF -DBUILD_WITH_OPENMP_SUPPORT=OFF -DBUILD_WITH_EXTRA_SUPPORT=OFF)
aviator_source(pin_ik DEPENDS source_pinocchio LIBRARIES pin_ik
    ARGS ${boost_args} -DPIN_IK_BUILD_EXAMPLES=OFF)

if(AVIATOR_WITH_SIMULATION)
    set(mujoco_args "")
    foreach(dependency lodepng marchingcubecpp trianglemeshdistance)
        string(TOUPPER "${dependency}" upper)
        list(APPEND mujoco_args "-DFETCHCONTENT_SOURCE_DIR_${upper}=${AVIATOR_SOURCE_${dependency}}")
    endforeach()
    aviator_source(mujoco LIBRARIES mujoco ARGS ${mujoco_args}
        -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_STUDIO=OFF
        -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF -DMUJOCO_WITH_USD=OFF
        -DMUJOCO_USE_FILAMENT=OFF)
endif()

# Source-built robotics libraries use this build prefix; common dependencies use apt packages.
add_library(aviator_dependencies INTERFACE)
add_dependencies(aviator_dependencies aviator_third_party)
target_include_directories(aviator_dependencies SYSTEM INTERFACE
    "${AVIATOR_DEPS}/include")
target_link_libraries(aviator_dependencies INTERFACE
    Eigen3::Eigen yaml-cpp ${NLOPT_LIBRARIES} ${AVIATOR_TINYXML_LIBRARY}
    PkgConfig::TINYXML2 ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES}
    Boost::filesystem Boost::system Boost::thread Boost::date_time Boost::serialization)
target_include_directories(aviator_dependencies SYSTEM INTERFACE
    ${urdfdom_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})
foreach(name coal pinocchio_default pinocchio_collision pinocchio_parsers pin_ik)
    add_library(dep_${name} SHARED IMPORTED GLOBAL)
    set_target_properties(dep_${name} PROPERTIES IMPORTED_LOCATION "${AVIATOR_DEPS}/lib/lib${name}.so")
    target_link_libraries(aviator_dependencies INTERFACE dep_${name})
endforeach()
target_compile_definitions(aviator_dependencies INTERFACE
    BOOST_MPL_LIMIT_LIST_SIZE=30 BOOST_MPL_LIMIT_VECTOR_SIZE=30
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION PINOCCHIO_WITH_HPP_FCL PINOCCHIO_WITH_URDFDOM
    COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL COAL_DISABLE_HPP_FCL_WARNINGS
    COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
    OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
