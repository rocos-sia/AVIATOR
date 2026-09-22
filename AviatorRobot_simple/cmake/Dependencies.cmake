# All open-source dependencies are built from checked-in source; no download steps.
include(ExternalProject)
set(AVIATOR_VENDOR "${PROJECT_SOURCE_DIR}/third_party")
set(AVIATOR_DEPS "${CMAKE_BINARY_DIR}/dependencies")
set(AVIATOR_DEPENDENCY_JOBS 2 CACHE STRING "Parallel compiler jobs per third-party project")
file(MAKE_DIRECTORY "${AVIATOR_DEPS}/include" "${AVIATOR_DEPS}/include/eigen3" "${AVIATOR_DEPS}/lib")
add_custom_target(aviator_third_party)

# Separate CMake caches avoid upstream projects overwriting each other's options.
function(aviator_source name)
    cmake_parse_arguments(SRC "" "DIRECTORY" "DEPENDS;ARGS;LIBRARIES" ${ARGN})
    if(NOT SRC_DIRECTORY)
        set(SRC_DIRECTORY "${AVIATOR_VENDOR}/${name}")
    endif()
    set(outputs "")
    foreach(library IN LISTS SRC_LIBRARIES)
        list(APPEND outputs "${AVIATOR_DEPS}/lib/lib${library}.so")
    endforeach()
    ExternalProject_Add(source_${name}
        SOURCE_DIR "${SRC_DIRECTORY}" BINARY_DIR "${CMAKE_BINARY_DIR}/third_party/${name}"
        PREFIX "${CMAKE_BINARY_DIR}/third_party/stamps/${name}"
        DOWNLOAD_COMMAND "" UPDATE_COMMAND "" PATCH_COMMAND ""
        DEPENDS ${SRC_DEPENDS}
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCMAKE_INSTALL_PREFIX=${AVIATOR_DEPS} -DCMAKE_INSTALL_LIBDIR=lib
            -DCMAKE_PREFIX_PATH=${AVIATOR_DEPS}
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

aviator_source(eigen ARGS -DBUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF)
ExternalProject_Add(source_boost
    SOURCE_DIR "${AVIATOR_VENDOR}/boost" BINARY_DIR "${CMAKE_BINARY_DIR}/third_party/boost"
    PREFIX "${CMAKE_BINARY_DIR}/third_party/stamps/boost"
    DOWNLOAD_COMMAND "" UPDATE_COMMAND "" PATCH_COMMAND "" CONFIGURE_COMMAND ""
    BUILD_COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=<SOURCE_DIR> -DBINARY_DIR=<BINARY_DIR>
        -DPREFIX=${AVIATOR_DEPS} -DCXX=${CMAKE_CXX_COMPILER} -DJOBS=${AVIATOR_DEPENDENCY_JOBS}
        -P "${CMAKE_CURRENT_LIST_DIR}/BuildBoost.cmake"
    INSTALL_COMMAND ""
    BUILD_ALWAYS ON
    BUILD_BYPRODUCTS "${AVIATOR_DEPS}/lib/libboost_filesystem.so"
        "${AVIATOR_DEPS}/lib/libboost_system.so" "${AVIATOR_DEPS}/lib/libboost_thread.so"
        "${AVIATOR_DEPS}/lib/libboost_date_time.so" "${AVIATOR_DEPS}/lib/libboost_serialization.so"
    LOG_BUILD ON LOG_OUTPUT_ON_FAILURE ON)
add_dependencies(aviator_third_party source_boost)

aviator_source(yaml-cpp LIBRARIES yaml-cpp ARGS -DYAML_CPP_BUILD_TESTS=OFF
    -DYAML_CPP_BUILD_TOOLS=OFF -DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_BUILD_SHARED_LIBS=ON)
aviator_source(nlopt LIBRARIES nlopt ARGS -DNLOPT_PYTHON=OFF -DNLOPT_OCTAVE=OFF
    -DNLOPT_MATLAB=OFF -DNLOPT_GUILE=OFF -DNLOPT_SWIG=OFF -DNLOPT_TESTS=OFF)
aviator_source(tinyxml LIBRARIES tinyxml)
aviator_source(tinyxml2 LIBRARIES tinyxml2 ARGS -Dtinyxml2_BUILD_TESTING=OFF)
aviator_source(console_bridge LIBRARIES console_bridge)
aviator_source(urdfdom_headers)
aviator_source(urdfdom DEPENDS source_tinyxml source_console_bridge source_urdfdom_headers
    LIBRARIES urdfdom_model ARGS -DTinyXML_ROOT_DIR=${AVIATOR_DEPS})
aviator_source(zlib LIBRARIES z ARGS -DZLIB_BUILD_EXAMPLES=OFF)
aviator_source(assimp DEPENDS source_zlib LIBRARIES assimp ARGS -DASSIMP_BUILD_TESTS=OFF
    -DASSIMP_BUILD_ASSIMP_TOOLS=OFF -DASSIMP_BUILD_SAMPLES=OFF -DASSIMP_BUILD_DRACO=OFF
    -DASSIMP_BUILD_ZLIB=OFF -DASSIMP_BUILD_MINIZIP=ON -DASSIMP_NO_EXPORT=ON
    -DASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT=OFF
    -DASSIMP_BUILD_STL_IMPORTER=ON -DASSIMP_BUILD_OBJ_IMPORTER=ON)
aviator_source(octomap LIBRARIES octomap octomath ARGS -DBUILD_OCTOVIS_SUBPROJECT=OFF
    -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF)
set(boost_args -DBOOST_ROOT=${AVIATOR_DEPS} -DBoost_NO_SYSTEM_PATHS=ON
    -DBoost_NO_BOOST_CMAKE=ON -DBOOST_INCLUDEDIR=${AVIATOR_DEPS}/include
    -DBOOST_LIBRARYDIR=${AVIATOR_DEPS}/lib)
aviator_source(coal DEPENDS source_eigen source_boost source_assimp source_octomap
    LIBRARIES coal ARGS ${boost_args} -DBUILD_PYTHON_INTERFACE=OFF
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON -DCOAL_HAS_QHULL=OFF)
aviator_source(pinocchio DEPENDS source_coal source_urdfdom LIBRARIES
    pinocchio_default pinocchio_collision pinocchio_parsers
    ARGS ${boost_args} -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_EXAMPLES=OFF
    -DBUILD_BENCHMARK=OFF -DBUILD_UTILS=OFF -DBUILD_WITH_URDF_SUPPORT=ON
    -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_WITH_SDF_SUPPORT=OFF
    -DBUILD_WITH_AUTODIFF_SUPPORT=OFF -DBUILD_WITH_CASADI_SUPPORT=OFF
    -DBUILD_WITH_CODEGEN_SUPPORT=OFF -DBUILD_WITH_OPENMP_SUPPORT=OFF -DBUILD_WITH_EXTRA_SUPPORT=OFF)
aviator_source(pin_ik DEPENDS source_pinocchio source_nlopt LIBRARIES pin_ik
    ARGS ${boost_args} -DPIN_IK_BUILD_EXAMPLES=OFF)

if(AVIATOR_WITH_SIMULATION)
    set(mujoco_args "")
    foreach(dependency lodepng marchingcubecpp qhull tinyxml2 tinyobjloader trianglemeshdistance ccd)
        string(TOUPPER "${dependency}" upper)
        list(APPEND mujoco_args "-DFETCHCONTENT_SOURCE_DIR_${upper}=${AVIATOR_VENDOR}/mujoco_deps/${dependency}")
    endforeach()
    aviator_source(mujoco LIBRARIES mujoco ARGS ${mujoco_args}
        -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_STUDIO=OFF
        -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF -DMUJOCO_WITH_USD=OFF
        -DMUJOCO_USE_FILAMENT=OFF)
    if(AVIATOR_WITH_VIEWER)
        aviator_source(glfw LIBRARIES glfw ARGS -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF
            -DGLFW_BUILD_DOCS=OFF -DGLFW_INSTALL=ON)
    endif()
endif()

# Application targets use only this build's headers/libraries, never a sibling project install.
add_library(aviator_dependencies INTERFACE)
add_dependencies(aviator_dependencies aviator_third_party)
target_include_directories(aviator_dependencies SYSTEM INTERFACE
    "${AVIATOR_DEPS}/include" "${AVIATOR_DEPS}/include/eigen3")
foreach(name yaml-cpp tinyxml tinyxml2 urdfdom_model console_bridge
        boost_filesystem boost_system boost_thread boost_date_time boost_serialization
        nlopt coal pinocchio_default pinocchio_collision pinocchio_parsers pin_ik)
    add_library(dep_${name} SHARED IMPORTED GLOBAL)
    set_target_properties(dep_${name} PROPERTIES IMPORTED_LOCATION "${AVIATOR_DEPS}/lib/lib${name}.so")
    target_link_libraries(aviator_dependencies INTERFACE dep_${name})
endforeach()
target_compile_definitions(aviator_dependencies INTERFACE
    BOOST_MPL_LIMIT_LIST_SIZE=30 BOOST_MPL_LIMIT_VECTOR_SIZE=30
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION PINOCCHIO_WITH_HPP_FCL PINOCCHIO_WITH_URDFDOM
    COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL COAL_DISABLE_HPP_FCL_WARNINGS
    COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=1 OCTOMAP_MINOR_VERSION=9 OCTOMAP_PATCH_VERSION=7)
