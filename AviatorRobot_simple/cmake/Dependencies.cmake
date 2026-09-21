# Offline dependency bundle. No sibling project or system development package lookup.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(FATAL_ERROR "Bundled dependencies target Linux x86_64 (Ubuntu 22.04). See third_party/README.md.")
endif()
set(AVIATOR_DEPS "${CMAKE_BINARY_DIR}/dependencies")
set(_archive "${PROJECT_SOURCE_DIR}/third_party/linux-x86_64.tar.gz")
file(READ "${PROJECT_SOURCE_DIR}/third_party/linux-x86_64.sha256" _expected)
string(STRIP "${_expected}" _expected)
file(SHA256 "${_archive}" _actual)
if(NOT _actual STREQUAL _expected)
    message(FATAL_ERROR "Dependency archive checksum mismatch")
endif()
set(_stamp "${AVIATOR_DEPS}/archive.sha256")
set(_installed "")
if(EXISTS "${_stamp}")
    file(READ "${_stamp}" _installed)
endif()
if(NOT _installed STREQUAL _actual)
    file(REMOVE_RECURSE "${AVIATOR_DEPS}")
    file(MAKE_DIRECTORY "${AVIATOR_DEPS}")
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf "${_archive}"
        WORKING_DIRECTORY "${AVIATOR_DEPS}" RESULT_VARIABLE _result)
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Cannot extract dependencies")
    endif()
    file(WRITE "${_stamp}" "${_actual}")
endif()
add_library(aviator_dependencies INTERFACE)
target_include_directories(aviator_dependencies SYSTEM INTERFACE
    "${AVIATOR_DEPS}/include" "${AVIATOR_DEPS}/include/eigen3")
foreach(_name yaml-cpp orocos-kdl tinyxml tinyxml2 urdfdom_model console_bridge
        boost_filesystem boost_system boost_thread boost_date_time boost_serialization
        nlopt coal pinocchio_default pinocchio_collision pinocchio_parsers)
    add_library(dep_${_name} SHARED IMPORTED GLOBAL)
    set_target_properties(dep_${_name} PROPERTIES IMPORTED_LOCATION "${AVIATOR_DEPS}/lib/lib${_name}.so")
    target_link_libraries(aviator_dependencies INTERFACE dep_${_name})
endforeach()
target_compile_definitions(aviator_dependencies INTERFACE
    BOOST_MPL_LIMIT_LIST_SIZE=30 BOOST_MPL_LIMIT_VECTOR_SIZE=30
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION PINOCCHIO_WITH_HPP_FCL PINOCCHIO_WITH_URDFDOM
    COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL COAL_DISABLE_HPP_FCL_WARNINGS
    COAL_HAS_OCTOMAP COAL_HAVE_OCTOMAP OCTOMAP_MAJOR_VERSION=1 OCTOMAP_MINOR_VERSION=9 OCTOMAP_PATCH_VERSION=7)
add_library(kdl_parser STATIC "${PROJECT_SOURCE_DIR}/third_party/kdl_parser/src/kdl_parser.cpp")
target_include_directories(kdl_parser PUBLIC "${PROJECT_SOURCE_DIR}/third_party/kdl_parser/include")
target_link_libraries(kdl_parser PUBLIC aviator_dependencies)
add_library(trac_ik STATIC
    "${PROJECT_SOURCE_DIR}/third_party/trac_ik/src/kdl_tl.cpp"
    "${PROJECT_SOURCE_DIR}/third_party/trac_ik/src/nlopt_ik.cpp"
    "${PROJECT_SOURCE_DIR}/third_party/trac_ik/src/trac_ik.cpp")
target_include_directories(trac_ik PUBLIC "${PROJECT_SOURCE_DIR}/third_party/trac_ik/include")
target_link_libraries(trac_ik PUBLIC kdl_parser Threads::Threads)
