# One version source: project(AVIATOR VERSION ...) in the root CMakeLists.txt.
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
set(AVIATOR_RELEASE_PROFILE "" CACHE STRING "Release identity (auto-detected when empty)")
if(AVIATOR_RELEASE_PROFILE)
  set(_profile "${AVIATOR_RELEASE_PROFILE}")
elseif(AVIATOR_COMMUNICATION_ONLY)
  if(AVIATOR_BUILD_SIMULATION_NODE)
    set(_profile "simulation")
  else()
    set(_profile "communication")
  endif()
elseif(AVIATOR_BUILD_SIMULATION_NODE)
  set(_profile "hardware-simulation")
else()
  set(_profile "hardware")
endif()
set(_release_tool "${PROJECT_SOURCE_DIR}/scripts/aviator_release.py")
set(_release_info "${PROJECT_BINARY_DIR}/generated/release-build.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_release_tool}")
set(_features)
foreach(name AVIATOR_COMMUNICATION_ONLY AVIATOR_BUILD_ROBOTICS_LIBS AVIATOR_BUILD_MUJOCO
    AVIATOR_BUILD_SIMULATION_NODE AVIATOR_BUILD_XCORE_SDK AVIATOR_BUILD_EXAMPLES)
  list(APPEND _features --feature "${name}=${${name}}")
endforeach()
set(_dependencies)
foreach(directory "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/common"
    "${PROJECT_SOURCE_DIR}/nodes/flight_gateway")
  foreach(name Eigen3_VERSION Boost_VERSION_STRING yaml-cpp_VERSION spdlog_VERSION
      nlohmann_json_VERSION Protobuf_VERSION urdfdom_VERSION FREETYPE_VERSION_STRING SDL2_VERSION
      AVIATOR_VIDEO_libavcodec_VERSION AVIATOR_VIDEO_libavutil_VERSION
      AVIATOR_VIDEO_libswscale_VERSION AVIATOR_VIDEO_libzstd_VERSION)
    get_directory_property(_value DIRECTORY "${directory}" DEFINITION "${name}")
    if(_value)
      list(APPEND _dependencies --dependency "${name}=${_value}")
    endif()
  endforeach()
endforeach()
execute_process(COMMAND "${Python3_EXECUTABLE}" "${_release_tool}" configure
  --version "${PROJECT_VERSION}" --profile "${_profile}" --output "${_release_info}"
  --compiler "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
  --cmake-version "${CMAKE_VERSION}" ${_features} ${_dependencies}
  COMMAND_ERROR_IS_FATAL ANY)
add_library(aviator_version INTERFACE)
target_include_directories(aviator_version INTERFACE "${PROJECT_BINARY_DIR}/generated")
target_link_libraries(aviator_logging INTERFACE aviator_version)
install(FILES "${PROJECT_BINARY_DIR}/generated/aviator/version.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aviator")

# Runtime resources shared by the selected components, including Python imports.
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/config")
install(PROGRAMS "${_release_tool}" DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/scripts")
if(TARGET aviator_core_control)
  install(PROGRAMS "${PROJECT_SOURCE_DIR}/scripts/start_aviator.sh"
    "${PROJECT_SOURCE_DIR}/scripts/start_camera_servo.sh"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/scripts")
endif()

# Explicit installation also regenerates metadata, after RPATH/strip operations.
configure_file("${CMAKE_CURRENT_LIST_DIR}/ReleaseInstall.cmake.in"
  "${PROJECT_BINARY_DIR}/generated/ReleaseInstall.cmake" @ONLY)
install(SCRIPT "${PROJECT_BINARY_DIR}/generated/ReleaseInstall.cmake")

# Depend on buildable project targets before installing; do not recursively invoke
# a build from inside Ninja/Make. Imported and EXCLUDE_FROM_ALL targets are skipped.
function(aviator_release_dependencies directory)
  get_property(_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
  foreach(target IN LISTS _targets)
    get_target_property(_type "${target}" TYPE)
    get_target_property(_excluded "${target}" EXCLUDE_FROM_ALL)
    if(NOT _excluded AND _type MATCHES "^(EXECUTABLE|SHARED_LIBRARY|STATIC_LIBRARY|MODULE_LIBRARY)$")
      add_dependencies(aviator_release "${target}")
    endif()
  endforeach()
  get_property(_children DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(child IN LISTS _children)
    get_property(_excluded DIRECTORY "${child}" PROPERTY EXCLUDE_FROM_ALL)
    if(NOT _excluded)
      aviator_release_dependencies("${child}")
    endif()
  endforeach()
endfunction()
add_custom_target(aviator_release ALL
  COMMAND "${Python3_EXECUTABLE}" "${_release_tool}" package
    --cmake "${CMAKE_COMMAND}" --build "${PROJECT_BINARY_DIR}"
    --prefix "${CMAKE_INSTALL_PREFIX}" --config "$<CONFIG>"
  COMMENT "Installing AVIATOR and verifying SHA256SUMS" VERBATIM USES_TERMINAL)
aviator_release_dependencies("${PROJECT_SOURCE_DIR}")
if(TARGET aviator_third_party_all)
  add_dependencies(aviator_release aviator_third_party_all)
endif()
if(BUILD_TESTING)
  add_test(NAME release_metadata COMMAND "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/tests/release_test.py")
endif()
