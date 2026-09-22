cmake_minimum_required(VERSION 3.16)
project(pin_ik VERSION 2.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CTest)
include(CMakePackageConfigHelpers)

# Import Pinocchio and its transitive targets in the scope shared by all subdirectories.
find_package(pinocchio REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(Threads REQUIRED)

option(PIN_IK_BUILD_EXAMPLES "Build standalone IK example" ON)

add_library(pin_ik SHARED src/pinocchio_tl.cpp src/nlopt_ik.cpp src/pin_ik.cpp src/urdf.cpp)
add_library(pin_ik::pin_ik ALIAS pin_ik)
# Keep non-system dependency prefixes discoverable after installation.
set_target_properties(pin_ik PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
target_compile_features(pin_ik PUBLIC cxx_std_17)
target_include_directories(pin_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pin_ik PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  ${NLOPT_LIBRARIES}
  Threads::Threads
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS pin_ik EXPORT pin_ikTargets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT pin_ikTargets NAMESPACE pin_ik:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pin_ik)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/pin_ikConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/pin_ikConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pin_ik)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/pin_ikConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pin_ikConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/pin_ikConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pin_ik)

if(PIN_IK_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
