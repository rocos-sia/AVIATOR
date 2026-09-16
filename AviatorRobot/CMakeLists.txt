cmake_minimum_required(VERSION 3.16)
project(AviatorRobot LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

get_filename_component(ROCOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../reference/RCMRobot" ABSOLUTE)
get_filename_component(SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../reference/rocos-mujoco" ABSOLUTE)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system chrono thread)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(orocos_kdl REQUIRED)

set(NLOPT_PYTHON OFF CACHE BOOL "" FORCE)
set(NLOPT_OCTAVE OFF CACHE BOOL "" FORCE)
set(NLOPT_MATLAB OFF CACHE BOOL "" FORCE)
set(NLOPT_GUILE OFF CACHE BOOL "" FORCE)
set(NLOPT_SWIG OFF CACHE BOOL "" FORCE)
set(NLOPT_TESTS OFF CACHE BOOL "" FORCE)
foreach(dependency nlopt kdl_parser trac_ik ruckig plog gripper spdlog sml)
    add_subdirectory("${ROCOS_DIR}/3rdparty/${dependency}" "dependencies/${dependency}" EXCLUDE_FROM_ALL)
endforeach()

# Compile the same Robot/Hardware sources with the MuJoCo POSIX backend.
# The original RCM build still selects its original ECM implementation.
set(ROCOS_SOURCES logger.cc command.cc status_word.cc control_word.cc drive_state.cc
    hardware.cc hardware_interface.cc hardware_sim.cc ft_sensor_interface.cc ft_sensor_ecat.cc
    drive.cc drive_guard.cc interpolate.cc robotMath.cc kinematics.cc dynamics.cc robot.cc
    UnitIntervalMotionProfile.cpp JC_helper_kinematics.cpp JC_helper_dynamics.cpp DHParamsLoader.cpp)
list(TRANSFORM ROCOS_SOURCES PREPEND "${ROCOS_DIR}/src/")
add_library(aviator_rocos_core SHARED ${ROCOS_SOURCES})
target_include_directories(aviator_rocos_core PUBLIC "${ROCOS_DIR}/include" "${SIM_DIR}/include"
    PRIVATE "${ROCOS_DIR}")
target_compile_definitions(aviator_rocos_core PUBLIC ROCOS_MUJOCO_BACKEND=1)
target_link_libraries(aviator_rocos_core PUBLIC yaml-cpp Eigen3::Eigen orocos-kdl
    Boost::date_time Boost::filesystem Boost::system Boost::chrono Boost::thread
    Threads::Threads trac_ik kdl_parser ruckig::ruckig plog::plog JC_helper::gripper
    spdlog::spdlog_header_only sml::sml OpenSSL::SSL dl rt)

add_library(aviator SHARED src/Aviator.cpp)
target_include_directories(aviator PUBLIC include)
target_link_libraries(aviator PUBLIC aviator_rocos_core)
target_include_directories(aviator PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/include")
target_link_libraries(aviator PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/lib/libmujoco.so.3.4.0")
add_executable(aviatorAppMain src/main.cpp)
target_link_libraries(aviatorAppMain PRIVATE aviator)
target_compile_definitions(aviatorAppMain PRIVATE AVIATOR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

add_executable(aviator_grasp_search tools/search_grasp.cpp)
target_include_directories(aviator_grasp_search PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/include")
target_link_libraries(aviator_grasp_search PRIVATE aviator_rocos_core
    "${SIM_DIR}/third_party/mujoco-3.4.0/lib/libmujoco.so.3.4.0")

add_custom_target(aviator_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/config" "${CMAKE_BINARY_DIR}/bin/config"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/bin/model"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SIM_DIR}/model/aviator.xml" "${CMAKE_BINARY_DIR}/bin/model/aviator.xml"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SIM_DIR}/model/aviator_meshes" "${CMAKE_BINARY_DIR}/bin/model/aviator_meshes")
add_dependencies(aviatorAppMain aviator_assets)
include(CTest)
if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_executable(aviator_integration_test tests/control_integration.cpp)
    target_link_libraries(aviator_integration_test PRIVATE aviator)
    set(AVIATOR_TEST_SIMULATOR "${SIM_DIR}/build/aviator-headless/bin/rocos_mujoco_sim"
        CACHE FILEPATH "MuJoCo simulator executable for the integration test")
    add_test(NAME aviator_dual_arm_integration
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_integration.py"
            "${AVIATOR_TEST_SIMULATOR}" "$<TARGET_FILE:aviator_integration_test>"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/aviator.yaml")
    set_tests_properties(aviator_dual_arm_integration PROPERTIES TIMEOUT 180)
endif()
