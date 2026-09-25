cmake_minimum_required(VERSION 3.16)
project(AviatorRobot LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

get_filename_component(ROCOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rocos_app" ABSOLUTE)
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

add_executable(aviator_authority_map tools/authority_map.cpp)
target_include_directories(aviator_authority_map PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/include")
target_link_libraries(aviator_authority_map PRIVATE aviator_rocos_core
    "${SIM_DIR}/third_party/mujoco-3.4.0/lib/libmujoco.so.3.4.0")

add_executable(aviator_clearance_trajectory tools/clearance_trajectory.cpp)
target_include_directories(aviator_clearance_trajectory PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/include")
target_link_libraries(aviator_clearance_trajectory PRIVATE aviator_rocos_core
    "${SIM_DIR}/third_party/mujoco-3.4.0/lib/libmujoco.so.3.4.0")

add_custom_target(aviator_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/config" "${CMAKE_BINARY_DIR}/bin/config"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/bin/model"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SIM_DIR}/model/aviator.xml" "${CMAKE_BINARY_DIR}/bin/model/aviator.xml"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SIM_DIR}/model/aviator_meshes" "${CMAKE_BINARY_DIR}/bin/model/aviator_meshes")
add_dependencies(aviatorAppMain aviator_assets)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../methods/t6" "${CMAKE_BINARY_DIR}/t6_method" EXCLUDE_FROM_ALL)
add_executable(aviatorT6Sine src/t6_sine_main.cpp)
target_link_libraries(aviatorT6Sine PRIVATE aviator aviator_t6)
target_compile_definitions(aviatorT6Sine PRIVATE AVIATOR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
add_dependencies(aviatorT6Sine aviator_assets)

# Joystick-driven variant of the t6 controller. The USB joystick library is
# compiled directly into this target (single process, no HTTP round-trip).
set(JOYSTICK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../methods/t6/joystick")
add_executable(aviatorT6Joystick src/t6_joystick_main.cpp "${JOYSTICK_DIR}/src/joystick.cpp")
target_include_directories(aviatorT6Joystick PRIVATE "${JOYSTICK_DIR}/include")
target_link_libraries(aviatorT6Joystick PRIVATE aviator aviator_t6)
target_compile_definitions(aviatorT6Joystick PRIVATE AVIATOR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
add_dependencies(aviatorT6Joystick aviator_assets)
add_executable(aviator_joint_step_probe tests/joint_step_probe.cpp)
target_link_libraries(aviator_joint_step_probe PRIVATE aviator aviator_t6)
add_dependencies(aviator_joint_step_probe aviator_assets)
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
    add_test(NAME aviator_t6_sine_integration
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_t6_sine.py"
            "${AVIATOR_TEST_SIMULATOR}" "$<TARGET_FILE:aviatorT6Sine>"
            "${CMAKE_CURRENT_SOURCE_DIR}/../methods/t6/actor_60k.onnx"
            "${CMAKE_CURRENT_SOURCE_DIR}/../hil-serl/data/aviator/manifold_phi")
    set_tests_properties(aviator_t6_sine_integration PROPERTIES TIMEOUT 90)
    add_test(NAME aviator_t6_wheel50_integration
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_t6_sine.py"
            "${AVIATOR_TEST_SIMULATOR}" "$<TARGET_FILE:aviatorT6Sine>"
            "${CMAKE_CURRENT_SOURCE_DIR}/../methods/t6/actor_60k.onnx"
            "${CMAKE_CURRENT_SOURCE_DIR}/../hil-serl/data/aviator/manifold_phi"
            10 0.1 0.01 0.1)
    set_tests_properties(aviator_t6_wheel50_integration PROPERTIES TIMEOUT 90)
    add_test(NAME aviator_t6_joystick_integration
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_t6_joystick.py"
            "${AVIATOR_TEST_SIMULATOR}" "$<TARGET_FILE:aviatorT6Joystick>"
            "${CMAKE_CURRENT_SOURCE_DIR}/../methods/t6/actor_60k.onnx"
            "${CMAKE_CURRENT_SOURCE_DIR}/../hil-serl/data/aviator/manifold_phi"
            2 /dev/input/js_nonexistent)
    set_tests_properties(aviator_t6_joystick_integration PROPERTIES TIMEOUT 90)
endif()
