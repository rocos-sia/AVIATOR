aviator_add_test_executable(aviator_communication_test communication_test.cpp)
target_link_libraries(aviator_communication_test PRIVATE
  aviator_transport aviator_runtime)
add_test(NAME communication COMMAND aviator_communication_test)
set_tests_properties(communication PROPERTIES TIMEOUT 20)

find_package(Threads REQUIRED)
aviator_add_test_executable(aviator_bus_test bus_test.cpp)
target_link_libraries(aviator_bus_test PRIVATE aviator_transport Threads::Threads)
add_dependencies(aviator_bus_test aviator_bus)
add_test(NAME bus_process COMMAND aviator_bus_test $<TARGET_FILE:aviator_bus>)
set_tests_properties(bus_process PROPERTIES TIMEOUT 20)
add_test(NAME bus_help COMMAND aviator_bus --help)
set_tests_properties(bus_help PROPERTIES TIMEOUT 5)

aviator_add_test_executable(aviator_gateway_test gateway_test.cpp ../nodes/flight_gateway/gateway.cpp)
target_include_directories(aviator_gateway_test PRIVATE ../nodes/flight_gateway)
target_link_libraries(aviator_gateway_test PRIVATE aviator_runtime)
add_test(NAME gateway COMMAND aviator_gateway_test)
add_test(NAME gateway_help COMMAND flight_gateway --help)
add_test(NAME gateway_legacy_options COMMAND flight_gateway --device /dev/null)
set_tests_properties(gateway gateway_help PROPERTIES TIMEOUT 5)
set_tests_properties(gateway_legacy_options PROPERTIES TIMEOUT 5 WILL_FAIL TRUE)
aviator_add_test_executable(aviator_gateway_config_test gateway_config_test.cpp ../nodes/flight_gateway/config.cpp)
target_include_directories(aviator_gateway_config_test PRIVATE ../nodes/flight_gateway)
target_compile_definitions(aviator_gateway_config_test PRIVATE AVIATOR_FLIGHT_CONFIG="${PROJECT_SOURCE_DIR}/config/flight.yaml")
target_link_libraries(aviator_gateway_config_test PRIVATE aviator_service yaml-cpp)
add_test(NAME gateway_config COMMAND aviator_gateway_config_test ${PROJECT_SOURCE_DIR}/config/flight.yaml)
set_tests_properties(gateway_config PROPERTIES TIMEOUT 5)

aviator_add_test_executable(aviator_monitor_test monitor_test.cpp)
target_include_directories(aviator_monitor_test PRIVATE ../nodes/aviator_monitor)
target_link_libraries(aviator_monitor_test PRIVATE aviator_monitor_state)
aviator_add_test_executable(aviator_monitor_overview_test monitor_overview_test.cpp)
target_link_libraries(aviator_monitor_overview_test PRIVATE aviator_monitor_state)
add_test(NAME monitor_overview COMMAND aviator_monitor_overview_test
  ${PROJECT_SOURCE_DIR}/tests/fixtures/monitor_preview.jpg
  ${PROJECT_SOURCE_DIR}/nodes/aviator_monitor/web ${PROJECT_SOURCE_DIR}/models)
set_tests_properties(monitor_overview PROPERTIES TIMEOUT 5)
add_test(NAME monitor_state COMMAND aviator_monitor_test)
add_test(NAME monitor_help COMMAND aviator_monitor --help)
set_tests_properties(monitor_state monitor_help PROPERTIES TIMEOUT 5)
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME monitor_http COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/monitor_http_test.py
    $<TARGET_FILE:aviator_monitor> $<TARGET_FILE:aviator_monitor_test>)
  set_tests_properties(monitor_http PROPERTIES TIMEOUT 20)
endif()

aviator_add_test_executable(aviator_recording_test recording_test.cpp ../nodes/aviator_logger/logger.cpp ../nodes/aviator_logger/arm_target_recorder.cpp)
target_include_directories(aviator_recording_test PRIVATE ../nodes/aviator_logger)
target_include_directories(aviator_recording_test SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/mcap-2.1.3/include)
target_compile_definitions(aviator_recording_test PRIVATE MCAP_COMPRESSION_NO_ZSTD MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(aviator_recording_test PRIVATE aviator_recording aviator_transport aviator_runtime)
add_test(NAME recording COMMAND aviator_recording_test)
add_test(NAME logger_help COMMAND aviator_logger --help)
add_test(NAME logger_invalid_queue COMMAND aviator_logger --queue-bytes 0)
set_tests_properties(recording PROPERTIES TIMEOUT 20)
set_tests_properties(logger_help logger_invalid_queue PROPERTIES TIMEOUT 5)
set_tests_properties(logger_invalid_queue PROPERTIES WILL_FAIL TRUE)
if(Python3_Interpreter_FOUND)
  add_test(NAME logger_process COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/logger_process_test.py $<TARGET_FILE:aviator_logger>)
  set_tests_properties(logger_process PROPERTIES TIMEOUT 20)
endif()

aviator_add_test_executable(aviator_camera_recording_test camera_recording_test.cpp ../nodes/aviator_logger/logger.cpp ../nodes/aviator_logger/arm_target_recorder.cpp)
target_include_directories(aviator_camera_recording_test PRIVATE ../nodes/aviator_logger)
target_include_directories(aviator_camera_recording_test SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/mcap-2.1.3/include)
target_compile_definitions(aviator_camera_recording_test PRIVATE MCAP_COMPRESSION_NO_ZSTD MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(aviator_camera_recording_test PRIVATE aviator_recording aviator_transport aviator_runtime PkgConfig::AVIATOR_VIDEO)
add_test(NAME camera_recording COMMAND aviator_camera_recording_test ${PROJECT_SOURCE_DIR}/config/recording.yaml)
set_tests_properties(camera_recording PROPERTIES TIMEOUT 40)

if(Python3_Interpreter_FOUND)
  add_test(NAME camera_adapter COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_adapter_test.py $<TARGET_FILE:aviator_camera_recording_test>
    ${PROJECT_SOURCE_DIR}/nodes/camera/recording_client.py)
  set_tests_properties(camera_adapter PROPERTIES TIMEOUT 10)
endif()

if(TARGET aviator_core_servo AND TARGET manipulator AND AVIATOR_BUILD_MUJOCO)
  aviator_add_test_executable(aviator_control_nodes_test control_nodes_test.cpp
    ../nodes/flight_gateway/gateway.cpp)
  target_include_directories(aviator_control_nodes_test PRIVATE ../nodes/flight_gateway)
  target_link_libraries(aviator_control_nodes_test PRIVATE aviator_motion_protocol)
  add_dependencies(aviator_control_nodes_test aviator_bus aviator_core aviator_core_servo manipulator)
  foreach(mode IN ITEMS flight flight_auto flight_hold)
    add_test(NAME control_nodes_${mode} COMMAND aviator_control_nodes_test
      $<TARGET_FILE:aviator_bus> $<TARGET_FILE:manipulator> $<TARGET_FILE:aviator_core_servo>
      ${PROJECT_SOURCE_DIR} ${mode})
    set_tests_properties(control_nodes_${mode} PROPERTIES TIMEOUT 190 ENVIRONMENT "LD_BIND_NOW=1")
  endforeach()
  add_test(NAME control_nodes_watchdog COMMAND aviator_control_nodes_test
    $<TARGET_FILE:aviator_bus> $<TARGET_FILE:manipulator> $<TARGET_FILE:aviator_core>
    ${PROJECT_SOURCE_DIR} watchdog)
  set_tests_properties(control_nodes_watchdog PROPERTIES TIMEOUT 190 ENVIRONMENT "LD_BIND_NOW=1")
  if(Python3_Interpreter_FOUND)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import zmq, yaml"
      RESULT_VARIABLE FSM_PYTHON_DEPS OUTPUT_QUIET ERROR_QUIET)
    if(FSM_PYTHON_DEPS EQUAL 0)
      add_test(NAME robot_state_machine_process COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/robot_state_machine_process_test.py
        $<TARGET_FILE:aviator_bus> $<TARGET_FILE:manipulator> $<TARGET_FILE:aviator_core_managed> ${PROJECT_SOURCE_DIR})
      set_tests_properties(robot_state_machine_process PROPERTIES TIMEOUT 220)
      add_test(NAME managed_gateway_process COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/managed_gateway_process_test.py
        $<TARGET_FILE:aviator_bus> $<TARGET_FILE:manipulator> $<TARGET_FILE:aviator_core_managed> ${PROJECT_SOURCE_DIR})
      set_tests_properties(managed_gateway_process PROPERTIES TIMEOUT 300)
      add_test(NAME managed_idle_startup COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/managed_startup_test.py
        $<TARGET_FILE:aviator_bus> $<TARGET_FILE:aviator_core_managed> ${PROJECT_SOURCE_DIR})
      set_tests_properties(managed_idle_startup PROPERTIES TIMEOUT 30)
    endif()
  endif()
endif()

if(TARGET aviator_robot_state_machine)
  aviator_add_test_executable(aviator_robot_state_machine_test robot_state_machine_test.cpp)
  target_link_libraries(aviator_robot_state_machine_test PRIVATE aviator_robot_state_machine)
  add_test(NAME robot_state_machine COMMAND aviator_robot_state_machine_test)
  add_test(NAME core_sml_demo COMMAND aviator_core_sml --demo)
  if(Python3_Interpreter_FOUND)
    add_test(NAME core_sml_cli COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/core_sml_test.py $<TARGET_FILE:aviator_core_sml>)
    set_tests_properties(core_sml_cli PROPERTIES TIMEOUT 10)
  endif()
endif()

if(TARGET aviator_core_control)
  aviator_add_test_executable(aviator_managed_test aviator_managed_test.cpp)
  target_link_libraries(aviator_managed_test PRIVATE aviator_core_control)
  add_test(NAME aviator_managed_api COMMAND aviator_managed_test)
  set_tests_properties(aviator_managed_api PROPERTIES TIMEOUT 10)
  aviator_add_test_executable(aviator_servo_planner_test servo_planner_test.cpp)
  target_link_libraries(aviator_servo_planner_test PRIVATE aviator_core_control ruckig::ruckig)
  add_test(NAME servo_planner COMMAND aviator_servo_planner_test ${PROJECT_SOURCE_DIR})
  set_tests_properties(servo_planner PROPERTIES TIMEOUT 60)
  aviator_add_test_executable(aviator_grasp_tools_test grasp_tools_test.cpp)
  target_link_libraries(aviator_grasp_tools_test PRIVATE aviator_core_control aviator_device)
  add_test(NAME grasp_tools COMMAND aviator_grasp_tools_test ${PROJECT_SOURCE_DIR})
  set_tests_properties(grasp_tools PROPERTIES TIMEOUT 20)
  aviator_add_test_executable(aviator_motion_protocol_test motion_protocol_test.cpp)
  target_link_libraries(aviator_motion_protocol_test PRIVATE aviator_motion_protocol)
  add_test(NAME motion_protocol COMMAND aviator_motion_protocol_test)
endif()

aviator_add_test_executable(aviator_service_test service_test.cpp
  ../nodes/flight_gateway/gateway.cpp ../nodes/aviator_logger/logger.cpp ../nodes/aviator_logger/arm_target_recorder.cpp)
target_include_directories(aviator_service_test PRIVATE ../nodes/flight_gateway ../nodes/aviator_logger)
target_include_directories(aviator_service_test SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/mcap-2.1.3/include)
target_compile_definitions(aviator_service_test PRIVATE MCAP_COMPRESSION_NO_ZSTD MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(aviator_service_test PRIVATE aviator_service aviator_recording)
add_test(NAME service COMMAND aviator_service_test)
set_tests_properties(service PROPERTIES TIMEOUT 15)

if(TARGET aviator_core_control)
  aviator_add_test_executable(aviator_core_hand_test core_hand_test.cpp)
  target_link_libraries(aviator_core_hand_test PRIVATE aviator_core_control)
  add_test(NAME core_hand COMMAND aviator_core_hand_test)
  aviator_add_test_executable(aviator_core_hand_link_driver core_hand_link_driver.cpp)
  target_link_libraries(aviator_core_hand_link_driver PRIVATE aviator_core_control)
  if(Python3_Interpreter_FOUND AND FSM_PYTHON_DEPS EQUAL 0)
    add_test(NAME core_hand_process COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/core_hand_process_test.py
      $<TARGET_FILE:aviator_core_hand_link_driver> $<TARGET_FILE:aviator_bus>)
    set_tests_properties(core_hand_process PROPERTIES TIMEOUT 30)
  endif()
endif()

aviator_add_test_executable(aviator_arm_target_recorder_test arm_target_recorder_test.cpp
  ../nodes/aviator_logger/logger.cpp ../nodes/aviator_logger/arm_target_recorder.cpp)
target_include_directories(aviator_arm_target_recorder_test PRIVATE ../nodes/aviator_logger)
target_include_directories(aviator_arm_target_recorder_test SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/mcap-2.1.3/include)
target_compile_definitions(aviator_arm_target_recorder_test PRIVATE MCAP_COMPRESSION_NO_ZSTD MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(aviator_arm_target_recorder_test PRIVATE aviator_recording aviator_transport aviator_runtime Threads::Threads)
add_test(NAME arm_target_recorder COMMAND aviator_arm_target_recorder_test)
set_tests_properties(arm_target_recorder PROPERTIES TIMEOUT 20)
