# SocketCAN + ZMQ only: also available in AVIATOR_COMMUNICATION_ONLY builds.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(STATUS "Skipping aviator_hand: Linux SocketCAN is required")
  return()
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

file(RELATIVE_PATH HAND_INSTALLED_CONFIG "${CMAKE_INSTALL_FULL_BINDIR}"
  "${CMAKE_INSTALL_FULL_DATADIR}/aviator/config/inspire_hand.yaml")
add_executable(aviator_hand hand_node.cpp)
target_link_libraries(aviator_hand PRIVATE aviator_transport yaml-cpp Threads::Threads)
target_compile_definitions(aviator_hand PRIVATE
  AVIATOR_HAND_SOURCE_CONFIG="${PROJECT_SOURCE_DIR}/config/inspire_hand.yaml"
  AVIATOR_HAND_INSTALLED_CONFIG="${HAND_INSTALLED_CONFIG}")

install(TARGETS aviator_hand RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT aviator_hand)
install(FILES ${PROJECT_SOURCE_DIR}/config/inspire_hand.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/config COMPONENT aviator_hand)
install(PROGRAMS hand_command.py DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/aviator_hand COMPONENT aviator_hand)
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/aviator_hand COMPONENT aviator_hand)

if(BUILD_TESTING)
  aviator_add_test_executable(aviator_hand_test hand_node_test.cpp)
  target_link_libraries(aviator_hand_test PRIVATE aviator_transport yaml-cpp Threads::Threads)
  add_test(NAME aviator_hand_help COMMAND aviator_hand --help)
  add_test(NAME aviator_hand_config COMMAND aviator_hand --check-config)
  add_test(NAME aviator_hand_controller COMMAND aviator_hand_test)
  set_tests_properties(aviator_hand_help aviator_hand_config aviator_hand_controller PROPERTIES TIMEOUT 15)
  set_tests_properties(aviator_hand_config PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME aviator_hand_publisher COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/hand_command_test.py)
    set_tests_properties(aviator_hand_publisher PROPERTIES TIMEOUT 20
      ENVIRONMENT "HAND_CONTROLLER_TEST=$<TARGET_FILE:aviator_hand_test>")
  endif()
endif()
