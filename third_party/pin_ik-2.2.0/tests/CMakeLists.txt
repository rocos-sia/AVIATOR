add_executable(standalone_tests standalone_tests.cpp)
target_link_libraries(standalone_tests PRIVATE pin_ik)
add_test(NAME cpp_standalone COMMAND standalone_tests ${CMAKE_CURRENT_SOURCE_DIR}/robot.urdf)
if(PIN_IK_BUILD_EXAMPLES)
  add_test(NAME example_random_ik COMMAND ik_tests ${CMAKE_CURRENT_SOURCE_DIR}/robot.urdf base tip 100)
endif()

# Diagnostic tools are built explicitly and are not automated regression tests.
add_subdirectory(manual)
