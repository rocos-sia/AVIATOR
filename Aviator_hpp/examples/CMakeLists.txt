# Test demo executable
add_executable(test_demo test_demo.cpp)
target_link_libraries(test_demo aviator)

# Test init executable
add_executable(test_init test_init.cpp)
target_link_libraries(test_init aviator)

# Copy config files to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test.yaml
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
