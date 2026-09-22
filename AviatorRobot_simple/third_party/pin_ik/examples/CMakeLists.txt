add_executable(ik_tests src/ik_tests.cpp)
target_link_libraries(ik_tests PRIVATE pin_ik)
install(TARGETS ik_tests RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
set_target_properties(ik_tests PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

set_target_properties(ik_tests PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
