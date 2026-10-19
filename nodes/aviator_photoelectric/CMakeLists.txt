if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  return()
endif()

file(RELATIVE_PATH PHOTOELECTRIC_INSTALLED_CONFIG "${CMAKE_INSTALL_FULL_BINDIR}"
  "${CMAKE_INSTALL_FULL_DATADIR}/aviator/config/photoelectric.yaml")
add_executable(aviator_photoelectric main.cpp)
target_link_libraries(aviator_photoelectric PRIVATE aviator_transport aviator_runtime yaml-cpp)
target_compile_definitions(aviator_photoelectric PRIVATE
  PHOTOELECTRIC_SOURCE_CONFIG="${PROJECT_SOURCE_DIR}/config/photoelectric.yaml"
  PHOTOELECTRIC_INSTALLED_CONFIG="${PHOTOELECTRIC_INSTALLED_CONFIG}")
install(TARGETS aviator_photoelectric RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  COMPONENT aviator_photoelectric)
install(FILES ${PROJECT_SOURCE_DIR}/config/photoelectric.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/config COMPONENT aviator_photoelectric)
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/aviator_photoelectric
  COMPONENT aviator_photoelectric)

if(BUILD_TESTING)
  add_test(NAME photoelectric_help COMMAND aviator_photoelectric --help)
  add_test(NAME photoelectric_config COMMAND aviator_photoelectric --check-config)
  set_tests_properties(photoelectric_help photoelectric_config PROPERTIES TIMEOUT 10)
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import yaml, zmq"
      RESULT_VARIABLE PHOTOELECTRIC_PYTHON_DEPS OUTPUT_QUIET ERROR_QUIET)
    if(PHOTOELECTRIC_PYTHON_DEPS EQUAL 0)
      add_test(NAME photoelectric_process COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/process_test.py $<TARGET_FILE:aviator_photoelectric>
        ${PROJECT_SOURCE_DIR}/config/photoelectric.yaml)
      set_tests_properties(photoelectric_process PROPERTIES TIMEOUT 30)
    endif()
  endif()
endif()
