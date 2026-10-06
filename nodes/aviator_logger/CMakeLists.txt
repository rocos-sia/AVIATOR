find_package(Threads REQUIRED)
add_executable(aviator_logger main.cpp logger.cpp arm_target_recorder.cpp)
target_compile_features(aviator_logger PRIVATE cxx_std_17)
target_compile_definitions(aviator_logger PRIVATE AVIATOR_PROJECT_ROOT="${PROJECT_SOURCE_DIR}")
target_link_libraries(aviator_logger PRIVATE
  aviator_recording aviator_transport aviator_runtime Threads::Threads)
install(TARGETS aviator_logger RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES ${PROJECT_SOURCE_DIR}/config/recording.yaml DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/config)
