find_package(Threads REQUIRED)
add_executable(flight_gateway main.cpp gateway.cpp config.cpp)
target_compile_features(flight_gateway PRIVATE cxx_std_17)
target_link_libraries(flight_gateway PRIVATE aviator_service aviator_transport aviator_runtime Threads::Threads yaml-cpp)
install(TARGETS flight_gateway RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

target_compile_definitions(flight_gateway PRIVATE AVIATOR_FLIGHT_CONFIG="${PROJECT_SOURCE_DIR}/config/flight.yaml")
install(FILES ${PROJECT_SOURCE_DIR}/config/flight.yaml DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/config)
