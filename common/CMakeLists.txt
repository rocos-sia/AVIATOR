find_package(Threads REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED CONFIG)
find_path(AVIATOR_ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(AVIATOR_ZMQ_LIBRARY NAMES zmq REQUIRED)

add_library(aviator_zmq UNKNOWN IMPORTED)
set_target_properties(aviator_zmq PROPERTIES
  IMPORTED_LOCATION "${AVIATOR_ZMQ_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${AVIATOR_ZMQ_INCLUDE_DIR}")

add_library(aviator_protocol protocol.cpp)
target_compile_features(aviator_protocol PUBLIC cxx_std_17)
target_include_directories(aviator_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/aviator>)
target_link_libraries(aviator_protocol PUBLIC nlohmann_json::nlohmann_json)

add_library(aviator_transport transport.cpp)
target_compile_features(aviator_transport PUBLIC cxx_std_17)
target_include_directories(aviator_transport SYSTEM PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/cppzmq-4.11.0>)
target_link_libraries(aviator_transport PUBLIC aviator_protocol aviator_zmq)

add_library(aviator_runtime runtime.cpp)
target_compile_features(aviator_runtime PUBLIC cxx_std_17)
target_link_libraries(aviator_runtime PUBLIC aviator_protocol Threads::Threads)

install(TARGETS aviator_protocol aviator_transport aviator_runtime
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES protocol.hpp transport.hpp runtime.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aviator)
