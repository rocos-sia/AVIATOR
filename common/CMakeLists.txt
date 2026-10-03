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

add_library(aviator_service service.cpp)
target_link_libraries(aviator_service PUBLIC aviator_transport aviator_runtime)

find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVIATOR_VIDEO REQUIRED IMPORTED_TARGET GLOBAL libavcodec libavutil libswscale libzstd)
add_library(aviator_camera_recording recording_config.cpp camera_recording.cpp)
target_compile_features(aviator_camera_recording PUBLIC cxx_std_17)
target_link_libraries(aviator_camera_recording PUBLIC aviator_protocol PRIVATE
  yaml-cpp protobuf::libprotobuf PkgConfig::AVIATOR_VIDEO)

# Single implementation unit for MCAP. Container compression stays disabled;
# camera payload compression belongs to aviator_camera_recording.
add_library(aviator_recording recording.cpp)
target_compile_features(aviator_recording PUBLIC cxx_std_17)
target_include_directories(aviator_recording SYSTEM PRIVATE
  ${PROJECT_SOURCE_DIR}/third_party/mcap-2.1.3/include)
target_compile_definitions(aviator_recording PRIVATE
  MCAP_COMPRESSION_NO_ZSTD MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(aviator_recording PUBLIC aviator_protocol aviator_camera_recording aviator_service)

install(TARGETS aviator_service aviator_protocol aviator_transport aviator_runtime aviator_recording aviator_camera_recording
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES service.hpp protocol.hpp transport.hpp runtime.hpp startup.hpp recording.hpp recording_config.hpp camera_recording.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aviator)

# Device protocol is independent of planning libraries and hardware SDKs.
add_library(aviator_motion_protocol motion.cpp)
target_include_directories(aviator_motion_protocol PUBLIC ${PROJECT_SOURCE_DIR}/common/include)
target_link_libraries(aviator_motion_protocol PUBLIC aviator_runtime aviator_transport yaml-cpp)
target_compile_definitions(aviator_motion_protocol PRIVATE AVIATOR_SOURCE_CONFIG="${PROJECT_SOURCE_DIR}/config/system.yaml")
find_package(urdfdom REQUIRED)
add_library(aviator_device_server device_server.cpp device_settings.cpp)
target_include_directories(aviator_device_server SYSTEM PUBLIC ${urdfdom_INCLUDE_DIRS})
target_link_libraries(aviator_device_server PUBLIC aviator_motion_protocol ${urdfdom_LIBRARIES})

install(FILES ${PROJECT_SOURCE_DIR}/schemas/recording/camera_packet.proto DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/schemas)

# The 81-point Servo wire format must fit the unchanged 50 ms watchdog even in
# Debug builds. Keep symbols, but optimize JSON validation/encoding and device IO.
# Planning and application code retain the selected build's normal debug flags.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target aviator_protocol aviator_motion_protocol aviator_device_server)
    target_compile_options(${target} PRIVATE $<$<CONFIG:Debug>:-O2>)
  endforeach()
endif()
