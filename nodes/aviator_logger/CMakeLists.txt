find_package(Threads REQUIRED)

add_executable(aviator_logger main.cpp logger.cpp)
target_compile_features(aviator_logger PRIVATE cxx_std_17)
target_include_directories(aviator_logger PRIVATE
  ${PROJECT_SOURCE_DIR}/third_party/mcap/include)
# Header-only MCAP: MCAP_IMPLEMENTATION is defined in exactly one TU (logger.cpp).
# Disable zstd/lz4 so no external compression libraries are required.
target_compile_definitions(aviator_logger PRIVATE
  MCAP_COMPRESSION_NO_ZSTD
  MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(aviator_logger PRIVATE
  aviator_transport aviator_runtime Threads::Threads)
install(TARGETS aviator_logger RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
