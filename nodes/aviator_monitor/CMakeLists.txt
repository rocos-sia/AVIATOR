find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/index.html" MONITOR_HTML)
configure_file(page.hpp.in generated/page.hpp @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS index.html)
add_library(aviator_monitor_state monitor.cpp overview.cpp config.cpp preview.cpp assets.cpp logs.cpp)
target_include_directories(aviator_monitor_state PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(aviator_monitor_state PUBLIC cxx_std_17)
target_link_libraries(aviator_monitor_state PUBLIC aviator_service aviator_transport aviator_runtime Threads::Threads yaml-cpp)
add_executable(aviator_monitor main.cpp)
target_include_directories(aviator_monitor PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_compile_definitions(aviator_monitor PRIVATE
  AVIATOR_MONITOR_INSTALL_ROOT="${CMAKE_INSTALL_DATADIR}/aviator/monitor"
  AVIATOR_MONITOR_WEB="${CMAKE_CURRENT_SOURCE_DIR}/web"
  AVIATOR_MONITOR_MODELS="${PROJECT_SOURCE_DIR}/models"
  AVIATOR_MONITOR_CONFIG="${PROJECT_SOURCE_DIR}/config/monitor.yaml")
target_link_libraries(aviator_monitor PRIVATE aviator_monitor_state)
# CONTROL publishes an 81-point arm.command window every 20 ms. Unoptimized
# JSON container operations cannot drain that traffic alongside device feedback:
# the SUB FIFO then makes current arm/hand samples appear seconds out of date.
# Optimize both translation-unit sets (JSON templates are instantiated in both),
# retaining debug symbols and the normal freshness thresholds.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target aviator_monitor_state aviator_monitor)
    target_compile_options(${target} PRIVATE $<$<CONFIG:Debug>:-O2>)
  endforeach()
endif()
install(TARGETS aviator_monitor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES web/app.js web/logs.js web/settings.js web/viewer.js web/vendor.js web/style.css web/THIRD_PARTY_NOTICES.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/monitor/web)
install(FILES ${PROJECT_SOURCE_DIR}/config/monitor.yaml DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/monitor)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/models/urdf ${PROJECT_SOURCE_DIR}/models/meshes
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/monitor/models)
