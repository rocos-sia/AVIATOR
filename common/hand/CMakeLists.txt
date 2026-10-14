# Reusable native hand implementation for calibration and existing offline tests.
# The standalone SDK node shares the rh56ftp_hand policy directly.
add_library(aviator_hand_runtime STATIC HandDevice.cpp HandRuntime.cpp)
target_include_directories(aviator_hand_runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(aviator_hand_runtime PUBLIC aviator_transport aviator_runtime yaml-cpp)
