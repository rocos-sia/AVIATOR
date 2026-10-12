# Python RealSense node; run nodes/camera/main.py with its camera environment.
# No C++ target or CMake-only dependency is required.
install(FILES main.py detectors.py preview_client.py recording_client.py
  steering_wheel.py visualization.py
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/nodes/camera)
install(FILES
  ${PROJECT_SOURCE_DIR}/tools/steering_wheel_calibration/calibration.py
  ${PROJECT_SOURCE_DIR}/tools/steering_wheel_calibration/geometry.py
  DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/tools/steering_wheel_calibration)
