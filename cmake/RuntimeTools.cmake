# Complete runtime tool resources. Native helper targets own their binary installs.
install(DIRECTORY "${PROJECT_SOURCE_DIR}/scripts/"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/scripts"
  USE_SOURCE_PERMISSIONS FILES_MATCHING PATTERN "*.sh" PATTERN "*.py")
foreach(tool grasp_tool_calibration steering_wheel_calibration wheel_bandwidth wheel_accuracy)
  install(DIRECTORY "${PROJECT_SOURCE_DIR}/tools/${tool}/"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/tools/${tool}"
    FILES_MATCHING PATTERN "*.py" PATTERN "*.html" PATTERN "*.css" PATTERN "*.js"
    PATTERN "test*" EXCLUDE PATTERN "native" EXCLUDE PATTERN "__pycache__" EXCLUDE)
endforeach()
install(DIRECTORY "${PROJECT_SOURCE_DIR}/models/tts/"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/models/tts"
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/deploy/"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/aviator/deploy"
  FILES_MATCHING PATTERN "*.py" PATTERN "*.txt" PATTERN "*.rules")
