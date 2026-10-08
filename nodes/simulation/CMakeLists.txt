option(AVIATOR_BUILD_SIMULATION_NODE "Build the MuJoCo ZMQ simulation node" ${AVIATOR_BUILD_MUJOCO})
if(NOT AVIATOR_BUILD_SIMULATION_NODE)
  return()
endif()
if(NOT TARGET mujoco_imported)
  include(${PROJECT_SOURCE_DIR}/cmake/MujocoDependency.cmake)
  aviator_add_mujoco()
endif()
set(simulation_mujoco mujoco_imported)
set(simulation_mujoco_include "${AVIATOR_DEPS_DIR}/include")
# The full build installs the dependency directory at the project level.
if(AVIATOR_COMMUNICATION_ONLY)
  install(DIRECTORY "${AVIATOR_DEPS_DIR}/lib/" DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILES_MATCHING PATTERN "libmujoco.so*")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(SIMULATION_EGL REQUIRED IMPORTED_TARGET egl)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
add_library(aviator_simulation_model simulation.cpp)
target_include_directories(aviator_simulation_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${simulation_mujoco_include})
target_link_libraries(aviator_simulation_model PUBLIC aviator_runtime ${simulation_mujoco})
add_library(aviator_simulation_device DataLink_direct.cpp)
target_include_directories(aviator_simulation_device PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(aviator_simulation_device PUBLIC aviator_device_server ${simulation_mujoco})
target_compile_definitions(aviator_simulation_device PUBLIC AVIATOR_HAVE_MUJOCO=1)
# Hand-state JSON is prepared while holding the physics snapshot lock.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(aviator_simulation_model PRIVATE $<$<CONFIG:Debug>:-O2>)
endif()
add_library(aviator_simulation_camera camera.cpp camera_output.cpp camera_window.cpp)
target_link_libraries(aviator_simulation_camera PUBLIC aviator_simulation_model aviator_camera_recording
  aviator_transport glfw PkgConfig::SIMULATION_EGL OpenGL::GL PRIVATE PkgConfig::AVIATOR_VIDEO)
add_executable(simulation main.cpp Viewer.cpp)
target_link_libraries(simulation PRIVATE aviator_simulation_camera aviator_simulation_device)
# JSON templates from these units also participate in device protocol decoding.
# Keep the camera-enabled Debug executable within the existing 50 ms watchdog.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target simulation aviator_simulation_camera)
    target_compile_options(${target} PRIVATE $<$<CONFIG:Debug>:-O2>)
  endforeach()
endif()
install(TARGETS simulation RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/config)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/models/urdf DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/models)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/models/mjcf ${PROJECT_SOURCE_DIR}/models/meshes ${PROJECT_SOURCE_DIR}/models/textures DESTINATION ${CMAKE_INSTALL_DATADIR}/aviator/models)
if(BUILD_TESTING)
  aviator_add_test_executable(aviator_simulation_test test_simulation.cpp)
  target_link_libraries(aviator_simulation_test PRIVATE aviator_simulation_model aviator_motion_protocol)
  add_test(NAME simulation_model COMMAND aviator_simulation_test ${PROJECT_SOURCE_DIR}/models/mjcf/aviator.xml)
  add_test(NAME simulation_help COMMAND simulation --help)
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME simulation_headless COMMAND ${Python3_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/tests/simulation_smoke_test.py $<TARGET_FILE:simulation> ${PROJECT_SOURCE_DIR})
    set_tests_properties(simulation_headless PROPERTIES TIMEOUT 60)
    # Optional Python physics regression; the C++ model/startup tests remain mandatory.
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import mujoco, numpy, yaml"
      RESULT_VARIABLE SIMULATION_CONTACT_PYTHON_DEPS OUTPUT_QUIET ERROR_QUIET)
    if(SIMULATION_CONTACT_PYTHON_DEPS EQUAL 0)
      add_test(NAME simulation_urdf_sync COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/mjcf_urdf_test.py)
      add_test(NAME simulation_grasp_contact COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/mjcf_grasp_contact_test.py)
      add_test(NAME simulation_grasp_contact_offset COMMAND ${Python3_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/tests/mjcf_grasp_contact_test.py --grip-inset 0.0025)
      set_tests_properties(simulation_grasp_contact PROPERTIES TIMEOUT 60)
    endif()
  endif()
  add_test(NAME simulation_bus COMMAND aviator_simulation_test $<TARGET_FILE:simulation>
    ${PROJECT_SOURCE_DIR}/models/mjcf/aviator.xml no-camera)
  set_tests_properties(simulation_model simulation_help simulation_bus PROPERTIES TIMEOUT 60)
  option(AVIATOR_SIMULATION_TEST_EGL "Run integration test requiring an EGL OpenGL driver" OFF)
  if(AVIATOR_SIMULATION_TEST_EGL)
    add_test(NAME simulation_camera COMMAND aviator_simulation_test $<TARGET_FILE:simulation>
      ${PROJECT_SOURCE_DIR}/models/mjcf/aviator.xml camera)
    set_tests_properties(simulation_camera PROPERTIES TIMEOUT 60)
    if(Python3_Interpreter_FOUND)
      execute_process(COMMAND ${Python3_EXECUTABLE} -c
        "import mujoco, numpy, cv2; assert hasattr(cv2.aruco, 'ArucoDetector')"
        RESULT_VARIABLE SIMULATION_TAG_PYTHON_DEPS OUTPUT_QUIET ERROR_QUIET)
      if(SIMULATION_TAG_PYTHON_DEPS EQUAL 0)
        add_test(NAME simulation_apriltag COMMAND ${Python3_EXECUTABLE}
          ${PROJECT_SOURCE_DIR}/tests/mjcf_apriltag_test.py)
        set_tests_properties(simulation_apriltag PROPERTIES TIMEOUT 60 ENVIRONMENT "MUJOCO_GL=egl")
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import zmq, yaml; from google.protobuf import message_factory; assert hasattr(message_factory, 'GetMessageClass')"
          RESULT_VARIABLE SIMULATION_STREAM_PYTHON_DEPS OUTPUT_QUIET ERROR_QUIET)
        if(SIMULATION_STREAM_PYTHON_DEPS EQUAL 0)
          add_test(NAME simulation_camera_stream COMMAND ${Python3_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/tests/simulation_camera_stream_test.py
            $<TARGET_FILE:simulation> ${PROJECT_SOURCE_DIR})
          set_tests_properties(simulation_camera_stream PROPERTIES TIMEOUT 60)
        endif()
      endif()
    endif()
  endif()
endif()
