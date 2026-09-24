#------------------------------------------------------------------------------
# AviatorRobot_simple - Simplified dual-arm aircraft control example
#------------------------------------------------------------------------------
# Reference implementation using:
# - PIN-IK for inverse kinematics
# - Pinocchio for forward kinematics and collision detection
# - MuJoCo for simulation (optional)
# - xCore SDK for Rokae robot hardware (optional)
#------------------------------------------------------------------------------

# Build options - inherit from parent or set defaults
option(AVIATOR_WITH_SIMULATION "Build MuJoCo backend and simulation tests" ${AVIATOR_BUILD_MUJOCO})
option(AVIATOR_WITH_ROKAE "Build xCore SDK backend" ${AVIATOR_BUILD_XCORE_SDK})
option(AVIATOR_WITH_VIEWER "Build GLFW viewer" ${AVIATOR_BUILD_VIEWER})

# Verify dependencies
if(NOT AVIATOR_BUILD_ROBOTICS_LIBS)
    message(FATAL_ERROR "AviatorRobot_simple requires AVIATOR_BUILD_ROBOTICS_LIBS=ON")
endif()

if(AVIATOR_WITH_SIMULATION AND NOT AVIATOR_BUILD_MUJOCO)
    message(FATAL_ERROR "AVIATOR_WITH_SIMULATION requires AVIATOR_BUILD_MUJOCO=ON")
endif()

if(AVIATOR_WITH_ROKAE AND NOT AVIATOR_BUILD_XCORE_SDK)
    message(FATAL_ERROR "AVIATOR_WITH_ROKAE requires AVIATOR_BUILD_XCORE_SDK=ON")
endif()

if(AVIATOR_WITH_VIEWER AND NOT (AVIATOR_WITH_SIMULATION AND AVIATOR_BUILD_VIEWER))
    message(WARNING "AVIATOR_WITH_VIEWER requires simulation and GLFW, disabling viewer")
    set(AVIATOR_WITH_VIEWER OFF)
endif()

message(STATUS "Configuring AviatorRobot_simple:")
message(STATUS "  - Simulation (MuJoCo): ${AVIATOR_WITH_SIMULATION}")
message(STATUS "  - Rokae hardware: ${AVIATOR_WITH_ROKAE}")
message(STATUS "  - Viewer (GLFW): ${AVIATOR_WITH_VIEWER}")

#------------------------------------------------------------------------------
# Core control library
#------------------------------------------------------------------------------
add_library(aviator_control SHARED
    src/Aviator.cpp
    src/backend_factory.cpp
    src/Kinematics_pin_ik.cpp
    src/CollisionChecker_pinocchio.cpp
)

target_include_directories(aviator_control
    PUBLIC include
    PRIVATE src
)

target_link_libraries(aviator_control
    PUBLIC aviator_robotics_deps
    PRIVATE Threads::Threads
)

# Add simulation backend
if(AVIATOR_WITH_SIMULATION)
    target_sources(aviator_control PRIVATE src/DataLink_direct.cpp)
    target_link_libraries(aviator_control PRIVATE mujoco_imported)
    target_compile_definitions(aviator_control PRIVATE AVIATOR_HAVE_MUJOCO=1)
endif()

# Add Rokae hardware backend
if(AVIATOR_WITH_ROKAE)
    # Keep SDK's embedded KDL private in a separate DSO
    add_library(aviator_rokae_sdk SHARED src/rokae_sdk.cpp)

    target_include_directories(aviator_rokae_sdk PRIVATE
        "${AVIATOR_VENDOR_DIR}/xCoreSDK-0.7.1/include"
    )

    target_link_libraries(aviator_rokae_sdk PRIVATE
        Eigen3::Eigen
        Rokae::Rokae
        Threads::Threads
        ${CMAKE_DL_LIBS}
    )

    target_link_options(aviator_rokae_sdk PRIVATE
        "-Wl,--exclude-libs,ALL"
        "-Wl,-Bsymbolic"
    )

    target_sources(aviator_control PRIVATE src/DataLink_rokae.cpp)
    target_link_libraries(aviator_control PRIVATE aviator_rokae_sdk)
    target_compile_definitions(aviator_control PUBLIC AVIATOR_HAVE_ROKAE=1)

    install(TARGETS aviator_rokae_sdk LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

#------------------------------------------------------------------------------
# Main executable
#------------------------------------------------------------------------------
add_executable(aviator src/main.cpp)

target_link_libraries(aviator PRIVATE aviator_control)

if(AVIATOR_WITH_SIMULATION)
    target_link_libraries(aviator PRIVATE mujoco_imported)
    target_compile_definitions(aviator PRIVATE AVIATOR_HAVE_MUJOCO=1)

    # Optional viewer
    if(AVIATOR_WITH_VIEWER)
        add_library(aviator_viewer STATIC src/Viewer.cpp)
        add_dependencies(aviator_viewer third_party_mujoco-3.4.0)

        target_include_directories(aviator_viewer PUBLIC
            "${AVIATOR_DEPS_DIR}/include"
        )

        target_link_libraries(aviator_viewer PUBLIC
            mujoco_imported
            glfw
        )

        target_link_libraries(aviator PRIVATE aviator_viewer)
        target_compile_definitions(aviator PRIVATE AVIATOR_HAVE_GLFW=1)
    endif()
endif()

#------------------------------------------------------------------------------
# Assets (config and models)
#------------------------------------------------------------------------------
add_custom_target(aviator_assets ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/config"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/model"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/model"
)

add_dependencies(aviator aviator_assets)

#------------------------------------------------------------------------------
# Installation
#------------------------------------------------------------------------------
install(TARGETS aviator aviator_control
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY config model
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#------------------------------------------------------------------------------
# Tests
#------------------------------------------------------------------------------
if(BUILD_TESTING)
    # Basic pose and kinematics tests (no backend required)
    add_executable(aviator_pose_test tests/test_pose.cpp)
    target_include_directories(aviator_pose_test PRIVATE include)
    target_link_libraries(aviator_pose_test PRIVATE aviator_system_deps)
    add_test(NAME aviator_pose COMMAND aviator_pose_test)

    # PIN-IK standalone test
    add_executable(aviator_pin_ik_test
        "${AVIATOR_VENDOR_DIR}/pin_ik-2.2.0/tests/standalone_tests.cpp"
    )
    target_link_libraries(aviator_pin_ik_test PRIVATE aviator_robotics_deps)
    add_test(NAME aviator_pin_ik COMMAND aviator_pin_ik_test
        "${AVIATOR_VENDOR_DIR}/pin_ik-2.2.0/tests/robot.urdf"
    )
    set_tests_properties(aviator_pin_ik PROPERTIES
        TIMEOUT 60
        ENVIRONMENT "LD_BIND_NOW=1"
    )

    # Simulation-based tests
    if(AVIATOR_WITH_SIMULATION)
        set(sim_test_names
            basic
            acceptance
            backend_selection
            realtime
            move_wheel
            servo_wheel
            kinematics
        )

        foreach(test_name IN LISTS sim_test_names)
            add_executable(aviator_${test_name}_test tests/test_${test_name}.cpp)
            target_link_libraries(aviator_${test_name}_test PRIVATE
                aviator_control
                mujoco_imported
            )
            target_compile_definitions(aviator_${test_name}_test PRIVATE
                AVIATOR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
                AVIATOR_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model"
            )
            add_test(NAME aviator_${test_name} COMMAND aviator_${test_name}_test)
            set_tests_properties(aviator_${test_name} PROPERTIES
                TIMEOUT 600
                ENVIRONMENT "LD_BIND_NOW=1"
            )
        endforeach()

        # Demo tests
        add_test(NAME aviator_demo
            COMMAND aviator --demo --headless
        )
        set_tests_properties(aviator_demo PROPERTIES
            TIMEOUT 600
            ENVIRONMENT "LD_BIND_NOW=1"
        )

        add_test(NAME aviator_servo_demo
            COMMAND aviator --servo-demo --headless
        )
        set_tests_properties(aviator_servo_demo PROPERTIES
            TIMEOUT 60
            ENVIRONMENT "LD_BIND_NOW=1"
        )

        # Python interactive test (optional)
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(NAME aviator_interactive
                COMMAND ${Python3_EXECUTABLE}
                    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_interactive.py"
                    "$<TARGET_FILE:aviator>"
            )
            set_tests_properties(aviator_interactive PROPERTIES TIMEOUT 30)
        endif()
    endif()

    # Controller safety test (no simulation required)
    add_executable(aviator_controller_safety_test tests/test_controller_safety.cpp)
    target_link_libraries(aviator_controller_safety_test PRIVATE aviator_control)
    target_compile_definitions(aviator_controller_safety_test PRIVATE
        AVIATOR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    )
    add_test(NAME aviator_controller_safety COMMAND aviator_controller_safety_test)

    # Open loop test
    add_executable(aviator_open_loop_test tests/test_open_loop.cpp)
    target_include_directories(aviator_open_loop_test PRIVATE include)
    add_test(NAME aviator_open_loop COMMAND aviator_open_loop_test)

    # Viewer test (manual, requires display)
    if(AVIATOR_WITH_VIEWER)
        add_executable(aviator_viewer_test tests/test_viewer.cpp)
        target_link_libraries(aviator_viewer_test PRIVATE aviator_viewer)
        target_compile_definitions(aviator_viewer_test PRIVATE
            AVIATOR_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/model"
        )
        # Not added to CTest - run manually on a display
    endif()
endif()
