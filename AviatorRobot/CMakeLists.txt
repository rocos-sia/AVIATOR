cmake_minimum_required(VERSION 3.16)
project(AviatorRobot LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

get_filename_component(ROCOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rocos_app" ABSOLUTE)
get_filename_component(SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../reference/rocos-mujoco" ABSOLUTE)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system chrono thread)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(orocos_kdl REQUIRED)

# Vendored Pinocchio + hpp-fcl (built by third_party/build_deps.sh into _install).
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/_install")
find_package(pinocchio REQUIRED)

set(NLOPT_PYTHON OFF CACHE BOOL "" FORCE)
set(NLOPT_OCTAVE OFF CACHE BOOL "" FORCE)
set(NLOPT_MATLAB OFF CACHE BOOL "" FORCE)
set(NLOPT_GUILE OFF CACHE BOOL "" FORCE)
set(NLOPT_SWIG OFF CACHE BOOL "" FORCE)
set(NLOPT_TESTS OFF CACHE BOOL "" FORCE)
foreach(dependency nlopt kdl_parser trac_ik ruckig plog gripper spdlog sml)
    add_subdirectory("${ROCOS_DIR}/3rdparty/${dependency}" "dependencies/${dependency}" EXCLUDE_FROM_ALL)
endforeach()

# Compile the same Robot/Hardware sources with the MuJoCo POSIX backend.
# The original RCM build still selects its original ECM implementation.
set(ROCOS_SOURCES logger.cc command.cc status_word.cc control_word.cc drive_state.cc
    hardware.cc hardware_interface.cc hardware_sim.cc ft_sensor_interface.cc ft_sensor_ecat.cc
    drive.cc drive_guard.cc interpolate.cc robotMath.cc kinematics.cc dynamics.cc robot.cc
    UnitIntervalMotionProfile.cpp JC_helper_kinematics.cpp JC_helper_dynamics.cpp DHParamsLoader.cpp)
list(TRANSFORM ROCOS_SOURCES PREPEND "${ROCOS_DIR}/src/")
add_library(aviator_rocos_core SHARED ${ROCOS_SOURCES})
target_include_directories(aviator_rocos_core PUBLIC "${ROCOS_DIR}/include" "${SIM_DIR}/include"
    PRIVATE "${ROCOS_DIR}")
target_compile_definitions(aviator_rocos_core PUBLIC ROCOS_MUJOCO_BACKEND=1)
target_link_libraries(aviator_rocos_core PUBLIC yaml-cpp Eigen3::Eigen orocos-kdl
    Boost::date_time Boost::filesystem Boost::system Boost::chrono Boost::thread
    Threads::Threads trac_ik kdl_parser ruckig::ruckig plog::plog JC_helper::gripper
    spdlog::spdlog_header_only sml::sml OpenSSL::SSL dl rt)

add_library(aviator SHARED
    src/Aviator.cpp
    src/DataLink_mujoco.cpp
    src/Kinematics_tracik.cpp
    src/CollisionChecker_pinocchio.cpp)
target_include_directories(aviator PUBLIC include)
target_link_libraries(aviator PUBLIC aviator_rocos_core)
target_include_directories(aviator PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/include")
target_link_libraries(aviator PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/lib/libmujoco.so.3.4.0")
# 碰撞后端:Pinocchio(parsers 已传递依赖 hpp-fcl/coal、urdfdom、Boost、console_bridge)。
target_link_libraries(aviator PRIVATE pinocchio::pinocchio_parsers pinocchio::pinocchio_collision)

# Rokae xCore SDK 真机后端(可选)。头文件与预编译库不入库(见 .gitignore),检测到才编译
# DataLink_rokae.cpp 并定义 AVIATOR_HAVE_ROKAE;缺失时自动跳过,不影响仿真构建。
set(ROKAE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../xCoreSDK-CPP-main/include"
    CACHE PATH "Rokae xCore SDK 头文件目录")
set(ROKAE_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../xCoreSDK-0.7.1-linux-x86_64/lib/Linux/x86_64"
    CACHE PATH "Rokae xCore SDK 预编译库目录")
set(ROKAE_CORE_LIB "")
# 优先共享库(自包含,运行时依赖最少);库文件带版本号后缀,按完整路径链接。
foreach(candidate
        "${ROKAE_LIB_DIR}/libxCoreSDK.so.0.7.1"
        "${ROKAE_LIB_DIR}/libxCoreSDK.so"
        "${ROKAE_LIB_DIR}/libxCoreSDK.a")
    if(EXISTS "${candidate}")
        set(ROKAE_CORE_LIB "${candidate}")
        break()
    endif()
endforeach()
if(ROKAE_CORE_LIB AND EXISTS "${ROKAE_INCLUDE_DIR}/rokae/robot.h")
    message(STATUS "Rokae 真机后端:启用 (${ROKAE_CORE_LIB})")
    target_sources(aviator PRIVATE src/DataLink_rokae.cpp)
    target_include_directories(aviator PRIVATE "${ROKAE_INCLUDE_DIR}")
    target_compile_definitions(aviator PRIVATE AVIATOR_HAVE_ROKAE=1)
    target_link_libraries(aviator PRIVATE "${ROKAE_CORE_LIB}")
    # libxCoreSDK.so 对 Model::*、ConvertFrameArray 及部分 KDL 求解器留有未定义符号,
    # 由 SDK 附带的静态库 libxMateModel.a 提供(必须排在共享库之后链接)。
    # 但 libxMateModel.a 内附一份与项目系统 orocos_kdl ABI 不一致的 KDL 副本,若按默认
    # 可见性导出会与系统 KDL 相互覆盖(符号插值)导致堆损坏(free(): invalid pointer)。
    # 故链接后用 version script(cmake/rokae_hide_kdl.map)将其 KDL 符号置为本地,
    # 仅保留 Model::* 与 Convert* 供 libxCoreSDK.so 引用;本项目算法路径只走关节级
    # ServoJ 接口,不调用 KDL 模型接口,故隐藏这些符号不影响功能。自定义 KDL 求解器
    # (如 ChainIkSolverAcc_pinv_givens)被隐藏后,下游可执行以 --allow-shlib-undefined
    # 允许其未定义引用延后到运行时(未调用即不解析)。
    if(EXISTS "${ROKAE_LIB_DIR}/libxMateModel.a")
        target_link_libraries(aviator PRIVATE "${ROKAE_LIB_DIR}/libxMateModel.a")
        target_link_options(aviator PRIVATE
            "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/rokae_hide_kdl.map")
    endif()
    # 库的 SONAME 为 libxCoreSDK.so.0(需 libxCoreSDK.so / .so.0 符号链接,见 SDK 布局)。
    # 下游可执行链接与运行时都需要能按该名字解析到库。
    link_directories("${ROKAE_LIB_DIR}")
    target_link_options(aviator PRIVATE "-Wl,-rpath,${ROKAE_LIB_DIR}")
else()
    message(STATUS "Rokae 真机后端:未启用(未找到 xCore SDK 头文件或库)")
endif()
add_executable(aviatorAppMain src/main.cpp)
target_link_libraries(aviatorAppMain PRIVATE aviator)
# libxCoreSDK.so 的 Model::*/KDL 未定义符号(见上方 Rokae 块说明)只在运行时按需解析,
# 可执行链接阶段需显式允许。
target_link_options(aviatorAppMain PRIVATE "-Wl,--allow-shlib-undefined")
target_compile_definitions(aviatorAppMain PRIVATE AVIATOR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

add_executable(aviator_grasp_search tools/search_grasp.cpp)
target_include_directories(aviator_grasp_search PRIVATE "${SIM_DIR}/third_party/mujoco-3.4.0/include")
target_link_libraries(aviator_grasp_search PRIVATE aviator_rocos_core
    "${SIM_DIR}/third_party/mujoco-3.4.0/lib/libmujoco.so.3.4.0")

add_custom_target(aviator_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/config" "${CMAKE_BINARY_DIR}/bin/config"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/bin/model"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SIM_DIR}/model/aviator.xml" "${CMAKE_BINARY_DIR}/bin/model/aviator.xml"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SIM_DIR}/model/aviator_collision.urdf" "${CMAKE_BINARY_DIR}/bin/model/aviator_collision.urdf"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SIM_DIR}/model/aviator_collision.srdf" "${CMAKE_BINARY_DIR}/bin/model/aviator_collision.srdf"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SIM_DIR}/model/aviator_meshes" "${CMAKE_BINARY_DIR}/bin/model/aviator_meshes")
add_dependencies(aviatorAppMain aviator_assets)
include(CTest)
if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_executable(aviator_integration_test tests/control_integration.cpp)
    target_link_libraries(aviator_integration_test PRIVATE aviator)
    target_link_options(aviator_integration_test PRIVATE "-Wl,--allow-shlib-undefined")
    set(AVIATOR_TEST_SIMULATOR "${SIM_DIR}/build/aviator-headless/bin/rocos_mujoco_sim"
        CACHE FILEPATH "MuJoCo simulator executable for the integration test")
    add_test(NAME aviator_dual_arm_integration
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_integration.py"
            "${AVIATOR_TEST_SIMULATOR}" "$<TARGET_FILE:aviator_integration_test>"
            "${CMAKE_CURRENT_SOURCE_DIR}/config/aviator.yaml")
    set_tests_properties(aviator_dual_arm_integration PROPERTIES TIMEOUT 180)
endif()
