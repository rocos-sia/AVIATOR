cmake_minimum_required(VERSION 3.16)
project(change_stiffness LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(FATAL_ERROR "change_stiffness currently supports Linux x86_64 only")
endif()

set(XCORE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/xCoreSDK-0.7.1"
    CACHE PATH "Path to xCoreSDK 0.7.1 (including prebuilt libraries)")
set(XCORE_LIBRARY "${XCORE_SDK_ROOT}/lib/Linux/${CMAKE_SYSTEM_PROCESSOR}/libxCoreSDK.a")
set(XMATE_LIBRARY "${XCORE_SDK_ROOT}/lib/Linux/${CMAKE_SYSTEM_PROCESSOR}/libxMateModel.a")
foreach(required_file IN ITEMS "${XCORE_LIBRARY}" "${XMATE_LIBRARY}" "${XCORE_SDK_ROOT}/include/rokae/robot.h"
                               "${XCORE_SDK_ROOT}/external/Eigen/Core")
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR "Missing ${required_file}; set XCORE_SDK_ROOT to a complete xCoreSDK 0.7.1 distribution")
    endif()
endforeach()

find_package(Threads REQUIRED)
function(link_stiffness_sdk target)
    target_compile_features(${target} PRIVATE cxx_std_17)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_definitions(${target} PRIVATE EIGEN_DONT_ALIGN_STATICALLY)
    target_include_directories(${target} SYSTEM PRIVATE
        "${XCORE_SDK_ROOT}/include" "${XCORE_SDK_ROOT}/external")
    target_link_libraries(${target} PRIVATE
        "${XCORE_LIBRARY}" "${XMATE_LIBRARY}" Threads::Threads ${CMAKE_DL_LIBS})
endfunction()

foreach(demo IN ITEMS change_stiffness change_stiffness_simple)
    add_executable(${demo} ${demo}.cpp)
    link_stiffness_sdk(${demo})
endforeach()

option(CHANGE_STIFFNESS_BUILD_UI "Build the Qt5 stiffness control window" ON)
if(CHANGE_STIFFNESS_BUILD_UI)
    find_package(Qt5 COMPONENTS Widgets QUIET)
    if(Qt5Widgets_FOUND)
        add_executable(change_stiffness_ui change_stiffness_ui.cpp)
        link_stiffness_sdk(change_stiffness_ui)
        target_link_libraries(change_stiffness_ui PRIVATE Qt5::Widgets)

        include(CTest)
        if(BUILD_TESTING)
            find_package(Qt5 COMPONENTS Test REQUIRED)
            add_executable(change_stiffness_ui_test tests/test_ui.cpp)
            link_stiffness_sdk(change_stiffness_ui_test)
            target_link_libraries(change_stiffness_ui_test PRIVATE Qt5::Widgets Qt5::Test)
            add_test(NAME stiffness_ui_offline COMMAND change_stiffness_ui_test)
            set_tests_properties(stiffness_ui_offline PROPERTIES
                ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 30)
        endif()
    else()
        message(WARNING "Qt5 Widgets not found: UI skipped. Install qtbase5-dev or set Qt5_DIR. CLI demos remain available.")
    endif()
endif()
