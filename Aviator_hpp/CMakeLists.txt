cmake_minimum_required(VERSION 3.16)
project(aviator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置 third_party 安装路径
set(THIRD_PARTY_INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/third_party/_install)
list(APPEND CMAKE_PREFIX_PATH ${THIRD_PARTY_INSTALL})

# 查找依赖
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pinocchio
find_package(pinocchio REQUIRED)

# KDL & URDF
pkg_check_modules(orocos_kdl REQUIRED orocos-kdl)
pkg_check_modules(urdfdom REQUIRED urdfdom)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)

# TRAC-IK
find_path(TRAC_IK_INCLUDE_DIR trac_ik/trac_ik.hpp
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/trac_ik/include)
find_library(TRAC_IK_LIBRARY trac_ik
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/trac_ik/lib)

if(NOT TRAC_IK_INCLUDE_DIR OR NOT TRAC_IK_LIBRARY)
    message(FATAL_ERROR "TRAC-IK not found in third_party/trac_ik")
endif()

# NLopt
find_path(NLOPT_INCLUDE_DIR nlopt.hpp
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/include)
find_library(NLOPT_LIBRARY nlopt
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/lib)

# MuJoCo
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/include)
find_library(MUJOCO_LIBRARY mujoco
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/lib)

# 主库源文件
set(AVIATOR_SOURCES
    src/Aviator.cpp
    src/planning/KinematicsTracIk.cpp
)

# 创建共享库
add_library(aviator SHARED ${AVIATOR_SOURCES})

target_include_directories(aviator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    ${TRAC_IK_INCLUDE_DIR}
    ${NLOPT_INCLUDE_DIR}
    ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(aviator PUBLIC
    ${pinocchio_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${TRAC_IK_LIBRARY}
    ${NLOPT_LIBRARY}
    ${MUJOCO_LIBRARY}
    yaml-cpp
)

# 编译选项
target_compile_options(aviator PRIVATE -Wall -Wextra)

# Examples
add_subdirectory(examples)
