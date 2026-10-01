cmake_minimum_required(VERSION 3.10)  

# 设置项目名称和版本  
project(HandAPI VERSION 1.0 LANGUAGES C CXX)  

# 查找 pthread 库  
find_package(Threads REQUIRED)  

# 添加 C 的可执行文件 hand_api  
 add_library(hand_api STATIC hand_api.c)  

# 添加 C++ 的可执行文件 test  
add_executable(test test.cpp hand_api.c)  

# 链接 pthread 库和 hand_api 库到 test  
 target_link_libraries(test PRIVATE hand_api Threads::Threads)  

# # 设置 C 和 C++ 标准  
# set_target_properties(hand_api PROPERTIES  
#     C_STANDARD 11  
#     C_STANDARD_REQUIRED True  
# )  

# set_target_properties(test PROPERTIES  
#     CXX_STANDARD 11  
#     CXX_STANDARD_REQUIRED True  
# )  

# 提示用户/开发者修改串口和手 ID  
message(STATUS "请在 hand_api.c 中修改 ttyUSB0 为实际串口USB名，以及对于手的ID。")