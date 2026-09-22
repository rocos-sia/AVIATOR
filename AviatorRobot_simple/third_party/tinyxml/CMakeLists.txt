# Local CMake adapter for the original TinyXML 2.6.2 source (which provides only Makefiles).
cmake_minimum_required(VERSION 3.16)
project(tinyxml VERSION 2.6.2 LANGUAGES CXX)
include(GNUInstallDirs)
add_library(tinyxml SHARED tinyxml.cpp tinyxmlparser.cpp tinyxmlerror.cpp tinystr.cpp)
target_compile_definitions(tinyxml PUBLIC TIXML_USE_STL)
set_target_properties(tinyxml PROPERTIES VERSION 2.6.2 SOVERSION 2.6.2)
install(TARGETS tinyxml LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES tinyxml.h tinystr.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
