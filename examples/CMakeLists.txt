# AVIATOR Examples

# AviatorRobot_simple - Simplified dual-arm control example
option(AVIATOR_BUILD_EXAMPLE_SIMPLE "Build AviatorRobot_simple example" ON)

if(AVIATOR_BUILD_EXAMPLE_SIMPLE)
    add_subdirectory(AviatorRobot_simple)
endif()
