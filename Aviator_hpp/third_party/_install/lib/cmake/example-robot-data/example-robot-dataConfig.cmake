

# See CMakeLists.txt in this directory for why this stub exists.
# The hpp-pinocchio library only needs the EXAMPLE_ROBOT_DATA_MODEL_DIR macro
# carried by this target; it never opens the directory the macro points at.
if(NOT TARGET example-robot-data::example-robot-data)
    add_library(example-robot-data::example-robot-data INTERFACE IMPORTED)
    set_target_properties(
        example-robot-data::example-robot-data
        PROPERTIES INTERFACE_COMPILE_DEFINITIONS
                   "EXAMPLE_ROBOT_DATA_MODEL_DIR=\"/home/sia/Documents/GitHub/AVIATOR/Aviator_hpp/third_party/_install/share/example-robot-data/robots\"")
endif()

set(EXAMPLE_ROBOT_DATA_MODEL_DIR "/home/sia/Documents/GitHub/AVIATOR/Aviator_hpp/third_party/_install/share/example-robot-data/robots")
