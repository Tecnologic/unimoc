# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.20.0)

# Add the library target
add_library(unimoc_lib INTERFACE)

add_subdirectory(cyphal)
add_subdirectory(units)
add_subdirectory(settings)
add_subdirectory(motor_control)
add_subdirectory(sim)
add_subdirectory(cymon)

target_link_libraries(unimoc_lib INTERFACE
    project_options
    project_warnings
    unimoc_cyphal
    unimoc_units
    unimoc_settings
    unimoc_motor_control
    cymon)
