# Specify the minimum CMake version
cmake_minimum_required(VERSION 4.00)

# Add the library target
add_library(unimoc_lib)

# Include the headers from the inc subfolder
target_include_directories(unimoc_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/system
    ${CMAKE_CURRENT_SOURCE_DIR}/units
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${CMAKE_CURRENT_SOURCE_DIR}/observer)

# Add the source files from the src subfolder
target_sources(unimoc_lib
    PRIVATE
    system/rotor_angle.cpp
    system/rotor_system.cpp
    system/stator_system.cpp
    system/three_phase_system.cpp)

target_link_libraries(unimoc_lib PUBLIC project_options project_warnings)
