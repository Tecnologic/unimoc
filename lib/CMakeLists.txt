# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.20.0)

# Add the library target
add_library(unimoc_lib INTERFACE)

add_subdirectory(cyphal)
add_subdirectory(units)
add_subdirectory(settings)
add_subdirectory(motor_control)
add_subdirectory(sim)
# cymon is a submodule (third-party for this repository): not held to the
# UNIMOC clang-tidy rules, like the Zephyr and vendored headers.
set(UNIMOC_SAVED_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
set(CMAKE_CXX_CLANG_TIDY "")
add_subdirectory(cymon)
set(CMAKE_CXX_CLANG_TIDY "${UNIMOC_SAVED_CLANG_TIDY}")

target_link_libraries(unimoc_lib INTERFACE
    project_options
    project_warnings
    unimoc_cyphal
    unimoc_units
    unimoc_settings
    unimoc_motor_control
    cymon)
