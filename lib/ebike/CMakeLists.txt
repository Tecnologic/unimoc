# SPDX-License-Identifier: GPL-3.0-or-later

add_library(unimoc_ebike INTERFACE)
target_include_directories(unimoc_ebike INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(unimoc_ebike INTERFACE
    project_options
    project_warnings
    unimoc_units
    unimoc_motor_control_system)

add_library(unimoc::ebike ALIAS unimoc_ebike)
