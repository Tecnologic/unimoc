# SPDX-License-Identifier: GPL-3.0-or-later
#
# Portable vehicle logic of the vehicle dynamics controller (VDC, apps/vdc):
# the wheels of the cart, the drive monitor and the torque split
# (docs/system_plan.md, section 5.3). Header only, free of Zephyr.

add_library(unimoc_vehicle INTERFACE)
target_include_directories(unimoc_vehicle INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(unimoc_vehicle INTERFACE
    project_options
    project_warnings
    unimoc_units)

add_library(unimoc::vehicle ALIAS unimoc_vehicle)
