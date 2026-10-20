# SPDX-License-Identifier: GPL-3.0-or-later

add_library(unimoc_cyphal INTERFACE)

target_include_directories(unimoc_cyphal INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(unimoc_cyphal INTERFACE project_options project_warnings)

add_library(unimoc::cyphal ALIAS unimoc_cyphal)
