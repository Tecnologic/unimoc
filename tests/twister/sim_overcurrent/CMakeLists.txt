# SPDX-License-Identifier: GPL-3.0-or-later
#
# Overcurrent path of the control core on native_sim: the fast interrupt trips
# the power stage and freezes the loop, a startup run acknowledges the trip.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_sim_overcurrent)

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/UnimocZephyr.cmake)
unimoc_zephyr_libraries()

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/src/current_control/current_control_isr.cpp
  ${UNIMOC_ROOT}/src/current_control/slow_update.cpp
  ${UNIMOC_ROOT}/src/startup/hw_startup.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/system_interface.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_sim/hardware_interface.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/startup
  ${UNIMOC_ROOT}/hardware/zephyr_sim
  ${UNIMOC_ROOT}/hardware/zephyr_common)
target_link_libraries(app PRIVATE unimoc_lib)
