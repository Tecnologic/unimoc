# SPDX-License-Identifier: GPL-3.0-or-later
#
# Closed loop on native_sim: the control core (control cycle and observer)
# drives the motor plant of lib/sim through the simulated hardware layer.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_sim_closed_loop)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/src/current_control/current_control_isr.cpp
  ${UNIMOC_ROOT}/src/current_control/slow_update.cpp
  ${UNIMOC_ROOT}/src/current_control/outer_loop.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/system_interface.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/zephyr_settings_storage.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_sim/hardware_interface.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/hardware/zephyr_sim
  ${UNIMOC_ROOT}/hardware/zephyr_common)
target_link_libraries(app PRIVATE unimoc_lib unimoc_sim)
