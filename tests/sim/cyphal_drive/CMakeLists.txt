# SPDX-License-Identifier: GPL-3.0-or-later
#
# The Cyphal application of the drive on native_sim: DriveNode with the real
# current-control runtime, the simulated plant and the settings on ZMS,
# reached by a second libcanard node over an in-memory CAN bus.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_sim_cyphal_drive)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  src/slcan_test.cpp
  ${UNIMOC_ROOT}/src/current_control/current_control_isr.cpp
  ${UNIMOC_ROOT}/src/current_control/slow_update.cpp
  ${UNIMOC_ROOT}/src/current_control/outer_loop.cpp
  ${UNIMOC_ROOT}/src/drive/cart_control.cpp
  ${UNIMOC_ROOT}/src/drive/drive_node.cpp
  ${UNIMOC_ROOT}/src/drive/settings_register_provider.cpp
  ${UNIMOC_ROOT}/src/drive/startup_registers.cpp
  ${UNIMOC_ROOT}/src/drive/timing_registers.cpp
  ${UNIMOC_ROOT}/src/startup/hw_startup.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/system_interface.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/zephyr_settings_storage.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_sim/hardware_interface.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_sim/slcan_media.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/drive
  ${UNIMOC_ROOT}/src/startup
  ${UNIMOC_ROOT}/hardware/zephyr_sim
  ${UNIMOC_ROOT}/hardware/zephyr_common
  ${UNIMOC_ROOT}/tests/common)
target_link_libraries(app PRIVATE unimoc_lib unimoc_sim)
