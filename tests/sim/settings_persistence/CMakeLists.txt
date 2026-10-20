# SPDX-License-Identifier: GPL-3.0-or-later
#
# Settings persistence on native_sim: SettingsStore over the Zephyr settings
# subsystem and ZMS on the flash simulator, with the power-stage save gate.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_sim_settings_persistence)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/zephyr_settings_storage.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_sim/hardware_interface.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src
  ${UNIMOC_ROOT}/hardware/zephyr_sim
  ${UNIMOC_ROOT}/hardware/zephyr_common)
target_link_libraries(app PRIVATE unimoc_lib unimoc_sim)
