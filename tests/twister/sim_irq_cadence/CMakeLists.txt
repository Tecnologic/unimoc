# SPDX-License-Identifier: GPL-3.0-or-later
#
# Checks the cycle contract of the native_sim hardware layer: one control and
# one observer callback per kHalfPeriodsPerCycle half periods, control first,
# and the half periods each duty-plan entry is applied in.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_sim_irq_cadence)

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/UnimocZephyr.cmake)
unimoc_zephyr_libraries()

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_sim/hardware_interface.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/hardware/zephyr_sim
  ${UNIMOC_ROOT}/hardware/zephyr_common)
target_link_libraries(app PRIVATE unimoc_lib unimoc_sim)
