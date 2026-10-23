# SPDX-License-Identifier: GPL-3.0-or-later
#
# ztest suite of the e-bike control (src/ebike/ebike_control.cpp): pedal sensor, display link, assist and
# engagement on demand against a fake pedal sensor and display line, without bus or plant.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_test_ebike_control)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/src/ebike/ebike_control.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/ebike
  ${UNIMOC_ROOT}/hardware
  ${UNIMOC_ROOT}/tests/common)
target_link_libraries(app PRIVATE unimoc_lib)
