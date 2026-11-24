# SPDX-License-Identifier: GPL-3.0-or-later
# The gear buttons and lamps of the VDC on gpio_emul (hardware/zephyr_common/vdc_gear_io.cpp).
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_test_vdc_gear_io)

set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/vdc_gear_io.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/hardware/zephyr_common
  ${UNIMOC_ROOT}/tests/common)
target_link_libraries(app PRIVATE unimoc_lib)
