# SPDX-License-Identifier: GPL-3.0-or-later
#
# ztest suite of the VDC application core (src/vdc/vdc_node.cpp): readiness,
# wheel command and drive status against a second Cyphal node on an
# in-memory CAN bus.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_test_vdc_node)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/src/vdc/vdc_node.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/vdc
  ${UNIMOC_ROOT}/tests/common)
target_link_libraries(app PRIVATE unimoc_lib)
