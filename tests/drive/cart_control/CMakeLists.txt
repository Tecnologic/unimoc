# SPDX-License-Identifier: GPL-3.0-or-later
#
# ztest suite of the cart control of the drive (src/drive/cart_control.cpp):
# torque source, limp-home, ramps and battery limits, without bus or plant.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_test_drive_cart_control)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/src/drive/cart_control.cpp)
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/drive
  ${UNIMOC_ROOT}/tests/common)
target_link_libraries(app PRIVATE unimoc_lib)
