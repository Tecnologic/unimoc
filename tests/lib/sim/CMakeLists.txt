# SPDX-License-Identifier: GPL-3.0-or-later
#
# ztest suites of lib/sim, run by twister on native_sim.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_test_lib_sim)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

file(GLOB test_sources src/*.cpp)
target_sources(app PRIVATE ${test_sources})
target_include_directories(app PRIVATE ${UNIMOC_ROOT}/tests/common)
target_link_libraries(app PRIVATE unimoc_lib unimoc_sim)
