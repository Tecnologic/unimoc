# SPDX-License-Identifier: GPL-3.0-or-later
#
# Runs one GoogleTest source of lib/ under Zephyr (native_sim). Twister
# scenarios in testcase.yaml pass the source through
#   -DUNIMOC_TEST_SOURCE=<path below lib/>.
# The lib/ code is compiled unchanged; only the harness differs from the
# hosted CMake build.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_lib_test)

if(NOT UNIMOC_TEST_SOURCE)
  message(FATAL_ERROR "Set -DUNIMOC_TEST_SOURCE=<path below lib/>")
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/UnimocZephyr.cmake)
unimoc_zephyr_libraries()

# GoogleTest comes from the west workspace (see west.yml).
set(GOOGLETEST_DIR ${ZEPHYR_BASE}/../modules/lib/googletest)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
add_subdirectory(${GOOGLETEST_DIR} ${CMAKE_CURRENT_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)

target_sources(app PRIVATE
  src/main.cpp
  ${UNIMOC_ROOT}/lib/${UNIMOC_TEST_SOURCE})
get_filename_component(unimoc_test_dir ${UNIMOC_ROOT}/lib/${UNIMOC_TEST_SOURCE} DIRECTORY)
target_include_directories(app PRIVATE ${unimoc_test_dir})
target_link_libraries(app PRIVATE unimoc_lib gtest)
