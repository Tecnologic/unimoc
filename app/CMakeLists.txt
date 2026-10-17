# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/UnimocZephyr.cmake)
unimoc_zephyr_libraries()

# --- Platform selection (Kconfig symbol chosen by the board) ----------------
if(CONFIG_UNIMOC_PLATFORM_BCC)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_bcc)
  set(UNIMOC_PLATFORM_EXTRA_SOURCES foc_stm32g4.cpp)
elseif(CONFIG_UNIMOC_PLATFORM_SIM)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_sim)
else()
  message(FATAL_ERROR "No UNIMOC platform for board '${BOARD}'.")
endif()

# --- Application core: strict warnings, no Zephyr headers ------------------
zephyr_library_named(unimoc_core)
zephyr_library_sources(
  ${UNIMOC_ROOT}/src/current_control/current_control_isr.cpp
  ${UNIMOC_ROOT}/src/current_control/slow_update.cpp
  ${UNIMOC_ROOT}/src/startup/hw_startup.cpp)
zephyr_library_include_directories(
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/startup
  ${UNIMOC_PLATFORM_DIR})
zephyr_library_link_libraries(unimoc_lib)
target_link_libraries(unimoc_core PRIVATE unimoc_strict_warnings)

# --- Application and Zephyr glue (relaxed warnings) ------------------------
target_sources(app PRIVATE
  ${UNIMOC_ROOT}/src/main.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/system_interface.cpp
  ${UNIMOC_PLATFORM_DIR}/hardware_interface.cpp)
foreach(unimoc_platform_source IN LISTS UNIMOC_PLATFORM_EXTRA_SOURCES)
  target_sources(app PRIVATE ${UNIMOC_PLATFORM_DIR}/${unimoc_platform_source})
endforeach()
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/hardware/zephyr_common
  ${UNIMOC_PLATFORM_DIR})
target_link_libraries(app PRIVATE unimoc_core unimoc_lib)
