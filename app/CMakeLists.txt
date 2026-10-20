# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

# --- Platform selection (Kconfig symbol chosen by the board) ----------------
if(CONFIG_UNIMOC_PLATFORM_BCC)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_bcc)
  set(UNIMOC_PLATFORM_EXTRA_SOURCES foc_stm32g4.cpp)
elseif(CONFIG_UNIMOC_PLATFORM_SIM)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_sim)
  set(UNIMOC_PLATFORM_LIBRARIES unimoc_sim)
else()
  message(FATAL_ERROR "No UNIMOC platform for board '${BOARD}'.")
endif()

# --- Application core: strict warnings, no Zephyr headers ------------------
# A plain library of the application: zephyr_library_*() is for Zephyr modules
# only. zephyr_interface keeps the target flags (ISA, FPU, defines) of the build.
add_library(unimoc_core STATIC)
target_sources(unimoc_core PRIVATE
  ${UNIMOC_ROOT}/src/current_control/current_control_isr.cpp
  ${UNIMOC_ROOT}/src/current_control/slow_update.cpp
  ${UNIMOC_ROOT}/src/startup/hw_startup.cpp)
target_include_directories(unimoc_core PRIVATE
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/startup
  ${UNIMOC_PLATFORM_DIR})
target_link_libraries(unimoc_core PRIVATE
  zephyr_interface
  unimoc_lib
  unimoc_strict_warnings)

# --- Application and Zephyr glue (relaxed warnings) ------------------------
target_sources(app PRIVATE
  ${UNIMOC_ROOT}/src/main.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/system_interface.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/zephyr_settings_storage.cpp
  ${UNIMOC_PLATFORM_DIR}/hardware_interface.cpp)
foreach(unimoc_platform_source IN LISTS UNIMOC_PLATFORM_EXTRA_SOURCES)
  target_sources(app PRIVATE ${UNIMOC_PLATFORM_DIR}/${unimoc_platform_source})
endforeach()
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/hardware/zephyr_common
  ${UNIMOC_PLATFORM_DIR})
target_link_libraries(app PRIVATE unimoc_core unimoc_lib ${UNIMOC_PLATFORM_LIBRARIES})
