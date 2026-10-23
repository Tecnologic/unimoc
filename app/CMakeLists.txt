# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

# --- Platform selection (Kconfig symbol chosen by the board) ----------------
if(CONFIG_UNIMOC_PLATFORM_BCC)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_bcc)
  set(UNIMOC_PLATFORM_EXTRA_SOURCES foc_stm32g4.cpp cyphal_platform.cpp)
elseif(CONFIG_UNIMOC_PLATFORM_SIM)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_sim)
  set(UNIMOC_PLATFORM_EXTRA_SOURCES cyphal_platform.cpp slcan_media.cpp sim_cmdline.c)
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
  ${UNIMOC_ROOT}/src/current_control/outer_loop.cpp
  ${UNIMOC_ROOT}/src/startup/hw_startup.cpp
  ${UNIMOC_ROOT}/src/drive/cart_control.cpp
  ${UNIMOC_ROOT}/src/drive/drive_node.cpp
  ${UNIMOC_ROOT}/src/drive/settings_register_provider.cpp
  ${UNIMOC_ROOT}/src/drive/startup_registers.cpp
  ${UNIMOC_ROOT}/src/drive/timing_registers.cpp
  ${UNIMOC_ROOT}/src/trace/trace_node.cpp
  ${UNIMOC_ROOT}/src/trace/trace_signals.cpp)
target_include_directories(unimoc_core PUBLIC
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/drive
  ${UNIMOC_ROOT}/src/trace
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
  ${UNIMOC_ROOT}/hardware/zephyr_common/control_thread.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/cyphal_runtime.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/zephyr_can_media.cpp
  ${UNIMOC_PLATFORM_DIR}/hardware_interface.cpp)
if(CONFIG_UNIMOC_CYPHAL_LOG_BACKEND)
  target_sources(app PRIVATE ${UNIMOC_ROOT}/hardware/zephyr_common/cyphal_log_backend.c)
endif()
foreach(unimoc_platform_source IN LISTS UNIMOC_PLATFORM_EXTRA_SOURCES)
  target_sources(app PRIVATE ${UNIMOC_PLATFORM_DIR}/${unimoc_platform_source})
endforeach()
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/src/current_control
  ${UNIMOC_ROOT}/src/trace
  ${UNIMOC_ROOT}/hardware/zephyr_common
  ${UNIMOC_PLATFORM_DIR})
target_link_libraries(app PRIVATE unimoc_core unimoc_lib ${UNIMOC_PLATFORM_LIBRARIES})

# GetInfo software_vcs_revision_id: the first 16 hex digits of the git hash.
execute_process(COMMAND git -C ${UNIMOC_ROOT} rev-parse --short=16 HEAD
                OUTPUT_VARIABLE unimoc_vcs_revision
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
                RESULT_VARIABLE unimoc_vcs_result)
if(unimoc_vcs_result EQUAL 0 AND unimoc_vcs_revision MATCHES "^[0-9a-f]+$")
  set_source_files_properties(${UNIMOC_ROOT}/hardware/zephyr_common/cyphal_runtime.cpp
    PROPERTIES COMPILE_DEFINITIONS "UNIMOC_VCS_REVISION=0x${unimoc_vcs_revision}ULL")
endif()
