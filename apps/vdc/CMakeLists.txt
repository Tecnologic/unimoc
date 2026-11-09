# SPDX-License-Identifier: GPL-3.0-or-later
#
# The vehicle dynamics controller (VDC) of the cart (docs/system_plan.md,
# section 5.3): a Cyphal node in a Zephyr thread, no hot path. The skeleton
# arms the four wheel drives, splits a torque request onto them and collects
# their drive status; on native_sim and the NUCLEO-H743ZI.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(unimoc_vdc)

# The UNIMOC module (CONFIG_UNIMOC) declares the library targets.
set(UNIMOC_ROOT ${ZEPHYR_UNIMOC_MODULE_DIR})

# --- Platform selection (Kconfig symbol chosen by the board) ----------------
if(CONFIG_UNIMOC_PLATFORM_SIM)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_sim)
  set(UNIMOC_PLATFORM_EXTRA_SOURCES slcan_media.cpp slcan_ports.cpp sim_cmdline.c)
elseif(CONFIG_UNIMOC_PLATFORM_STM32H7)
  set(UNIMOC_PLATFORM_DIR ${UNIMOC_ROOT}/hardware/zephyr_stm32h7)
  set(UNIMOC_PLATFORM_EXTRA_SOURCES can_transceiver.cpp)
else()
  message(FATAL_ERROR "No UNIMOC platform for board '${BOARD}'.")
endif()

# --- Application core: strict warnings, no Zephyr headers ------------------
add_library(unimoc_vdc_core STATIC)
target_sources(unimoc_vdc_core PRIVATE ${UNIMOC_ROOT}/src/vdc/vdc_node.cpp)
target_include_directories(unimoc_vdc_core PUBLIC ${UNIMOC_ROOT}/src/vdc)
target_link_libraries(unimoc_vdc_core PRIVATE
  zephyr_interface
  unimoc_lib
  unimoc_strict_warnings)

# --- Application and Zephyr glue (relaxed warnings) ------------------------
target_sources(app PRIVATE
  ${UNIMOC_ROOT}/src/vdc/main.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/vdc_runtime.cpp
  ${UNIMOC_ROOT}/hardware/zephyr_common/zephyr_can_media.cpp
  ${UNIMOC_PLATFORM_DIR}/vdc_platform.cpp)
if(CONFIG_UNIMOC_CYPHAL_LOG_BACKEND)
  target_sources(app PRIVATE ${UNIMOC_ROOT}/hardware/zephyr_common/cyphal_log_backend.c)
endif()
foreach(unimoc_platform_source IN LISTS UNIMOC_PLATFORM_EXTRA_SOURCES)
  target_sources(app PRIVATE ${UNIMOC_PLATFORM_DIR}/${unimoc_platform_source})
endforeach()
target_include_directories(app PRIVATE
  ${UNIMOC_ROOT}/hardware/zephyr_common
  ${UNIMOC_PLATFORM_DIR})
target_link_libraries(app PRIVATE unimoc_vdc_core unimoc_lib)

# GetInfo software_vcs_revision_id: the first 16 hex digits of the git hash.
execute_process(COMMAND git -C ${UNIMOC_ROOT} rev-parse --short=16 HEAD
                OUTPUT_VARIABLE unimoc_vcs_revision
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
                RESULT_VARIABLE unimoc_vcs_result)
if(unimoc_vcs_result EQUAL 0 AND unimoc_vcs_revision MATCHES "^[0-9a-f]+$")
  set_source_files_properties(${UNIMOC_ROOT}/hardware/zephyr_common/vdc_runtime.cpp
    PROPERTIES COMPILE_DEFINITIONS "UNIMOC_VCS_REVISION=0x${unimoc_vcs_revision}ULL")
endif()
