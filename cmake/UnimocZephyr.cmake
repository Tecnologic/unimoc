# SPDX-License-Identifier: GPL-3.0-or-later
#
# CMake helpers of the UNIMOC Zephyr module. zephyr/CMakeLists.txt includes
# this file and calls unimoc_zephyr_libraries() when CONFIG_UNIMOC=y; the
# application and the tests only link the resulting targets. The hardware
# platform is selected by the build (Kconfig symbol -> sources), never by the
# preprocessor.

get_filename_component(UNIMOC_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# Declares the portable libraries of lib/ as INTERFACE targets.
#
# project_options / project_warnings are the targets the lib/ CMake files link
# against. In the Zephyr build they carry no warning flags, because Zephyr
# headers are not warning-clean under the strict UNIMOC set. The strict set is
# applied per target through unimoc_strict_warnings instead.
macro(unimoc_zephyr_libraries)
  add_library(project_options INTERFACE)
  target_include_directories(project_options INTERFACE ${UNIMOC_ROOT}/hardware)
  # The zero-latency hot path must not call libm (docs/hotpath_plan.md, M3):
  # without errno, std::sqrt and friends compile to FPU instructions or
  # builtins. Applies to everything that links the UNIMOC libraries.
  target_compile_options(project_options INTERFACE -fno-math-errno)
  add_library(project_warnings INTERFACE)

  include(${UNIMOC_ROOT}/cmake/CompilerWarnings.cmake)
  add_library(unimoc_strict_warnings INTERFACE)
  set_project_warnings(unimoc_strict_warnings)

  foreach(unimoc_lib_name units cyphal settings motor_control ebike)
    add_subdirectory(${UNIMOC_ROOT}/lib/${unimoc_lib_name}
                     ${CMAKE_CURRENT_BINARY_DIR}/lib/${unimoc_lib_name})
  endforeach()

  # cymon-lib (submodule): variable registry and captures of the traces. As a
  # subproject it builds only the target cymon, with the flags of this build;
  # the limits come from Kconfig (CONFIG_UNIMOC_TRACE_*).
  set(CYMON_MAX_BUFFER_BYTES ${CONFIG_UNIMOC_TRACE_BUFFER_BYTES} CACHE STRING "" FORCE)
  set(CYMON_MAX_VARS ${CONFIG_UNIMOC_TRACE_VARS} CACHE STRING "" FORCE)
  set(CYMON_MAX_CHANNELS ${CONFIG_UNIMOC_TRACE_CHANNELS} CACHE STRING "" FORCE)
  set(CYMON_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  if(NOT EXISTS ${UNIMOC_ROOT}/lib/cymon/CMakeLists.txt)
    message(FATAL_ERROR "lib/cymon is empty: run 'git submodule update --init lib/cymon'.")
  endif()
  add_subdirectory(${UNIMOC_ROOT}/lib/cymon ${CMAKE_CURRENT_BINARY_DIR}/lib/cymon)
  target_link_libraries(cymon PRIVATE zephyr_interface project_options unimoc_strict_warnings)

  # Motor plant simulation (header only). Not part of unimoc_lib: only the
  # native_sim platform and its tests link it.
  add_subdirectory(${UNIMOC_ROOT}/lib/sim ${CMAKE_CURRENT_BINARY_DIR}/lib/sim)

  add_library(unimoc_lib INTERFACE)
  target_link_libraries(unimoc_lib INTERFACE
    project_options
    project_warnings
    unimoc_cyphal
    unimoc_units
    unimoc_settings
    unimoc_motor_control
    unimoc_ebike
    cymon)
endmacro()
