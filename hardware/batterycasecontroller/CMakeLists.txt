# make modm available to this hardware type
include(../../cmake/Modm.cmake)

add_subdirectory(modm)

add_executable(${CMAKE_PROJECT_NAME})

#   HARDWARE_MAX_PHASE_CURRENT_A
#       Absolute maximum phase current the power-stage inverter can carry [A].
#       Exceeding this will destroy the FETs.
#
#   HARDWARE_MAX_MOTOR_CURRENT_A
#       Absolute maximum resultant stator current vector magnitude [A].
#       Typically set to the motor's peak rating or the drive's continuous
#       rating, whichever is lower.  Must be <= HARDWARE_MAX_PHASE_CURRENT_A.
#
#   HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A
#       Absolute maximum battery discharge current [A].
#       Set by the battery / wiring / fuse rating for the specific build.
#
#   HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A
#       Absolute maximum battery charge (regenerative braking) current [A].
#       Set to 0 for systems that do not support regenerative braking.
#
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
  HARDWARE_MAX_PHASE_CURRENT_A=100.0
  HARDWARE_MAX_MOTOR_CURRENT_A=80.0
  HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A=50.0
  HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A=20.0
)

target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  adc.cpp
  sincos.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ../
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  modm_arch_options
  modm_options
  modm_warnings
  modm)

# Add the hardware directory to the linker search path so the linker can find
# modm/link/linkerscript.ld relative to this directory (not CMAKE_SOURCE_DIR).
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
  -L${CMAKE_CURRENT_LIST_DIR}
)

# Outputs hex and bin files.
modm_targets_create(${CMAKE_PROJECT_NAME})

