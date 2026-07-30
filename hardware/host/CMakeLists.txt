cmake_minimum_required(VERSION 4.0.0)

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