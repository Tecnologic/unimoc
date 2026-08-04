add_executable(${CMAKE_PROJECT_NAME})

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
  HARDWARE_MAX_PHASE_CURRENT_A=100.0
  HARDWARE_MAX_MOTOR_CURRENT_A=80.0
  HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A=50.0
  HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A=20.0
)

target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  hardware_interface.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ../
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  unimoc_lib
  threads
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC pthread)
