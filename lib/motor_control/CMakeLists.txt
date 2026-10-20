set(UNIMOC_MOTOR_CONTROL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(unimoc_motor_control_system INTERFACE)
target_include_directories(unimoc_motor_control_system INTERFACE
    ${UNIMOC_MOTOR_CONTROL_INCLUDE_DIR})
target_link_libraries(unimoc_motor_control_system INTERFACE
    project_options
    project_warnings
    unimoc_units)

add_library(unimoc_motor_control_observer INTERFACE)
target_include_directories(unimoc_motor_control_observer INTERFACE
    ${UNIMOC_MOTOR_CONTROL_INCLUDE_DIR})
target_link_libraries(unimoc_motor_control_observer INTERFACE
    project_options
    project_warnings
    unimoc_motor_control_system
    unimoc_settings)

add_library(unimoc_motor_control_control INTERFACE)
target_include_directories(unimoc_motor_control_control INTERFACE
    ${UNIMOC_MOTOR_CONTROL_INCLUDE_DIR})
target_link_libraries(unimoc_motor_control_control INTERFACE
    project_options
    project_warnings
    unimoc_motor_control_system
    unimoc_settings)

add_library(unimoc_motor_control INTERFACE)
target_link_libraries(unimoc_motor_control INTERFACE
    unimoc_motor_control_system
    unimoc_motor_control_observer
    unimoc_motor_control_control)

add_library(unimoc::motor_control::system ALIAS unimoc_motor_control_system)
add_library(unimoc::motor_control::observer ALIAS unimoc_motor_control_observer)
add_library(unimoc::motor_control::control ALIAS unimoc_motor_control_control)
add_library(unimoc::motor_control ALIAS unimoc_motor_control)
