add_unimoc_test(three_phase_system_test three_phase_system_test.cpp
    LIBRARIES unimoc_motor_control_system unimoc_settings)
add_unimoc_test(stator_system_test stator_system_test.cpp
    LIBRARIES unimoc_motor_control_system)
add_unimoc_test(rotor_system_test rotor_system_test.cpp
    LIBRARIES unimoc_motor_control_system)
add_unimoc_test(rotor_angle_test rotor_angle_test.cpp
    LIBRARIES unimoc_motor_control_system)
add_unimoc_test(sin_cos_test sin_cos_test.cpp
    LIBRARIES unimoc_motor_control_system)
add_unimoc_test(svm_test svm_test.cpp
    LIBRARIES unimoc_motor_control_control)
add_unimoc_test(core_control_test core_control_test.cpp
    LIBRARIES unimoc_motor_control_control)

add_unimoc_test(excitation_controller_test excitation_controller_test.cpp
    LIBRARIES unimoc_motor_control_control)
add_unimoc_test(position_controller_test position_controller_test.cpp
    LIBRARIES unimoc_motor_control_control unimoc_motor_control_observer)

add_unimoc_test(excitation_observer_test excitation_observer_test.cpp
    LIBRARIES unimoc_motor_control_observer)
add_unimoc_test(position_tracker_test position_tracker_test.cpp
    LIBRARIES unimoc_motor_control_observer)
add_unimoc_test(hfi_test hfi_test.cpp
    LIBRARIES unimoc_motor_control_observer)

add_custom_target(motor_control_tests DEPENDS
    three_phase_system_test
    stator_system_test
    rotor_system_test
    rotor_angle_test
    sin_cos_test
    svm_test
    core_control_test
    excitation_controller_test
    position_controller_test
    excitation_observer_test
    position_tracker_test
    hfi_test)