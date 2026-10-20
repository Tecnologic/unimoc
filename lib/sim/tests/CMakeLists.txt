add_unimoc_test(sim_pmsm_model_test pmsm_model_test.cpp
    LIBRARIES unimoc_sim)
add_unimoc_test(sim_mechanical_model_test mechanical_model_test.cpp
    LIBRARIES unimoc_sim)
add_unimoc_test(sim_hall_sensor_test hall_sensor_test.cpp
    LIBRARIES unimoc_sim)
add_unimoc_test(sim_motor_simulation_test motor_simulation_test.cpp
    LIBRARIES unimoc_sim)

add_custom_target(sim_tests DEPENDS
    sim_pmsm_model_test
    sim_mechanical_model_test
    sim_hall_sensor_test
    sim_motor_simulation_test)
