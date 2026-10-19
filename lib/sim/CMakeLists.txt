# Deterministic motor plant simulation (inverter, motor models, shaft, sensors).
# Header-only and HAL-free; the native_sim hardware layer binds it separately.
add_library(unimoc_sim INTERFACE)
target_include_directories(unimoc_sim INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(unimoc_sim INTERFACE
    project_options
    project_warnings
    unimoc_units
    unimoc_settings
    unimoc_motor_control_system)

add_library(unimoc::sim ALIAS unimoc_sim)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
