add_library(unimoc_settings INTERFACE)

target_include_directories(unimoc_settings INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(unimoc_settings INTERFACE
    project_options
    project_warnings
    unimoc_units
    unimoc_cyphal)

add_library(unimoc::settings ALIAS unimoc_settings)
