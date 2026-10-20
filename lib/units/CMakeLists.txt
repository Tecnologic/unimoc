add_library(unimoc_units INTERFACE)

target_include_directories(unimoc_units INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(unimoc_units INTERFACE project_options project_warnings)

add_library(unimoc::units ALIAS unimoc_units)
