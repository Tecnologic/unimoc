
add_executable(${CMAKE_PROJECT_NAME})

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
