
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
