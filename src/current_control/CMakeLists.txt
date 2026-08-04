# current_control subsystem
#
# Contains the double-buffered current-control ISR and the background
# slow-update task that runs the mechanical/flux observers.

target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/CurrentControlIsr.cpp
  ${CMAKE_CURRENT_LIST_DIR}/SlowUpdate.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR})
