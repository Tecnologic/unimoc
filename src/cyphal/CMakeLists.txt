target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  # heartbeat.hpp is header-only; list it here so IDEs pick it up
  ${CMAKE_CURRENT_SOURCE_DIR}/heartbeat.hpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
