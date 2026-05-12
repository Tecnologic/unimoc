# make modm available to this hardware type
include(../../cmake/Modm.cmake)

add_subdirectory(modm)

add_executable(${CMAKE_PROJECT_NAME})

target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  adc.cpp
  sincos.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ../
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  modm_arch_options
  modm_options
  modm_warnings
  modm)

# Add the hardware directory to the linker search path so the linker can find
# modm/link/linkerscript.ld relative to this directory (not CMAKE_SOURCE_DIR).
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
  -L${CMAKE_CURRENT_LIST_DIR}
)

# Outputs hex and bin files.
modm_targets_create(${CMAKE_PROJECT_NAME})

