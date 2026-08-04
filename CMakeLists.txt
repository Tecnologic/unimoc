cmake_minimum_required(VERSION 4.00)

if(POLICY CMP0175)
  cmake_policy(SET CMP0175 NEW)
endif()

# Projektname und Version
project(unimoc LANGUAGES C CXX ASM)

add_library(project_options INTERFACE)

# Colors is unsupported vscode, can also be turend off in cmake/StandardProjectSettings.cmake
target_compile_options(project_options INTERFACE -fdiagnostics-color=never)

add_library(project_warnings INTERFACE)
include(cmake/Cache.cmake)

# recomended warniings for modm projects
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/StaticAnalyzers.cmake)

# generate docs only for the main project
include(cmake/Doxygen.cmake)
enable_doxygen()

option(ENABLE_TESTS "Enable tests" OFF)
option(HARDWARE_DIRECTORY "Directory for hardware files" "${CMAKE_SOURCE_DIR}/hardware/batterycasecontroller")
option(RUNNING_ON_HOST "Set to ON if running on host (not on target)" OFF)

if(HARDWARE_DIRECTORY)
  # Always expose the hardware/ include path so platform-agnostic headers
  # (e.g. HardwareLimits.hpp) are available to all targets, including tests.
  target_include_directories(project_options INTERFACE ${CMAKE_SOURCE_DIR}/hardware)
  # add hardware files
  add_subdirectory(${HARDWARE_DIRECTORY})

  if (RUNNING_ON_HOST)
    target_compile_definitions(project_options INTERFACE RUNNING_ON_HOST)
  else ()
    target_compile_definitions(project_options INTERFACE RUNNING_ON_TARGET)

    # gitversion is only needed for firmware targets; include it here so
    # hosted/test builds do not require the submodule to be initialised.
    INCLUDE(${CMAKE_SOURCE_DIR}/lib/gitversion/cmake.cmake)
    # attach gitversion to the firmware executable created by the hardware directory
    TARGET_GIT_VERSION_INIT(${CMAKE_PROJECT_NAME})
  endif()
else()
  # Even without a hardware implementation directory, expose the hardware/
  # header path so that HardwareLimits.hpp is found during hosted builds and
  # tests.  No platform-specific source files are compiled.
  target_include_directories(project_options INTERFACE ${CMAKE_SOURCE_DIR}/hardware)
  message(WARNING "HARDWARE_DIRECTORY is not set, skipping hardware files.")
endif()

if(ENABLE_TESTS)
  add_subdirectory(tests)
else()
  # add the projects code folders
  add_subdirectory(src)
endif()

# add universal code as library
add_subdirectory(lib)

# The firmware executable only exists when tests are disabled (src/ is added).
if(NOT ENABLE_TESTS)
  target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC unimoc_lib project_options project_warnings)
endif()


