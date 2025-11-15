cmake_minimum_required(VERSION 3.20)

# remove the warning about CMP0175
cmake_policy(SET CMP0175 OLD)

# Projektname und Version
project(unimoc LANGUAGES C CXX)

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

if(HARDWARE_DIRECTORY)
  target_include_directories(project_options INTERFACE ${CMAKE_SOURCE_DIR}/hardware)
  # add hardware files
  add_subdirectory(${HARDWARE_DIRECTORY})
else()
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

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC unimoc_lib project_options project_warnings)




