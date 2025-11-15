# CMakeLists.txt for tests directory
cmake_minimum_required(VERSION 4.00)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main # Or a specific release tag
)
FetchContent_MakeAvailable(googletest)

# use googletest if tests are enabled
enable_testing()


# Add test executable
add_executable(system_test system_test.cpp)
target_link_libraries(system_test gtest gtest_main)
target_link_libraries(system_test unimoc_lib)
# Add test to CMake
add_test(NAME SystemTest COMMAND system_test)

# Add three phase system test executable
add_executable(three_phase_system_test three_phase_system_test.cpp)
target_link_libraries(three_phase_system_test gtest gtest_main)
target_link_libraries(three_phase_system_test unimoc_lib)
# Add test to CMake
add_test(NAME ThreePhaseSystemTest COMMAND three_phase_system_test)