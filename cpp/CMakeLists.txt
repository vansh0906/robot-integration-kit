cmake_minimum_required(VERSION 3.14)
project(rik_kinematics CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_library(rik_kinematics INTERFACE)
target_include_directories(rik_kinematics INTERFACE include)

enable_testing()
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE rik_kinematics)
add_test(NAME kinematics COMMAND test_kinematics ${CMAKE_CURRENT_SOURCE_DIR}/../tests/golden)
