cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

# navigation
add_executable(temp_test test_odometry.cpp)
target_link_libraries(temp_test pm1_sdk)
