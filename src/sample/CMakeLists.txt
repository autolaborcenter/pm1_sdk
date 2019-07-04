cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

add_executable(pm1_sdk_sample test_sample/test_serial_break.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)
