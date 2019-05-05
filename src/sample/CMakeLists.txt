cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 17)

add_executable(pm1_sdk_sample test_sample/test_sdk_10.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)
