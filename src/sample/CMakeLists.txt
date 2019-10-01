cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

include_directories(../main)

add_subdirectory(test_sample)

# test
add_executable(pm1_sdk_sample user_sample/initialization.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)
