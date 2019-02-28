cmake_minimum_required(VERSION 3.13.4)

set(CMAKE_CXX_STANDARD 11)

add_executable(pm1_sdk_test
        test.h
        test.cpp

        main.cpp)

target_link_libraries(pm1_sdk_test pm1_sdk)
