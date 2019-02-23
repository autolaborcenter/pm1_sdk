cmake_minimum_required(VERSION 3.13.4)
project(PM1_SDK)

set(CMAKE_CXX_STANDARD 11)

add_executable(PM1_SDK
        pm1/Api.h
        pm1/Api.cpp

        pm1/Extensions.h
        pm1/TimeExtensions.h

        main.cpp)
