cmake_minimum_required(VERSION 3.13.4)
project(PM1_SDK)

set(CMAKE_CXX_STANDARD 11)

add_executable(PM1_SDK
        pm1/sdk/Api.h
        pm1/sdk/Api.cpp

        pm1/sdk/Extensions.h
        pm1/sdk/TimeExtensions.h

        main.cpp)
