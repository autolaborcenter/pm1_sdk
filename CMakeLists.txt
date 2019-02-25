cmake_minimum_required(VERSION 3.13.4)
project(PM1_SDK)

set(CMAKE_CXX_STANDARD 11)

add_executable(PM1_SDK
        pm1/api.h
        pm1/api.cpp

        pm1/extensions.h
        pm1/time_extensions.h

        main.cpp pm1/internal/chassis.cpp pm1/internal/chassis.h)
