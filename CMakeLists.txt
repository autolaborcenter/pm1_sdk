cmake_minimum_required(VERSION 3.13.4)
project(PM1_SDK)

set(CMAKE_CXX_STANDARD 11)

add_executable(PM1_SDK
        pm1/api.h
        pm1/api.cpp

        pm1/extensions.h
        pm1/time_extensions.h

        pm1/internal/serial/v8stdint.h

        pm1/internal/serial/serial.h
        pm1/internal/serial/serial.cc

        pm1/internal/serial/win.h
        pm1/internal/serial/win.cc

        pm1/internal/serial/unix.h
        pm1/internal/serial/unix.cc

        pm1/internal/serial/list_ports/list_ports_win.cc
        pm1/internal/serial/list_ports/list_ports_linux.cc
        pm1/internal/serial/list_ports/list_ports_osx.cc

        pm1/internal/can_message.h
        pm1/internal/can_message.cpp

        pm1/internal/chassis.hh
        pm1/internal/chassis.cc

        main.cpp)
