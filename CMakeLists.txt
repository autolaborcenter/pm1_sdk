cmake_minimum_required(VERSION 3.13.4)
project(pm1_sdk)

set(CMAKE_CXX_STANDARD 11)

add_library(pm1_sdk SHARED
        #--------------------------
        src/main/pm1/api.h
        src/main/pm1/api.cpp

        src/main/pm1/time_extensions.h
        # --------------------------
        src/main/pm1/internal/serial/v8stdint.h
        src/main/pm1/internal/serial/serial.h
        src/main/pm1/internal/serial/serial.cc

        src/main/pm1/internal/serial/win.h
        src/main/pm1/internal/serial/win.cc

        src/main/pm1/internal/serial/unix.h
        src/main/pm1/internal/serial/unix.cc

        src/main/pm1/internal/serial/list_ports/list_ports_win.cc
        src/main/pm1/internal/serial/list_ports/list_ports_linux.cc
        src/main/pm1/internal/serial/list_ports/list_ports_osx.cc
        # --------------------------
        src/main/pm1/internal/can/can_message.h
        src/main/pm1/internal/can/can_message.cpp

        src/main/pm1/internal/can/can_define.h

        src/main/pm1/internal/can/parser.hh
        src/main/pm1/internal/can/parser.cc

        src/main/pm1/internal/chassis.hh
        src/main/pm1/internal/chassis.cc
        # --------------------------
        src/test/test.h
        src/test/test.cpp
        # --------------------------
        src/main/pm1/internal/chassis/pm1.h)

add_executable(pm1_sdk_test src/test/main.cpp)
link_directories(${PROJECT_SOURCE_DIR})
target_link_libraries(pm1_sdk_test pm1_sdk)
