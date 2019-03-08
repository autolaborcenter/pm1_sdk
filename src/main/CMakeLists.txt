cmake_minimum_required(VERSION 3.13.4)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_DEBUG_POSTFIX "_debug")

add_subdirectory(internal/can)
add_library(pm1_sdk
        # --------------------------
        pm1_sdk.h
        pm1_sdk.cpp
        # --------------------------
        internal/time_extensions.h
        # --------------------------
        internal/serial/v8stdint.h
        internal/serial/serial.h
        internal/serial/serial.cc

        internal/serial/win.h
        internal/serial/win.cc

        internal/serial/unix.h
        internal/serial/unix.cc

        internal/serial/list_ports/list_ports_win.cc
        internal/serial/list_ports/list_ports_linux.cc
        internal/serial/list_ports/list_ports_osx.cc
        # --------------------------
        internal/can/can_message.h
        internal/can/can_message.cpp

        internal/can/parser.hh
        internal/can/parser.cc

        internal/chassis.hh
        internal/chassis.cc
        # --------------------------
        internal/mechanical.h
        internal/can_define.h)
target_link_libraries(pm1_sdk can_msg)

# 指定安装路径
install(TARGETS pm1_sdk DESTINATION ${PROJECT_SOURCE_DIR}/out)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/out)
