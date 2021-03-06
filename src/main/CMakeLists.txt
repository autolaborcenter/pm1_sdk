﻿cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

if (WIN32)
    set(PLATFORM "win")
elseif (UNIX)
    set(PLATFORM "unix")
endif ()

if (${CMAKE_SIZEOF_VOID_P} MATCHES "8")
    set(BITS "x64")
else ()
    set(BITS "x86")
endif ()

set(ARCH ${PLATFORM}_${BITS})

set(CMAKE_DEBUG_POSTFIX "_debug")
message("${ARCH} selected")

include_directories(./)

set(UTILITIES
        # --------------------------
        # serial port
        utilities/serial_port/serial.h

        utilities/serial_port/list_ports/list_ports_win.cc
        utilities/serial_port/list_ports/list_ports_linux.cc

        utilities/serial_port/macros.h
        utilities/serial_port/serial_port.hh
        utilities/serial_port/serial_port_win.cc
        utilities/serial_port/serial_port_linux.cc
        utilities/serial_port/serial_port_operators.h
        # --------------------------
        # serial parser
        utilities/serial_parser/memory.hpp
        utilities/serial_parser/parse_engine.hpp
        # --------------------------
        # api
        utilities/time/time_extensions.h
        utilities/time/stamped_t.h
        utilities/time/matcher_t.hpp

        utilities/odometry_t.hpp
        utilities/differentiator_t.hpp

        utilities/raii/safe_shared_ptr.hpp
        utilities/raii/weak_lock_guard.hpp
        utilities/raii/weak_shared_lock.hpp

        utilities/raii/exception_engine.hpp)

set(NATIVE_LIBRARY
        # --------------------------
        # pm1 sdk
        pm1_sdk_definitions.h
        pm1_sdk_native.h
        pm1_sdk_native.cpp
        internal/process_controller.hpp
        # --------------------------
        # can hub
        internal/can/protocol.hpp
        internal/can/pack.hpp
        internal/can/parser_t.hpp
        # --------------------------
        # pm1 control model
        internal/control_model/pi.h

        internal/control_model/chassis_config_t.h
        internal/control_model/chassis_config_t.c

        internal/control_model/motor_map.h

        internal/control_model/model.h
        internal/control_model/model.c

        internal/control_model/optimization.h
        internal/control_model/optimization.c
        # --------------------------
        # pm1 chassis
        internal/can_define.h

        internal/pm1_odometry_t.hh
        internal/pm1_odometry_t.cc

        internal/chassis.hh
        internal/chassis.cc)

add_library(pm1_sdk_native SHARED
        ${UTILITIES}
        ${NATIVE_LIBRARY})

add_library(pm1_sdk
        ${UTILITIES}
        ${NATIVE_LIBRARY}

        pm1_sdk.h
        pm1_sdk.cpp)

add_library(pm1_sdk_shared SHARED
        ${UTILITIES}
        ${NATIVE_LIBRARY}

        pm1_sdk.h
        pm1_sdk.cpp)

if (UNIX)
    target_link_libraries(pm1_sdk pthread atomic)
    target_link_libraries(pm1_sdk_shared pthread atomic)
    target_link_libraries(pm1_sdk_native pthread atomic)
endif ()

set(INSTALL_DIR build)

# 指定安装路径
install(TARGETS pm1_sdk_native DESTINATION ${PROJECT_SOURCE_DIR}/${INSTALL_DIR}/${ARCH}/native)
install(FILES pm1_sdk_native.h DESTINATION ${PROJECT_SOURCE_DIR}/${INSTALL_DIR}/${ARCH}/native)

install(TARGETS pm1_sdk DESTINATION ${PROJECT_SOURCE_DIR}/${INSTALL_DIR}/${ARCH}/lib)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/${INSTALL_DIR}/${ARCH}/lib)

install(TARGETS pm1_sdk_shared DESTINATION ${PROJECT_SOURCE_DIR}/${INSTALL_DIR}/${ARCH}/shared)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/${INSTALL_DIR}/${ARCH}/shared)
