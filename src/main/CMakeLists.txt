cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
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

set(NATIVE_LIBRARY
        # --------------------------
        # pm1 sdk
        pm1_sdk_native.h

        internal/api/exception_engine.hpp

        internal/process_controller.hh
        # --------------------------
        # serial port
        internal/serial/serial.h

        internal/serial/list_ports/list_ports_win.cc
        internal/serial/list_ports/list_ports_linux.cc

        internal/serial/macros.h
        internal/serial/serial_port.hh
        internal/serial/serial_port_win.cc
        internal/serial/serial_port_linux.cc
        internal/serial/serial_port_operators.h
        # --------------------------
        # extensions
        internal/raii/safe_shared_ptr.hpp
        internal/raii/weak_lock_guard.hpp
        internal/raii/weak_shared_lock.hpp

        internal/time_extensions.h
        # --------------------------
        # can hub
        internal/can/info.h
        internal/can/pack.h

        internal/can/circular_buffer.hpp

        internal/can/parser_t.hpp
        internal/can/parse_engine.hpp
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

        internal/chassis.hh
        internal/chassis.cc

        internal/odometry_t.hh
        internal/odometry_t.cc)

add_library(pm1_sdk_native SHARED
        ${NATIVE_LIBRARY}
        pm1_sdk_native.cpp)

add_library(pm1_sdk
        ${NATIVE_LIBRARY}
        pm1_sdk_native.cpp

        pm1_sdk.h
        pm1_sdk.cpp)

add_library(pm1_sdk_shared SHARED
        ${NATIVE_LIBRARY}
        pm1_sdk_native.cpp

        pm1_sdk.h
        pm1_sdk.cpp)

if (UNIX)
    target_link_libraries(pm1_sdk pthread atomic)
    target_link_libraries(pm1_sdk_shared pthread atomic)
    target_link_libraries(pm1_sdk_native pthread atomic)
endif ()

# 指定安装路径
install(TARGETS pm1_sdk_native DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/native)
install(FILES pm1_sdk_native.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/native)

install(TARGETS pm1_sdk DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/lib)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/lib)

install(TARGETS pm1_sdk_shared DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/shared)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/shared)
