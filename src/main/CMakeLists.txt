cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 17)

if (${CMAKE_SIZEOF_VOID_P} MATCHES "8")
    set(ARCH "x64")
else ()
    set(ARCH "x86")
endif ()

set(CMAKE_DEBUG_POSTFIX "_debug")
message("${ARCH} selected")

add_subdirectory(internal/can)
add_subdirectory(internal/control_model)

set(NATIVE_LIBRARY
        # --------------------------
        # pm1 sdk
        pm1_sdk_native.h

        internal/exception_engine.hh
        internal/exception_engine.cc

        internal/process_controller.hh
        # --------------------------
        # serial port
        internal/serial/serial.h

        internal/serial/list_ports/list_ports_win.cc
        internal/serial/list_ports/list_ports_linux.cc
        internal/serial/list_ports/list_ports_osx.cc

        internal/serial/serial_port.hh
        internal/serial/serial_port.cc
        # --------------------------
        # extensions
        internal/raii/safe_shared_ptr.hh
        internal/raii/weak_lock_guard.hh
        internal/raii/weak_shared_lock.hh

        internal/time_extensions.h
        # --------------------------
        # can hub
        internal/can/pack.h

        internal/can/info.h

        internal/can/parser.hh
        internal/can/parser.cc

        internal/can/parse_engine.hh
        internal/can/parse_engine.cc
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

#add_library(pm1_sdk_native_injection SHARED
#        ${NATIVE_LIBRARY}
#        pm1_sdk_native_injection.cpp)

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

# 指定安装路径
install(TARGETS pm1_sdk_native DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/native)
install(FILES pm1_sdk_native.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/native)

#install(TARGETS pm1_sdk_native_injection DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/native/injection)
#install(FILES pm1_sdk_native.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/native/injection)

install(TARGETS pm1_sdk DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/lib)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/lib)

install(TARGETS pm1_sdk_shared DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/dll)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/out/${ARCH}/dll)

#if (MSVC)
#    set(CompilerFlags
#            CMAKE_CXX_FLAGS
#            CMAKE_CXX_FLAGS_DEBUG
#            CMAKE_CXX_FLAGS_RELEASE
#            CMAKE_C_FLAGS
#            CMAKE_C_FLAGS_DEBUG
#            CMAKE_C_FLAGS_RELEASE)
#    foreach (CompilerFlag ${CompilerFlags})
#        string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
#    endforeach ()
#endif (MSVC)
