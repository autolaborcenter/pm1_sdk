cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_DEBUG_POSTFIX "_debug")

add_subdirectory(internal/can)
add_subdirectory(internal/control_model)

add_library(pm1_sdk
        # --------------------------
        # pm1 sdk
        pm1_sdk.h
        pm1_sdk.cpp

        exception_engine.hh
        exception_engine.cc

        safe_shared_ptr.hh

        pm1_sdk_native.h
        pm1_sdk_native.cpp

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
        internal/raii/weak_shared_lock.hh

        internal/raii/weak_lock_guard.hh

        internal/time_extensions.h
        internal/serial_extension.h

        internal/can_define.h
        # --------------------------
        # can hub
        internal/can/pack.h
        internal/can/pack.cpp

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
        internal/chassis.hh
        internal/chassis.cc

        internal/odometry_t.hh
        internal/odometry_t.cc
        # --------------------------
        # others
        internal/rudder_controller.hh
        internal/rudder_controller.cc)

# 指定安装路径
install(TARGETS pm1_sdk DESTINATION ${PROJECT_SOURCE_DIR}/out)
install(FILES pm1_sdk.h DESTINATION ${PROJECT_SOURCE_DIR}/out)

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
