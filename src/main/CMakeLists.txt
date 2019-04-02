cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_DEBUG_POSTFIX "_debug")

add_subdirectory(internal/can)
add_subdirectory(internal/control_model)

add_library(pm1_sdk
        # --------------------------
        exception.h
        pm1_sdk.h
        #        pm1_sdk.cpp
        pm1_sdk_sync.cpp
        # --------------------------
        internal/serial/serial.h

        internal/serial/list_ports/list_ports_win.cc
        internal/serial/list_ports/list_ports_linux.cc
        internal/serial/list_ports/list_ports_osx.cc

        internal/serial/serial_port.hh
        internal/serial/serial_port.cc
        # --------------------------
        internal/time_extensions.h
        internal/serial_extension.h

        internal/can_define.h
        # --------------------------
        internal/can/pack.h
        internal/can/pack.cpp

        internal/can/info.h

        internal/can/parser.hh
        internal/can/parser.cc

        internal/can/parse_engine.hh
        internal/can/parse_engine.cc
        # --------------------------
        internal/control_model/pi.h

        internal/control_model/chassis_config_t.h
        internal/control_model/chassis_config_t.c

        internal/control_model/motor_map.h

        internal/control_model/model.h
        internal/control_model/model.c

        internal/control_model/optimization.h
        internal/control_model/optimization.c
        # --------------------------
        internal/chassis.hh
        internal/chassis.cc

        internal/odometry_t.hh
        internal/odometry_t.cc
        # --------------------------
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
