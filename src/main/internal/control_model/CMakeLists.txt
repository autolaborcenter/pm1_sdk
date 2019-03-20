cmake_minimum_required(VERSION 3.12)
project(pm1_control_model C)

set(CMAKE_C_STANDARD 11)

add_library(pm1_control_model
        pi.h

        chassis_config_t.h

        motor_map.h

        model.h
        model.c

        optimization.h
        optimization.c)
