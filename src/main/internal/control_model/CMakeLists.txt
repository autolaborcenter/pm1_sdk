cmake_minimum_required(VERSION 3.12)
project(pm1_control_model C)

set(CMAKE_C_STANDARD 11)

add_library(pm1_control_model chassis_t.h chassis_t.c)

message("hello")
