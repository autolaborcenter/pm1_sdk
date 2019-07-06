cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

add_executable(pm1_sdk_sample
        navigation_test/pid/point_t.h
        navigation_test/pid/path_manage.hpp
        navigation_test/pid/shape_t.hpp
        navigation_test/pid/virtual_light_sensor_t.hpp
        navigation_test/pid/path_follower_t.hpp

        navigation_test/main.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)
