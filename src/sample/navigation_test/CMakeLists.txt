cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

include_directories($ENV{EIGEN_ROOT}/include)

# navigation
add_executable(navigation
        path_follower/point_t.h
        path_follower/path_manage.hpp
        path_follower/shape_t.hpp
        path_follower/virtual_light_sensor_t.hpp
        path_follower/path_follower_t.hpp

        marvelmind/protocol.h
        marvelmind/protocol.cpp
        marvelmind/parser_t.hpp
        marvelmind/mobile_beacon_t.hh
        marvelmind/mobile_beacon_t.cc

        fusion_loacator/transformer_t.hpp
        fusion_loacator/fusion_locator_t.hh
        fusion_loacator/fusion_locator_t.cc

        particle_filter/particle_filter_t.hh
        particle_filter/particle_filter_t.cc

        pose_t.h
        navigation_system_t.hh
        navigation_system_t.cc
        main.cpp)
target_link_libraries(navigation pm1_sdk)

add_executable(test
        fusion_loacator/transformer_t.hpp
        fusion_loacator/fusion_locator_t.hh
        fusion_loacator/fusion_locator_t.cc
        fusion_loacator/test.cpp)
