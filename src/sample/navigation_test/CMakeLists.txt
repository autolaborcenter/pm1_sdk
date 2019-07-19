cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

include_directories($ENV{EIGEN_INCLUDE_DIR})

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

        matcher/stamped_t.h
        matcher/matcher_t.hpp
        matcher/transformer_t.hpp
        matcher/fusion_locator_t.hh
        matcher/fusion_locator_t.cc

        pose_t.h
        navigation_system_t.hh
        navigation_system_t.cc
        main.cpp)
target_link_libraries(navigation pm1_sdk)

add_executable(test
        matcher/stamped_t.h
        matcher/matcher_t.hpp
        matcher/transformer_t.hpp
        matcher/fusion_locator_t.hh
        matcher/fusion_locator_t.cc
        matcher/test.cpp)
