cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

include_directories(../main)
include_directories(../main/utilities)

# navigation
add_executable(pm1_sdk_sample
        navigation_test/pid/point_t.h
        navigation_test/pid/path_manage.hpp
        navigation_test/pid/shape_t.hpp
        navigation_test/pid/virtual_light_sensor_t.hpp
        navigation_test/pid/path_follower_t.hpp

        navigation_test/main.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)

# faselase
#add_executable(faselase_test
#        faselase_test/main.cpp)
#target_link_libraries(faselase_test pm1_sdk)

# marvelmind
#add_executable(marvelmind_serial_test
#        navigation_test/marvelmind/protocol.hpp
#        navigation_test/marvelmind/parser_t.hpp
#        navigation_test/marvelmind/main.cpp)
#target_link_libraries(marvelmind_serial_test pm1_sdk)

# test
#add_executable(pm1_sdk_sample
#        test_sample/test_sdk_5.cpp)
#target_link_libraries(pm1_sdk_sample pm1_sdk)
