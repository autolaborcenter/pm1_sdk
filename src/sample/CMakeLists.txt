cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

add_executable(pm1_sdk_sample
        navigation_test/trajectory.hpp

        navigation_test/pm1_trajectory_t.hh
        navigation_test/pm1_trajectory_t.cc

        navigation_test/pm_1_precise_trajectory_t.hh
        navigation_test/pm_1_precise_trajectory_t.cc

        navigation_test/main.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)
