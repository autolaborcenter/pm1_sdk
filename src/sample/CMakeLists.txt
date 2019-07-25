cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

include_directories(../main)
include_directories(../main/utilities)

# navigation
add_subdirectory(navigation_test)

# faselase
#add_executable(faselase_test
#        faselase_test/main.cpp)
#target_link_libraries(faselase_test pm1_sdk)

# test
add_executable(pm1_sdk_sample user_sample/continuous_control.cpp)
target_link_libraries(pm1_sdk_sample pm1_sdk)
