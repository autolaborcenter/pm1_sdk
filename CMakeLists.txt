cmake_minimum_required(VERSION 3.13)
project(PM1_SDK)

set(CMAKE_CXX_STANDARD 11)

add_library(PM1_SDK pm1/sdk/World.cpp pm1/sdk/World.h)
