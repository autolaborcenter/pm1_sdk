cmake_minimum_required(VERSION 3.12)
project(auto_can)

set(CMAKE_CXX_STANDARD 11)

add_library(auto_can
        info.h

        parser.hh
        parser.cc

        parse_engine.hh
        parse_engine.cc

        pack.h)
