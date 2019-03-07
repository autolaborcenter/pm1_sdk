cmake_minimum_required(VERSION 3.13.4)

set(CMAKE_CXX_STANDARD 11)

add_library(can_msg
        can_define.h

        can_message.h
        can_message.cpp

        parser.hh
        parser.cc parse_engine.cpp parse_engine.h)
