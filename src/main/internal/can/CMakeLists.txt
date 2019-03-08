cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 11)

add_library(can_msg
        can_message.h
        can_message.cpp

        parser.hh
        parser.cc

        parse_engine.hh
        parse_engine.cc)
