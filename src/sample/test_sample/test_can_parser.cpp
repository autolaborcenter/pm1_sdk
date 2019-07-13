//
// Created by User on 2019/7/12.
//

#include <iostream>

#include "../../main/internal/can/pack.h"
#include "../../main/internal/can/parser_t.hpp"
#include "utilities/serial_parser/parse_engine.hpp"

#include "../../main/internal/can_define.h"

int main() {
    auto packet = autolabor::can::pack<autolabor::pm1::unit<>::emergency_stop>();
    auto begin  = packet.bytes,
         end    = packet.bytes + sizeof(decltype(packet));
    
    std::cout << packet.data.to_string() << std::endl;
    
    autolabor::parse_engine_t<autolabor::can::parser_t> engine;
    engine(begin, end, [](const autolabor::can::parser_t::result_t &result) {
        switch (result.type) {
            case parser_t::result_type_t::nothing:
                std::cout << "nothing" << std::endl;
                break;
            case parser_t::result_type_t::signal_failed:
                std::cout << "signal failed" << std::endl;
                break;
            case parser_t::result_type_t::message_failed:
                std::cout << "message failed" << std::endl;
                break;
            case parser_t::result_type_t::signal:
                std::cout << result.signal.to_string() << std::endl;
                break;
            case parser_t::result_type_t::message:
                std::cout << result.message.to_string() << std::endl;
                break;
        }
    });
    
    return 0;
}
