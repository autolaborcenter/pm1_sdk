//
// Created by User on 2019/7/12.
//

#include "../../main/internal/can/pack.h"
#include "../../main/internal/can/parser_t.hpp"
#include "../../main/internal/can/parse_engine.hpp"

#include "../../main/internal/can_define.h"

int main() {
    autolabor::can::parser_t parser;
    
    auto packet = autolabor::can::pack<autolabor::pm1::unit<>::emergency_stop>();
    auto begin  = packet.bytes,
         end    = packet.bytes + sizeof(decltype(packet));
    parser(begin, end);
    
    begin = packet.bytes;
    end   = packet.bytes + sizeof(decltype(packet));
    
    autolabor::parse_engine_t<autolabor::can::parser_t> engine;
    engine(begin, end, [](const autolabor::can::parser_t::result_t &) {});
    
    return 0;
}
