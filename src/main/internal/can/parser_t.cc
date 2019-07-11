//
// Created by ydrml on 2019/2/26.
//

#include <cstring>
#include "parser_t.hh"

using namespace autolabor::can;

parser_t::parser_t(const parser_t &others) : state(others.state) {
    std::memcpy(bytes, others.bytes, sizeof(bytes));
}

parser_t::result_t parser_t::operator()(uint8_t byte) {
    // 保存当前状态
    auto last = state.state();
    switch (last) {
        case state_type::origin:
            // 初始状态，等待起始位
            if (byte == 0xfe)
                state.value++;
            break;
        case state_type::determine:
            // 判断有无数据域，调整状态
            bytes[1] = byte;
            state.value = byte & (1u << 5u) ? 6 : 2;
            break;
        case state_type::signal: {
            // 保存数据字节
            bytes[state.value++] = byte;
            if (state.state() == last)
                return {parser_t::result_type::nothing};
            
            state.value = 0;
            result_t result{crc_check(sgn_buffer)
                            ? parser_t::result_type::signal
                            : parser_t::result_type::signal_failed};
            result.signal = sgn_buffer;
            return result;
        }
        case state_type::message: {
            // 保存数据字节
            bytes[state.value++ - 4] = byte;
            if (state.state() == last)
                return {parser_t::result_type::nothing};
            
            state.value = 0;
            result_t result{crc_check(msg_buffer)
                            ? parser_t::result_type::message
                            : parser_t::result_type::message_failed};
            result.message = msg_buffer;
            return result;
        }
        case state_type::ending:
            throw std::out_of_range("switch out of range");
    }
    return {parser_t::result_type::nothing};
}

parser_t::state_type parser_t::state_t::state() {
    return value == 0
           ? state_type::origin
           : value == 1
             ? state_type::determine
             : value < 6
               ? state_type::signal
               : value < 18
                 ? state_type::message
                 : state_type::ending;
}
