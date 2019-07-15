//
// Created by User on 2019/7/11.
//

#include <iostream>

#include "utilities/serial_port/serial_port.hh"
#include "utilities/serial_parser/parse_engine.hpp"
#include "parser_t.hpp"

namespace marvelmind {
    void marvelmind_receive(std::string com_name) {
        using engine_t = autolabor::parse_engine_t<marvelmind::parser_t>;
        
        serial_port port("COM11", 115200);
        engine_t    engine;
        uint8_t     buffer[256];
        while (true)
            engine(buffer, buffer + port.read(buffer, sizeof(buffer)),
                   [](const typename engine_t::result_t &result) {
                       switch (result.type) {
                           case marvelmind::parser_t::result_type_t::nothing:
                               break;
                           case marvelmind::parser_t::result_type_t::failed:
                               std::cout << "crc check failed" << std::endl;
                               break;
                           case marvelmind::parser_t::result_type_t::success:
                               using namespace marvelmind::resolution_coordinate;
                               auto begin = result.bytes.data() + 5;
                               std::cout << x(begin) / 1000.0 << ", "
                                         << y(begin) / 1000.0 << ", "
                                         << z(begin) / 1000.0 << std::endl;
                               break;
                       }
                   });
    }
}
