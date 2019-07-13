//
// Created by ydrml on 2019/7/13.
//

#ifndef PM1_SDK_PROTOCOL_HPP
#define PM1_SDK_PROTOCOL_HPP


#include <vector>

namespace marvelmind {
    constexpr uint8_t
        destination_address = 0xff,
        packet_type         = 0x47;
    
    template<class iterator_t>
    inline uint8_t destination_address_of(const iterator_t &begin) {
        return begin[0];
    }
    
    template<class iterator_t>
    inline uint8_t packet_type_of(const iterator_t &begin) {
        return begin[1];
    }
    
    template<class iterator_t>
    inline uint16_t data_code_of(const iterator_t &begin) {
        return static_cast<uint16_t>(begin[2]) << 8u | begin[3];
    }
    
    template<class iterator_t>
    inline uint8_t payload_length_of(const iterator_t &begin) {
        return begin[4];
    }
    
    template<class iterator_t>
    inline uint8_t crc_code_of(const iterator_t &begin, size_t n) {
        return static_cast<uint16_t>(begin[5 + n]) << 8u | begin[6 + n];
    }
}


#endif //PM1_SDK_PROTOCOL_HPP
