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
        return static_cast<uint16_t>(begin[3]) << 8u | begin[2];
    }
    
    template<class iterator_t>
    inline uint8_t payload_length_of(const iterator_t &begin) {
        return begin[4];
    }
    
    union crc16_t {
        uint16_t _short;
        uint8_t  _byte;
    };
    
    template<class iterator_t>
    bool crc16_check(
        iterator_t begin,
        iterator_t end) {
        crc16_t crc{0xffff};
        while (begin < end) {
            crc._byte ^= *begin++;
            for (size_t i = 0; i < 8; ++i) {
                auto odd = crc._short & 1u;
                crc._short >>= 1u;
                if (odd) crc._short ^= 0xa001u;
            }
        }
        return crc._short == 0;
    }
    
    namespace resolution_coordinate {
        #define DEFINE_ELEMENT(NAME, TYPE, OFFSET) \
        TYPE NAME(const uint8_t *begin) { return *(TYPE *) (begin + (OFFSET)); }
        
        DEFINE_ELEMENT(time_stamp, uint32_t, 0)
        
        DEFINE_ELEMENT(x, int32_t, 4)
        
        DEFINE_ELEMENT(y, int32_t, 8)
        
        DEFINE_ELEMENT(z, int32_t, 12)
        
        DEFINE_ELEMENT(flags, uint8_t, 16)
        
        DEFINE_ELEMENT(address, uint8_t, 17)
        
        DEFINE_ELEMENT(pair_direction, uint16_t, 18)
        
        DEFINE_ELEMENT(time_passed, uint16_t, 20)
    
        #undef DEFINE_ELEMENT
    }
}


#endif //PM1_SDK_PROTOCOL_HPP
