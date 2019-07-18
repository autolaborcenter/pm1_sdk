//
// Created by ydrml on 2019/7/13.
//

#ifndef MARVELMIND_PROTOCOL_H
#define MARVELMIND_PROTOCOL_H


#include <cstdint>
#include <cstddef>

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
        uint32_t time_stamp(const uint8_t *);
    
        int32_t x(const uint8_t *);
    
        int32_t y(const uint8_t *);
    
        int32_t z(const uint8_t *);
    
        uint8_t flags(const uint8_t *);
    
        uint8_t address(const uint8_t *);
    
        uint16_t pair_direction(const uint8_t *);
    
        uint16_t time_passed(const uint8_t *);
    }
}


#endif // MARVELMIND_PROTOCOL_H
