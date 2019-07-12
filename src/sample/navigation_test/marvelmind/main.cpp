//
// Created by User on 2019/7/11.
//

#include <iostream>
#include <chrono>
#include <thread>
#include "../../../main/internal/serial/serial_port.hh"

class parser_t {
public:
    struct result_t {
        enum class type_t : uint8_t {
            none,
            reset,
            crc_error,
            success
        };
    
        type_t   type;
        uint32_t time_stamp;
        uint8_t  address, flags;
        uint16_t orientation, delay;
        float    x, y, z;
    };
    
    template<size_t payload_size>
    struct packet_t {
        uint8_t destination_address = 0xff,
                packet_type         = 0x47;
        
        uint16_t data_code = 0;
        uint8_t  size      = payload_size,
                 payload[payload_size]{};
        
        uint16_t crc_bytes = 0;
    };
};

int main() {
    std::unique_ptr<serial_port> port = nullptr;
    
    return 0;
}
