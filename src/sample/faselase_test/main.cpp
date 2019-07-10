//
// Created by User on 2019/7/10.
//

#include <iostream>
#include "../../main/internal/serial/serial_port.hh"

template<size_t _size>
struct byte_array {
    constexpr static auto size = _size;
    uint8_t               array[size];
};

constexpr byte_array<8> set_frequency(uint8_t frequency) {
    return {'#', 'S', 'F', ' ',
            static_cast<uint8_t>('0' + frequency / 10),
            static_cast<uint8_t>('0' + frequency % 10),
            '\r',
            '\n'};
}

template<size_t size>
inline serial_port &operator<<(
    serial_port &port,
    const byte_array<size> &msg) noexcept {
    port.send(msg.array, size);
    return port;
}

int main() {
    try {
        serial_port port("COM4", 460800);
        port << set_frequency(0);
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    
    return 0;
}
