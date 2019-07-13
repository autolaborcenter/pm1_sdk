//
// Created by User on 2019/7/10.
//

#include <iostream>
#include "utilities/serial_port/serial_port.hh"

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

struct frame_t {
    uint8_t
        rho0   : 4,
        crc    : 3,
        zero_a : 1,
        
        rho1   : 7,
        zero_b : 1,
        
        theta0 : 6,
        rho2   : 1,
        zero_c : 1,
        
        theta1 : 7,
        zero_d : 1;
};

template<class t>
union msg_union_t {
    t       value;
    uint8_t bytes[sizeof(t)]{};
};

class parser_t {
    uint8_t buffer[256]{}, *ptr = buffer;

public:
    bool operator()(uint8_t byte) {
        msg_union_t<frame_t> frame{};
        if (byte > 127) {
            if (ptr - 3 >= buffer) {
                frame.bytes[0] = ptr[-3];
                frame.bytes[1] = ptr[-2];
                frame.bytes[2] = ptr[-1];
                frame.bytes[3] = byte;
                ptr = buffer;
                
                if (!crc_check(frame.bytes))
                    return false;
                
                constexpr static auto
                    k_rho   = 40.0 / (1u << 13u),
                    k_theta = 2 * 3.141592654 / 5760;
                
                uint16_t rho   = frame.value.rho0,
                         theta = frame.value.theta0;
                
                rho <<= 7u;
                rho |= frame.value.rho1;
                rho <<= 1u;
                rho |= frame.value.rho2;
                
                auto rho_ = rho * k_rho;
                
                if (rho_ < 0.01 || rho_ > 4)
                    return false;
                
                theta <<= 7u;
                theta |= frame.value.theta1;
                
                std::cout << rho_ << ", " << theta * k_theta << std::endl;
                
                return true;
            }
            ptr = buffer;
        }
        *ptr++ = byte;
        return false;
    }
    
    void reset() { ptr = buffer; }

private:
    template<class t>
    inline static uint8_t calculate(t value) {
        return static_cast<uint8_t>(value) & 0x07u;
    }
    
    inline static bool crc_check(const unsigned char *value) {
        static uint8_t crc_bit[]{
            0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
        };
        
        auto value0 = calculate(crc_bit[value[1]] + crc_bit[value[2]] + crc_bit[value[3]]),
             value1 = calculate(value[0] >> 4u);
        
        return value0 == value1;
    }
};

int main() {
    try {
        serial_port port("COM4", 460800);
        parser_t    parser;
        port << set_frequency(10);
        uint8_t buffer[256];
        while (true) {
            auto      actual = port.read(buffer, sizeof(buffer));
            for (auto item   = buffer; item < buffer + actual; ++item)
                parser(*item);
        }
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    
    return 0;
}
