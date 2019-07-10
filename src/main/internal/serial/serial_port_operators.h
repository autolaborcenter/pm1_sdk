//
// Created by User on 2019/7/10.
//

#ifndef PM1_SDK_SERIAL_PORT_OPERATORS_H
#define PM1_SDK_SERIAL_PORT_OPERATORS_H

#include "serial_port.hh"

inline serial_port &operator<<(serial_port &port,
                               const char *text) noexcept {
    port.send((uint8_t *) text, std::strlen(text));
    return port;
}

inline serial_port &operator<<(serial_port &port,
                               const std::string &text) noexcept {
    port.send((uint8_t *) text.c_str(), text.length());
    return port;
}

#endif //PM1_SDK_SERIAL_PORT_OPERATORS_H
