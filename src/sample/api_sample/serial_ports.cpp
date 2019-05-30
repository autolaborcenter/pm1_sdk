//
// Created by User on 2019/4/1.
//

#include <iostream>
#include "pm1_sdk.h"

int main() {
    for (const auto &port : autolabor::pm1::serial_ports())
        std::cout << port << std::endl;
    return 0;
}
