//
// Created by User on 2019/4/1.
//

#include <iostream>
#include <chrono>
#include "pm1_sdk.h"

int main() {
    using namespace autolabor::pm1;
    using namespace std::chrono_literals;
    
    if (!initialize()) return 1;
    auto time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - time < 3s) {
        drive(0.2, 0);
        std::cout << get_odometry().value.x << std::endl;
    }
}
