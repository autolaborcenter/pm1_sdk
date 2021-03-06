﻿#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
    std::thread([] {
        while (true) {
            std::cout << autolabor::pm1::get_odometry().value.x << std::endl;
            autolabor::pm1::delay(0.1);
        }
    }).detach();
    
    auto result = autolabor::pm1::initialize();
    if (result)
        std::cout << result.value << std::endl;
    else {
        std::cerr << result.error_info << std::endl;
        return 1;
    }
    autolabor::pm1::unlock();
    
    while (true) {
        auto result = autolabor::pm1::go_straight(+0.2, 0.2);
        if (!result)std::cout << result.error_info << std::endl;
    
        autolabor::pm1::go_straight(-0.2, 0.2);
        autolabor::pm1::delay(1);
    }
}
