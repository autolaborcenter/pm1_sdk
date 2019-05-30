#include <iostream>
#include <thread>
#include "pm1_sdk.h"

int main() {
    using namespace autolabor::pm1;
    
    auto result = initialize();
    if (result)
        std::cout << result.value << std::endl;
    else {
        std::cerr << result.error_info << std::endl;
        return 1;
    }
    unlock();
    
    for (auto i = 0; i < 200; ++i) {
        drive(2, 0);
        std::cout << get_odometry().value.vx << std::endl;
        delay(0.05);
    }
    for (auto i = 0; i < 20; ++i) {
        drive(0, 0);
        delay(0.05);
    }
}
