#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
    std::thread([] {
        while (true) {
            std::cout << autolabor::pm1::get_odometry().value.x << std::endl;
            autolabor::pm1::delay(0.1);
        }
    }).detach();
    
    while (true) {
        auto result = autolabor::pm1::initialize();
        if (result)
            std::cout << result.value << std::endl;
        else {
            std::cerr << result.error_info << std::endl;
            continue;
        }
        
        autolabor::pm1::unlock();
        
        autolabor::pm1::go_straight_timing(+0.2, 2);
        autolabor::pm1::go_straight_timing(-0.2, 2);
        
        autolabor::pm1::lock();
        
        autolabor::pm1::shutdown();
    }
}
