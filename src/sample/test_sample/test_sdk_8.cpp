#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
    std::thread([] {
        while (true) {
            //			std::cout << autolabor::pm1::get_odometry().data.x << std::endl;
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
        autolabor::pm1::go_arc_va(+0.2, 1, 0.5);
        autolabor::pm1::go_arc_va(-0.2, 1, 0.5);
        autolabor::pm1::delay(1);
    }
}
