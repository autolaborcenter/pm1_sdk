#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
    size_t i = 0;
    while (true) {
        auto result = autolabor::pm1::initialize();
        std::cout << i++ << ": ";
        if (result)
            std::cout << result.value << std::endl;
        else
            std::cerr << result.error_info << std::endl;
        
        autolabor::pm1::cancel_action();
        autolabor::pm1::shutdown();
    }
}
