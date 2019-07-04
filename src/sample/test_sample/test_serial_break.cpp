//
// Created by User on 2019/7/4.
//

#include "../../main/pm1_sdk_native.h"

#include <string>
#include <iostream>
#include <thread>

int main() {
    using namespace autolabor::pm1;
    size_t times = 0;
    while (true) {
        double progress;
        auto   handler = native::initialize("", progress);
        auto   error   = std::string(native::get_error_info(handler));
        if (!error.empty()) {
            native::remove_error_info(handler);
            std::cout << error << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        std::cout << "connected" << std::endl;
        
        while (native::check_state() != 0);
        native::shutdown();
        
        std::cout << times++ << std::endl;
    }
}
