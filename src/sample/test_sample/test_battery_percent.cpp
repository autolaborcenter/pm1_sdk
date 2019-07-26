//
// Created by User on 2019/7/26.
//

#include <pm1_sdk_native.h>
#include <iostream>
#include <thread>

namespace autolabor::pm1::test {
    void on_native(autolabor::pm1::native::handler_t handler) {
        auto error = std::string(autolabor::pm1::native::get_error_info(handler));
        native::remove_error_info(handler);
        if (!error.empty()) std::cerr << error << std::endl;
    }
}

int main() {
    using namespace autolabor::pm1;
    double _;
    
    test::on_native(native::initialize("", _));
    for (size_t i = 0; i < 10; ++i) {
        _ = -1;
        test::on_native(native::get_battery_percent(_));
        std::cout << _ << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    native::shutdown();
    return 0;
}
