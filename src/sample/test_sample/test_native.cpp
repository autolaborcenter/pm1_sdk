//
// Created by User on 2019/8/6.
//

#include <pm1_sdk_native.h>
#include <string>
#include <iostream>
#include <internal/control_model/pi.h>

int main() {
    using namespace autolabor::pm1;
    
    double _;
    auto   id    = native::initialize("", _);
    auto   error = std::string(native::get_error_info(id));
    
    if (!error.empty()) {
        std::cerr << error << std::endl;
        return 1;
    }
    
    native::set_enabled(true);
    
    while (true) {
        native::drive_wheels(-M_PI, M_PI);
    }
    
    return 0;
}
