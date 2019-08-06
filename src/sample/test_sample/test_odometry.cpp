//
// Created by User on 2019/8/6.
//

#include <pm1_sdk_native.h>
#include <string>
#include <iostream>

int main() {
    using namespace autolabor::pm1;
    
    double _;
    auto   id    = native::initialize("", _);
    auto   error = std::string(native::get_error_info(id));
    if (!error.empty()) {
        native::remove_error_info(id);
        std::cout << error << std::endl;
        return 1;
    }
    
    // 设置参数、修改状态
    native::set_parameter(0, 0.465);
    native::set_parameter(1, 0.355);
    native::set_parameter(2, 0.105);
    
    native::set_enabled(true);
    native::set_command_enabled(false);
    
    while (true);
    
    return 0;
}
