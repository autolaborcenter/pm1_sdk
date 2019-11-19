//
// Created by User on 2019/4/1.
//

#include <iostream>
#include "pm1_sdk.h"

int main() {
    using namespace autolabor::pm1;
    
    if (!initialize()) return 1;
    std::cout << "width          : " << get_parameter(parameter_id::width).value << std::endl
              << "length         : " << get_parameter(parameter_id::length).value << std::endl
              << "left_radius    : " << get_parameter(parameter_id::left_radius).value << std::endl
              << "right_radius   : " << get_parameter(parameter_id::right_radius).value << std::endl
              << "max_wheel_speed: " << get_parameter(parameter_id::max_wheel_speed).value << std::endl
              << "max_v          : " << get_parameter(parameter_id::max_v).value << std::endl
              << "max_w          : " << get_parameter(parameter_id::max_w).value << std::endl
              << "optimize_width : " << get_parameter(parameter_id::optimize_width).value << std::endl
              << "acceleration   : " << get_parameter(parameter_id::acceleration).value << std::endl;
}
