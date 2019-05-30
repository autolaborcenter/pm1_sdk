//
// Created by User on 2019/4/1.
//

#include <iostream>

#include "pm1_sdk.h"

int main() {
    using namespace autolabor::pm1;
    
    if (!initialize()) return 1;
    reset_parameter(parameter_id::width);
    reset_parameter(parameter_id::length);
    reset_parameter(parameter_id::wheel_radius);
    reset_parameter(parameter_id::max_wheel_speed);
    reset_parameter(parameter_id::max_v);
    reset_parameter(parameter_id::max_w);
    reset_parameter(parameter_id::optimize_width);
    reset_parameter(parameter_id::acceleration);
    reset_parameter(parameter_id::max_v);
    reset_parameter(parameter_id::max_w);
}
