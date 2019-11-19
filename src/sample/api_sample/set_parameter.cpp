//
// Created by User on 2019/4/1.
//

#include "pm1_sdk.h"

int main() {
    using namespace autolabor::pm1;
    
    if (initialize()) set_parameter(parameter_id::max_wheel_speed, .5);
}
