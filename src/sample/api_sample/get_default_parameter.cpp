//
// Created by User on 2019/4/1.
//

#include <iostream>

#include "pm1_sdk.h"

int main() {
	using namespace autolabor::pm1;
	
	std::cout << "width          : " << get_defualt_parameter(parameter_id::width) << std::endl
	          << "length         : " << get_defualt_parameter(parameter_id::length) << std::endl
	          << "wheel_radius   : " << get_defualt_parameter(parameter_id::wheel_radius) << std::endl
	          << "max_wheel_speed: " << get_defualt_parameter(parameter_id::max_wheel_speed) << std::endl
	          << "max_v          : " << get_defualt_parameter(parameter_id::max_v) << std::endl
	          << "max_w          : " << get_defualt_parameter(parameter_id::max_w) << std::endl
	          << "optimize_width : " << get_defualt_parameter(parameter_id::optimize_width) << std::endl
	          << "acceleration   : " << get_defualt_parameter(parameter_id::acceleration) << std::endl;
}
