//
// Created by User on 2019/4/1.
//

#include <iostream>

#include "pm1_sdk.h"

int main() {
	using namespace autolabor::pm1;
	
	if (!initialize()) return 1;
	set_parameter(parameter_id::wheel_radius, 0.12);
}
                            
