//
// Created by ydrml on 2019/2/22.
//

#include <conio.h>
#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"
#include "../main/internal/can_define.h"

using namespace autolabor::pm1;

int main() {
	auto result = initialize();
	if (!result) std::cerr << result.error_info << std::endl;
	
	while (result) {
		go_straight_timing(.5, 1);
		turn_around(mechanical::pi / 2, mechanical::pi / 2);
		auto odometry = get_odometry();
		std::cout << "x = " << odometry.x << ",\t"
		          << "y = " << odometry.y << ",\t"
		          << "θ = " << odometry.yaw << std::endl;
		while (_kbhit());
	}
}
