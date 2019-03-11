//
// Created by ydrml on 2019/2/22.
//

#include <conio.h>
#include <iostream>
#include <thread>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"
#include "../main/internal/can_define.h"

using namespace autolabor::pm1;

int main() {
	auto result = initialize();
	if (!result) std::cerr << result.error_info << std::endl;
	
	std::thread([] {
		while (true) {
			if (get_odometry().yaw < -1) {
				std::cout << get_odometry().yaw << std::endl;
				pause();
				break;
			}
			//
			//			std::cout << get_odometry().x << '\t'
			//			          << get_odometry().y << '\t'
			//			          << get_odometry().yaw << std::endl;
			
			delay(0.1);
		}
	}).detach();
	
	turn_around(1, 2 * mechanical::pi);
	
	//	go_straight(+.5, 1);
	//	turn_around(-1, mechanical::pi / 2);
	//	go_straight(+.5, 1);
	//	turn_around(-1, mechanical::pi);
	//	go_straight(+.5, 1);
	//	turn_around(1, mechanical::pi / 2);
	//	go_straight(+.5, 1);
	//	turn_around(-1, mechanical::pi);
	
	delay(1.0);
	
	//	while (result) {
	//		go_straight_timing(.5, 1);
	//		turn_around(mechanical::pi / 2, mechanical::pi / 2);
	//		auto odometry = get_odometry();
	//		std::cout << "x = " << odometry.x << ",\t"
	//		          << "y = " << odometry.y << ",\t"
	//		          << "θ = " << odometry.yaw << std::endl;
	//		while (_kbhit());
	//	}
	
	shutdown();
}
