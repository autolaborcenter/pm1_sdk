#include <iostream>
#include <thread>
#include "../main/pm1_sdk.h"
#include "../main/exception.h"

using namespace autolabor::pm1;

int main() {
	auto result = initialize();
	if (!result) {
		std::cout << result.error_info << std::endl;
		return 1;
	}
	
	std::thread([] {
		while (true) {
			std::cout << get_odometry().data.x << ", "
			          << get_odometry().data.y << ", "
			          << get_odometry().data.yaw << ", "
			          << (int) get_chassis_state().data._ecu0 << ", "
			          << (int) get_chassis_state().data._ecu1 << ", "
			          << (int) get_chassis_state().data._tcu << std::endl;
			delay(0.1);
		}
	}).detach();
	
	go_straight(0.2, 0.5);
	
	reset_odometry();
	
	go_straight(-0.2, 0.4);
	
	delay(10);
	return 0;
}
