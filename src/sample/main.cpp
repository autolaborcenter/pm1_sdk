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
			//			std::cout << get_odometry().data.x << ", "
			//			          << get_odometry().data.y << ", "
			//			          << get_odometry().data.yaw << ", "
			std::cout << (int) get_chassis_state().data._ecu0 << ", "
			          << (int) get_chassis_state().data._ecu1 << ", "
			          << (int) get_chassis_state().data._tcu << std::endl;
			delay(0.1);
		}
	}).detach();
	
	//go_arc(0.1, 0.5, 2);
	
	delay(3);
	return 0;
}
