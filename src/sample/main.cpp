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
			          << get_odometry().data.yaw << std::endl;
			delay(0.1);
		}
	}).detach();
	
	unlock();
	
	//	go_straight(0.2, 0.4);
	go_arc(0.1, 0.5, 1);
	delay(1);
	return 0;
}
