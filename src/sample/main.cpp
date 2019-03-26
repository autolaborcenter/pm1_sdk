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
			std::cout << get_odometry().x << ", "
			          << get_odometry().y << ", "
			          << get_odometry().yaw << std::endl;
			delay(0.1);
		}
	}).detach();
	
	turn_around(0.157, 1.57);
	delay(1);
	return 0;
}
