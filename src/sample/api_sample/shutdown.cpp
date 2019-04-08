//
// Created by User on 2019/4/1.
//

#include <iostream>
#include <thread>
#include "pm1_sdk.h"

int main() {
	std::thread([] {
		while (true) {
			std::cout << (int) autolabor::pm1::get_chassis_state().value._ecu0 << std::endl;
			autolabor::pm1::delay(0.1);
		}
	}).detach();
	
	if (!autolabor::pm1::initialize()) return 1;
	autolabor::pm1::delay(1);
	autolabor::pm1::shutdown();
	autolabor::pm1::delay(1);
}
