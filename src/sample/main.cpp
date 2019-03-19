//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
	initialize();
	
	std::thread([] {
		while (true) {
			std::cout << get_odometry().yaw << std::endl;
		}
	}).detach();
	
	turn_around(0.2, mechanical::pi * 2);
	delay(1);
}

#pragma clang diagnostic pop
