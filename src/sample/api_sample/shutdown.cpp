//
// Created by User on 2019/4/1.
//

#include <iostream>
#include <thread>
#include "pm1_sdk.h"

int main() {
	using namespace autolabor::pm1;
	
	std::thread([] {
		while (true) {
			std::cout << static_cast<int>(check_state()) << std::endl;
			delay(0.1);
		}
	}).detach();
	
	if (!initialize()) return 1;
	delay(1);
	shutdown();
	delay(1);
}
