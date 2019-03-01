//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include <thread>
#include "../main/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	for (const auto &name: serial_ports()) {
		std::cout << name << std::endl;
	}
	
	initialize("com3");
	std::thread([] {
		std::this_thread::sleep_for(std::chrono::seconds(1));
		shutdown();
	}).detach();
	std::cerr << go_straight(1, 1).error_info << std::endl;
}
