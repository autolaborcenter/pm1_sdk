//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "../main/pm1_sdk.h"

using namespace autolabor::pm1;

int main() {
	initialize();
	
	//	std::thread([] {
	//		while (true)
	//			std::cout << get_odometry().x << std::endl;
	//	}).detach();
	
	go_straight(0.1, 1);
}
