//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include <conio.h>
#include <thread>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

int main() {
	auto result = initialize();
	if (!result) {
		std::cerr << result.error_info << std::endl;
		return 1;
	}
	
	std::thread([] {
		delay(1);
		pause();
		std::cout << "paused" << std::endl;
		delay(3);
		resume();
		std::cout << "resumed" << std::endl;
	}).detach();
	
	go_straight_timing(1, 2);
	std::cout << "done" << std::endl;
	
	double v = 0, w = 0;
	while (true) {
		if (_kbhit()) {
			auto c = std::tolower(_getch());
			switch (c) {
				case 'w':
					v = +.2;
					w = 0;
					break;
				case 'a':
					v = 0;
					w = +.3;
					break;
				case 's':
					v = -.2;
					w = 0;
					break;
				case 'd':
					v = 0;
					w = -.3;
					break;
				case ' ':
					v = w = 0;
					break;
				case 27:
					return 0;
				default:
					break;
			}
		}
		drive(v, w);
		delay(0.05);
	}
}
