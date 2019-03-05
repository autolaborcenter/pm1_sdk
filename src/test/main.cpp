//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include <conio.h>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

int main() {
	initialize();
	double v = 0, w = 0;
	while (true) {
		if (_kbhit()) {
			auto c = std::tolower(_getch());
			switch (c) {
				case 'w':
					v = .2;
					break;
				case 'a':
					w = .3;
					break;
				case 's':
					v = -.2;
					break;
				case 'd':
					w = -.3;
					break;
				case 27:
					return 0;
				default:
					break;
			}
		} else {
			v = w = 0;
		}
		drive(v, w);
	}
}
