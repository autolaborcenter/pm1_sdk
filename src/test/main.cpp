//
// Created by ydrml on 2019/2/22.
//

#include <iostream>
#include <random>
#include "../main/pm1_sdk.h"
#include "../main/internal/mechanical.h"

using namespace autolabor::pm1;

int main() {
	while (true) {
		initialize();
		std::cout << "tik";
		delay(.1);
		shutdown();
		std::cout << "-tok" << std::endl;
		delay(.1);
	}
}
