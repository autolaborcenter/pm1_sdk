//
// Created by ydrml on 2019/2/22.
//

#include <thread>
#include <iostream>
#include "../main/pm1_sdk.h"
#include "../main/internal/chassis.hh"

using namespace autolabor::pm1;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
	chassis chassis("com4");
	while (true) {
		chassis.set_target({1, 1.57});
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

#pragma clang diagnostic pop
