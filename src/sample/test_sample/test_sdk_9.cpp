#include <iostream>
#include <thread>
#include "../pm1_sdk.h"
#include "../../main/pm1_sdk_native.h"

extern "C" {
#include "../../main/internal/control_model/chassis_config_t.h"
}

int main() {
	double progress = 0;
	
	std::thread([&] {
		while (true) {
			//			std::cout << autolabor::pm1::get_odometry().value.yaw << ", "
			//			          << progress
			//			          << std::endl;
			//			autolabor::pm1::delay(0.1);
		}
	}).detach();
	
	auto result = autolabor::pm1::initialize();
	if (result)
		std::cout << result.value << std::endl;
	else {
		std::cerr << result.error_info << std::endl;
		return 1;
	}
	autolabor::pm1::unlock();
	while (true) {
		using namespace autolabor::pm1::native;
		drive_spatial(+0.1, 0, spatium_calculate(1, 0), progress);
		drive_spatial(0, +0.34, spatium_calculate(0, pi_f), progress);
		drive_spatial(+0.1, 0, spatium_calculate(1, 0), progress);
		drive_spatial(0, -0.34, spatium_calculate(0, pi_f), progress);
	}
	autolabor::pm1::shutdown();
}
