#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
	std::thread([] {
		while (true) {
			std::cout << autolabor::pm1::get_odometry().data.x << std::endl;
			autolabor::pm1::delay(0.1);
		}
	}).detach();
	
	while (true) {
		auto result = autolabor::pm1::initialize();
		if (result)
			std::cout << result.data << std::endl;
		else {
			std::cerr << result.error_info << std::endl;
			continue;
		}
		
		autolabor::pm1::unlock();

		const auto time = std::chrono::steady_clock::now();
		while (std::chrono::steady_clock::now() - time < std::chrono::seconds(3)) {
			autolabor::pm1::drive(+0.2, 0);
			autolabor::pm1::delay(0.1);
		}
		
		while (std::chrono::steady_clock::now() - time < std::chrono::seconds(6)) {
			autolabor::pm1::drive(-0.2, 0);
			autolabor::pm1::delay(0.1);
		}

		autolabor::pm1::lock();
		
		autolabor::pm1::shutdown();
	}
}

#pragma clang diagnostic pop
