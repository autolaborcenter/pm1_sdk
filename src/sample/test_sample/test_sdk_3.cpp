#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
	std::thread([] {
		while (true) {
			auto state = autolabor::pm1::get_chassis_state().data._ecu0 == autolabor::pm1::node_state::enabled;
			std::cout << state << std::endl;
			autolabor::pm1::delay(0.1);
		}
	}).detach();
	
	std::thread([] {
		while (true) {
			autolabor::pm1::lock();
			autolabor::pm1::delay(0.1);
			autolabor::pm1::unlock();
			autolabor::pm1::delay(0.1);
		}
	}).detach();
	
	while (true) {
		auto result = autolabor::pm1::initialize();
		if (result)
			std::cout << result.data << std::endl;
		else
			std::cerr << result.error_info << std::endl;
		autolabor::pm1::shutdown();
	}
}

#pragma clang diagnostic pop
