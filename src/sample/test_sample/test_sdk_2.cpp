#include <iostream>
#include <thread>
#include "../pm1_sdk.h"

int main() {
	std::thread([] {
		while (true) {
			autolabor::pm1::lock();
			std::cout << "tik-" << std::endl;
			autolabor::pm1::delay(0.1);
			autolabor::pm1::unlock();
			std::cout << "-tok" << std::endl;
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
