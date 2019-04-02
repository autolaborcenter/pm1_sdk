#include <iostream>
#include <thread>
#include "pm1_sdk.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
using namespace autolabor::pm1;

int main() {
	//	autolabor::pm1::chassis chassis("COM6");
	//
	//	std::thread([&chassis] {
	//		while (true) {
	//			std::cout << chassis.odometry().x << ", "
	//			          << chassis.odometry().y << ", "
	//			          << chassis.odometry().theta << std::endl;
	//			std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//		}
	//	}).detach();
	//
	//	std::thread([&chassis] {
	//		while (true) {
	//			std::cout << std::boolalpha
	//			          << chassis.get_state().check_all(node_state_t::enabled)
	//			          << std::endl;
	//			std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//		}
	//	}).detach();
	//
	//	auto time = std::chrono::steady_clock::now();
	//
	//	while (std::chrono::steady_clock::now() - time < std::chrono::seconds(3)) {
	//		chassis.set_target({+0.3, 0});
	//		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//	}
	//
	//	while (std::chrono::steady_clock::now() - time < std::chrono::seconds(6)) {
	//		chassis.set_target({-0.3, 0});
	//		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//	}
	
	while (true) {
		auto result = initialize();
		if (result)
			std::cout << result.data << std::endl;
		else
			std::cerr << result.error_info << std::endl;
		shutdown();
	}
	return 0;
}

#pragma clang diagnostic pop
