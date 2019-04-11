//
// Created by User on 2019/4/10.
//

#include <thread>
#include <iostream>
#include "../main/pm1_sdk_native.h"

using namespace autolabor::pm1::native;

int main() {
	double        progress = 0;
	std::thread([&] {
		while (true) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			std::cout << 100 * progress << '%' << std::endl;
		}
	}).detach();
	
	auto handler = initialize(nullptr, progress);
	auto error   = std::string(get_error_info(handler));
	remove_error_info(handler);
	if (!error.empty()) {
		std::cerr << error << std::endl;
		return 1;
	}
	std::cout << "connected: " << get_current_port() << std::endl;
	
	handler = drive_spatial(0.15, 0, 1.0, progress);
	error   = std::string(get_error_info(handler));
	remove_error_info(handler);
	if (!error.empty()) {
		std::cerr << error << std::endl;
		return 1;
	}
	
	double _, s, x;
	get_odometry(s, _, x, _, _, _, _, _);
	
	std::cout << s << ", " << x << std::endl;
	
	return 0;
}
