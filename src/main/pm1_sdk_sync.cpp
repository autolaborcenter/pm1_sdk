//
// Created by User on 2019/4/2.
//

#include "pm1_sdk.h"

#include "internal/chassis.hh"
#include "internal/serial/serial.h"

#include <mutex>
#include <memory>

std::atomic<autolabor::odometry_t>       odometry_mark;
std::recursive_mutex                     mutex;
std::shared_ptr<autolabor::pm1::chassis> ptr;

std::vector<std::string> autolabor::pm1::serial_ports() {
	auto                     info = serial::list_ports();
	std::vector<std::string> result(info.size());
	std::transform(info.begin(), info.end(), result.begin(),
	               [](const serial::PortInfo &it) { return it.port; });
	return result;
}

autolabor::pm1::result<std::string>
autolabor::pm1::initialize(const std::string &port) {
	std::lock_guard<std::recursive_mutex> lock(mutex);
	
	odometry_mark.store({});
	
	if (port.empty()) {
		std::stringstream builder;
		for (const auto   &item : serial_ports()) {
			auto result = initialize(item);
			if (result) return {"", item};
			builder << item << ": " << result.error_info << std::endl;
		}
		
		auto msg = builder.str();
		return {msg.empty() ? "no available port" : msg};
	} else {
		try {
			ptr = std::make_shared<chassis>(port);
			return {"", port};
		}
		catch (std::exception &e) {
			ptr = nullptr;
			return {e.what()};
		}
	}
}

autolabor::pm1::result<void> autolabor::pm1::shutdown() {
	std::lock_guard<std::recursive_mutex> lock(mutex);
	
	if (ptr) {
		ptr = nullptr;
		return {};
	} else {
		return {"null chassis pointer"};
	}
}
