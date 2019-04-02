//
// Created by User on 2019/4/2.
//

#include "pm1_sdk.h"

#include "internal/chassis.hh"
#include "internal/serial/serial.h"

#include <shared_mutex>

std::atomic<autolabor::odometry_t>
		odometry_mark = ATOMIC_VAR_INIT({});

std::shared_ptr<autolabor::pm1::chassis>
		ptr;

std::shared_mutex
		mutex;

std::vector<std::string> autolabor::pm1::serial_ports() {
	auto                     info = serial::list_ports();
	std::vector<std::string> result(info.size());
	std::transform(info.begin(), info.end(), result.begin(),
	               [](const serial::PortInfo &it) { return it.port; });
	return result;
}

autolabor::pm1::result<std::string>
autolabor::pm1::initialize(const std::string &port) {
	
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
		std::unique_lock<std::shared_mutex> lock(mutex);
		try {
			ptr = std::make_shared<chassis>(port);
			odometry_mark.store({});
			return {"", port};
		}
		catch (std::exception &e) {
			ptr = nullptr;
			return {e.what()};
		}
	}
}

autolabor::pm1::result<void>
autolabor::pm1::shutdown() {
	std::unique_lock<std::shared_mutex> lock(mutex);
	
	if (ptr) {
		ptr = nullptr;
		return {};
	} else {
		return {"null chassis pointer"};
	}
}

autolabor::pm1::result<autolabor::pm1::odometry>
autolabor::pm1::get_odometry() {
	std::shared_lock<std::shared_mutex> lock(mutex);
	if (!ptr) return {"null chassis pointer", {NAN, NAN, NAN, NAN, NAN, NAN}};
	
	auto temp = ptr->odometry() - odometry_mark;
	return {"", {temp.x, temp.y, temp.theta, temp.vx, temp.vy, temp.w}};
}

autolabor::pm1::result<void>
autolabor::pm1::reset_odometry() {
	std::shared_lock<std::shared_mutex> lock(mutex);
	if (!ptr) return {"null chassis pointer"};
	
	odometry_mark = ptr->odometry();
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::lock() {
	std::shared_lock<std::shared_mutex> lock(mutex);
	if (!ptr) return {"null chassis pointer"};
	
	ptr->disable();
	return {};
}

autolabor::pm1::result<void> autolabor::pm1::unlock() {
	std::shared_lock<std::shared_mutex> lock(mutex);
	if (!ptr) return {"null chassis pointer"};
	
	ptr->enable();
	return {};
}

void
autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1>>(time));
}
