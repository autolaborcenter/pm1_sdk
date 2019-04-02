//
// Created by User on 2019/4/2.
//

#include "pm1_sdk.h"

#include "internal/chassis.hh"
#include "internal/serial/serial.h"
#include "internal/raii/weak_shared_lock.hh"
#include "internal/process_controller.hh"

extern "C" {
#include "internal/control_model/model.h"
}

std::atomic<autolabor::odometry_t>
		odometry_mark = ATOMIC_VAR_INIT({});

std::shared_ptr<autolabor::pm1::chassis>
		ptr;

std::shared_mutex
		mutex;

// ===========================================================

constexpr auto
		chassis_pointer_busy = "chassis pointer is busy",
		null_chassis_pointer = "null chassis pointer",
		infinite_action      = "action never complete",
		illegal_argument     = "illegal argument",
		invalid_target       = "invalid target";

#define POINTER_ASSERT                    \
weak_shared_lock lock(mutex);             \
if (!lock) return {chassis_pointer_busy}; \
if (!ptr)  return {null_chassis_pointer}

#define POINTER_ASSERT_OR(DEFAULT)                 \
weak_shared_lock lock(mutex);                      \
if (!lock) return {chassis_pointer_busy, DEFAULT}; \
if (!ptr)  return {null_chassis_pointer, DEFAULT}

// ===========================================================

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
		return {null_chassis_pointer};
	}
}

autolabor::pm1::result<void>
autolabor::pm1::drive(double v, double w) {
	POINTER_ASSERT;
	
	velocity temp{static_cast<float>(v), static_cast<float>(w)};
	ptr->set_target(velocity_to_physical(&temp, &default_config));
	return {};
}

autolabor::pm1::result<autolabor::pm1::odometry>
autolabor::pm1::get_odometry() {
	const static autolabor::pm1::odometry
			nan{NAN, NAN, NAN, NAN, NAN, NAN};
	
	POINTER_ASSERT_OR(nan);
	auto temp = ptr->odometry() - odometry_mark;
	return {"", {temp.x, temp.y, temp.theta, temp.vx, temp.vy, temp.w}};
}

autolabor::pm1::result<void>
autolabor::pm1::reset_odometry() {
	POINTER_ASSERT;
	odometry_mark = ptr->odometry();
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::lock() {
	POINTER_ASSERT;
	ptr->disable();
	return {};
}

autolabor::pm1::result<void> autolabor::pm1::unlock() {
	POINTER_ASSERT;
	ptr->enable();
	return {};
}

autolabor::pm1::result<autolabor::pm1::chassis_state>
autolabor::pm1::get_chassis_state() {
	POINTER_ASSERT;
	
	auto temp = ptr->state();
	return {"", {(node_state) temp._ecu0,
	             (node_state) temp._ecu1,
	             (node_state) temp._tcu}};
}

void
autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1>>(time));
}

#undef POINTER_ASSERT
#undef POINTER_ASSERT_OR

// =====================================================================

constexpr autolabor::process_controller
		move_controller(0, 0.01, 0.2, 0.1);

autolabor::pm1::result<void>
autolabor::pm1::go_straight(double speed, double distance) {
	if (speed == 0)
		return {distance == 0 ? "" : infinite_action};
	if (distance <= 0)
		return {invalid_target};
	
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::go_straight_timing(double speed, double time) {
	if (time < 0)
		return {invalid_target};
	
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc(double speed, double r, double rad) {
	if (r == 0)
		return {illegal_argument};
	if (speed == 0)
		return {rad == 0 ? "" : infinite_action};
	if (rad <= 0)
		return {invalid_target};
	
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	if (r == 0)
		return {illegal_argument};
	if (time < 0)
		return {invalid_target};
	
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::turn_around(double speed, double rad) {
	if (speed == 0)
		return {rad == 0 ? "" : infinite_action};
	if (rad <= 0)
		return {invalid_target};
	
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::turn_around_timing(double speed, double time) {
	if (time < 0)
		return {invalid_target};
	
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::pause() {
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::resume() {
	return {};
}
