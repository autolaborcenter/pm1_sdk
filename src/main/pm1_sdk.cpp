//
// Created by User on 2019/4/2.
//

#include "pm1_sdk.h"
#include "pm1_sdk_native.h"

#include <algorithm>
#include <functional>
#include <thread>

#include "internal/serial/serial.h"
#include "internal/time_extensions.h"
#include "internal/control_model/chassis_config_t.h"

autolabor::pm1::result<void>
on_native(std::function < autolabor::pm1::native::handler_t() > && block) {
	using namespace autolabor::pm1;
	
	auto handler = block();
	auto error   = std::string(native::get_error_info(handler));
	native::remove_error_info(handler);
	return {error};
}

std::vector<std::string> autolabor::pm1::serial_ports() {
	auto                     info = serial::list_ports();
	std::vector<std::string> result(info.size());
	std::transform(info.begin(), info.end(), result.begin(),
	               [](const serial::PortInfo &it) { return it.port; });
	return result;
}

autolabor::pm1::result<std::string>
autolabor::pm1::initialize(const std::string &port, double *progress) {
	auto handler = native::initialize(port.c_str(), progress);
	auto error   = std::string(native::get_error_info(handler));
	native::remove_error_info(handler);
	
	return {error, std::string(native::get_current_port())};
}

autolabor::pm1::result<void>
autolabor::pm1::shutdown() {
	return on_native([] { return native::shutdown(); });
}

autolabor::pm1::result<void>
autolabor::pm1::drive(double v, double w) {
	return on_native([=] { return native::drive(v, w); });
}

autolabor::pm1::result<autolabor::pm1::odometry>
autolabor::pm1::get_odometry() {
	double   _;
	odometry temp{};
	
	auto handler = native::get_odometry
		(_, _,
		 temp.x, temp.y, temp.yaw,
		 temp.vx, temp.vy, temp.w);
	auto error   = std::string(native::get_error_info(handler));
	native::remove_error_info(handler);
	return {error, temp};
}

autolabor::pm1::result<void>
autolabor::pm1::reset_odometry() {
	return on_native([] { return native::reset_odometry(); });
}

autolabor::pm1::result<void>
autolabor::pm1::lock() {
	return on_native([] { return native::lock(); });
}

autolabor::pm1::result<void> autolabor::pm1::unlock() {
	return on_native([] { return native::unlock(); });
}

autolabor::pm1::result<autolabor::pm1::chassis_state>
autolabor::pm1::get_chassis_state() {
	uint8_t state;
	
	auto handler = native::check_state(state);
	auto error   = std::string(native::get_error_info(handler));
	native::remove_error_info(handler);
	return {error, static_cast<chassis_state>(state)};
}

void
autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(seconds_duration(time));
}

inline double transform(double s, double sa) {
	const static auto w_2 = default_config.width / 2;
	return std::abs(s + w_2 * sa) + std::abs(s - w_2 * sa);
}

constexpr auto
	infinite_action = "action never complete",
	invalid_target  = "invalid target";

autolabor::pm1::result<void>
autolabor::pm1::go_straight(double speed,
                            double distance,
                            double *progress) {
	if (speed == 0)
		return {distance == 0 ? "" : infinite_action};
	if (distance <= 0)
		return {invalid_target};
	
	double _progress;
	return on_native([&] {
		return native::drive_spatial(
			speed, 0, transform(distance, 0),
			progress ? *progress : _progress);
	});
}

autolabor::pm1::result<void>
autolabor::pm1::go_straight_timing(double speed,
                                   double time,
                                   double *progress) {
	if (time < 0)
		return {invalid_target};
	
	double _progress;
	return on_native([&] {
		return native::drive_timing(
			speed, 0, time,
			progress ? *progress : _progress);
	});
}

autolabor::pm1::result<void>
autolabor::pm1::turn_around(double speed,
                            double rad,
                            double *progress) {
	if (speed == 0)
		return {rad == 0 ? "" : infinite_action};
	if (rad <= 0)
		return {invalid_target};
	
	double _progress;
	return on_native([&] {
		return native::drive_spatial(
			0, speed, transform(0, rad),
			progress ? *progress : _progress);
	});
}

autolabor::pm1::result<void>
autolabor::pm1::turn_around_timing(double speed,
                                   double time,
                                   double *progress) {
	if (time < 0)
		return {invalid_target};
	
	double _progress;
	return on_native([&] {
		return native::drive_timing(
			0, speed, time,
			progress ? *progress : _progress);
	});
}


autolabor::pm1::result<void>
autolabor::pm1::go_arc(double speed,
                       double r,
                       double rad,
                       double *progress) {
	if (std::abs(r) < 0.05)
		return {"radius is too little, use turn_around instead"};
	if (speed == 0)
		return {rad == 0 ? "" : infinite_action};
	if (rad <= 0)
		return {invalid_target};
	
	double _progress;
	return on_native([&] {
		return native::drive_spatial(
			speed, speed / r, transform(r * rad, rad),
			progress ? *progress : _progress);
	});
}

autolabor::pm1::result<void>
autolabor::pm1::go_arc_timing(double speed,
                              double r,
                              double time,
                              double *progress) {
	if (std::abs(r) < 0.05)
		return {"radius is too little, use turn_around instead"};
	if (time < 0)
		return {invalid_target};
	
	double _progress;
	return on_native([&] {
		return native::drive_timing(
			speed, speed / r, time,
			progress ? *progress : _progress);
	});
}

autolabor::pm1::result<void>
autolabor::pm1::pause() {
	native::pause();
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::resume() {
	native::resume();
	return {};
}

autolabor::pm1::result<void>
autolabor::pm1::cancel_all() {
	native::cancel_all();
	return {};
}
