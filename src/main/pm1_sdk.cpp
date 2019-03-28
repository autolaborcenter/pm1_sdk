//
// Created by ydrml on 2019/2/22.
//

#include "pm1_sdk.h"

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include <iostream>
#include "internal/time_extensions.h"
#include "internal/chassis.hh"
#include "exception.h"

using namespace autolabor::pm1;

std::shared_ptr<chassis> _ptr;

constexpr auto serial_error_prefix     = "IO Exception";
constexpr auto try_wrong_chassis       = "it's not a pm1 chassis";
constexpr auto chassis_not_initialized = "chassis has not been initialized";
constexpr auto action_cannot_complete  = "this action will never complete";
constexpr auto illegal_target          = "target state should greater than 0";

inline uint16_t build_code(const std::string &what) noexcept {
	union_error_code error{};
	
	if (what.find_first_of(serial_error_prefix) == 0 || what == try_wrong_chassis)
		error.bits.serial_error = true;
	
	else if (what == chassis_not_initialized)
		error.bits.not_initialized = true;
	
	else if (what == action_cannot_complete || what == illegal_target)
		error.bits.illegal_argument = true;
	
	return error.code;
}

/** 空安全检查 */
inline std::shared_ptr<chassis> ptr() {
	auto copy = _ptr;
	if (!copy) throw std::exception(chassis_not_initialized);
	return copy;
}

/** 记录暂停状态 */
volatile bool paused = false;

/** 检查并执行 */
template<class t>
inline result<t> run(const std::function<t()> &code) {
	try {
		return {0, "", code()};
	}
	catch (std::exception &e) {
		return {build_code(e.what()), e.what()};
	}
	catch (...) {
		union_error_code error{};
		error.bits.others = true;
		return {error.code, "unknown and not-exception error"};
	}
}

/** 检查并执行 */
template<>
inline result<void> run(const std::function<void()> &code) {
	try {
		code();
		return {};
	}
	catch (std::exception &e) {
		return {build_code(e.what()), e.what()};
	}
	catch (...) {
		union_error_code error{};
		error.bits.others = true;
		return {error.code, "unknown and not-exception error"};
	}
}

struct process_controller {
	double x0, y0, x1, y1, k;
	
	constexpr process_controller(
			double x0, double y0, double x1, double y1)
			: x0(x0), y0(y0),
			  x1(x1), y1(y1),
			  k((y1 - y0) / (x1 - x0)) {
		if (x0 < 0 || x1 < x0 ||
		    y0 < 0 || y1 < y0)
			throw std::exception("illegal parameters");
	}
	
	inline double operator()(double x) const {
		return x < x0 ? y0
		              : x > x1 ? y1
		                       : k * (x - x0) + y0;
	}
};

const auto max_v = 3 * 2 * pi_f,
           max_w = max_v / default_config.width;

const process_controller
		move_up(0, 0.01, 0.5, max_v),                     // NOLINT(cert-err58-cpp)
		move_down(0.02, 0.01, 10, max_v),                 // NOLINT(cert-err58-cpp)
		rotate_up(0, pi_f / 90, pi_f / 4, max_w),         // NOLINT(cert-err58-cpp)
		rotate_down(pi_f / 900, pi_f / 180, pi_f, max_w); // NOLINT(cert-err58-cpp)

namespace block {
	/** 阻塞等待后轮转动 */
	inline void wait_or_drive(double v, double w) {
		if (paused)
			ptr()->set_target({0, NAN});
		else {
			velocity temp = {static_cast<float>(v), static_cast<float>(w)};
			ptr()->set_target(velocity_to_physical(&temp, &default_config));
		}
	}
	
	/** 阻塞并抑制输出 */
	autolabor::seconds_floating inhibit() {
		return autolabor::measure_time([] {
			while (paused) {
				ptr()->set_target({0, NAN});
				std::this_thread::yield();
			}
		});
	}
	
	/** 按固定控制量运行指定时间 */
	void go_timing(double v, double w, double seconds) {
		auto ending = autolabor::now() + autolabor::seconds_duration(seconds);
		while (autolabor::now() < ending) {
			if (paused) ending += inhibit();
			else wait_or_drive(v, w);
			std::this_thread::yield();
		}
	}
}

std::vector<std::string> autolabor::pm1::serial_ports() {
	auto                     info = serial::list_ports();
	std::vector<std::string> result(info.size());
	std::transform(info.begin(), info.end(), result.begin(),
	               [](const serial::PortInfo &it) { return it.port; });
	return result;
}

result<void> autolabor::pm1::initialize(const std::string &port) {
	if (port.empty()) {
		std::stringstream builder;
		for (const auto   &item : serial_ports()) {
			auto result = initialize(item);
			if (result) return {};
			builder << item << ": " << result.error_info << std::endl;
		}
		
		union_error_code error{};
		error.bits.no_serial = true;
		auto msg = builder.str();
		
		return {error.code, msg.empty() ? "no available port" : msg};
	} else {
		try {
			_ptr = std::make_shared<chassis>(port);
			return {};
		}
		catch (std::exception &e) {
			_ptr = nullptr;
			union_error_code error{};
			error.bits.no_serial = true;
			return {error.code, e.what()};
		}
	}
}

result<void> autolabor::pm1::shutdown() {
	if (_ptr) {
		_ptr = nullptr;
		return {};
	} else {
		union_error_code error{};
		error.bits.not_initialized = true;
		return {error.code, chassis_not_initialized};
	}
}

result<void> autolabor::pm1::go_straight(double speed, double distance) {
	return run<void>([speed, distance] {
		if (speed == 0) {
			if (distance == 0) return;
			throw std::exception(action_cannot_complete);
		}
		if (distance <= 0)
			throw std::exception(illegal_target);
		
		const auto o = ptr()->steady_odometry().s;
		
		while (true) {
			auto current = std::abs(ptr()->steady_odometry().s - o),
			     rest    = distance - current;
			if (rest < 0) break;
			auto actual = std::min({std::abs(speed),
			                        move_up(current),
			                        move_down(rest)});
			block::wait_or_drive(speed > 0 ? actual : -actual, 0);
			std::this_thread::yield();
		}
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::go_straight_timing(double speed, double time) {
	return run<void>([speed, time] { block::go_timing(speed, 0, time); });
}

result<void> autolabor::pm1::go_arc(double speed, double r, double rad) {
	return run<void>([speed, r, rad] {
		if (r == 0) throw std::exception(illegal_target);
		if (speed == 0) {
			if (rad == 0) return;
			throw std::exception(action_cannot_complete);
		}
		if (rad <= 0)
			throw std::exception(illegal_target);
		
		const auto o = ptr()->steady_odometry().s;
		const auto d = std::abs(r * rad);
		
		while (true) {
			auto current = std::abs(ptr()->steady_odometry().s - o),
			     rest    = d - current;
			if (rest < 0) break;
			auto actual    = std::min({std::abs(speed),
			                           move_up(current),
			                           move_down(rest)}),
			     available = speed > 0 ? actual : -actual;
			block::wait_or_drive(available, available / r);
			std::this_thread::yield();
		}
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	return run<void>([speed, r, time] {
		if (r == 0) throw std::exception(illegal_target);
		block::go_timing(speed, speed / r, time);
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::turn_around(double speed, double rad) {
	return run<void>([speed, rad] {
		if (speed == 0) {
			if (rad == 0) return;
			throw std::exception(action_cannot_complete);
		}
		if (rad <= 0)
			throw std::exception(illegal_target);
		
		if (rad < 0.01) return;
		auto temp = rad - 0.01;
		
		const auto o = ptr()->steady_odometry().theta;
		while (true) {
			auto current = std::abs(ptr()->steady_odometry().theta - o),
			     rest    = temp - current;
			if (rest < 0) break;
			auto actual = std::min({std::abs(speed),
			                        rotate_up(current),
			                        rotate_down(rest)});
			block::wait_or_drive(0, speed > 0 ? actual : -actual);
			std::this_thread::yield();
		}
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::turn_around_timing(double speed, double time) {
	return run<void>([speed, time] {
		block::go_timing(0, speed, time);
		ptr()->set_target({0, NAN});
	});
}

result<void> autolabor::pm1::pause() {
	return run<void>([] {
		block::wait_or_drive(0, 0);
		paused = true;
	});
}

result<void> autolabor::pm1::resume() {
	return run<void>([] { paused = false; });
}

void autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(autolabor::seconds_duration(time));
}

result<odometry> autolabor::pm1::get_odometry() {
	return run<odometry>([] {
		auto data = ptr()->odometry();
		return odometry{data.x,
		                data.y,
		                data.theta,
		                data.vx,
		                data.vy,
		                data.w};
	});
}

result<void> autolabor::pm1::drive(double v, double w) {
	return run<void>([v, w] {
		velocity temp = {static_cast<float>(v), static_cast<float>(w)};
		ptr()->set_target(velocity_to_physical(&temp, &default_config));
	});
}

result<void> autolabor::pm1::reset_odometry() {
	return run<void>([] { ptr()->clear_odometry(); });
}

result<void> autolabor::pm1::lock() {
	return run<void>([] { ptr()->disable(); });
}

result<void> autolabor::pm1::unlock() {
	return run<void>([] { ptr()->enable(); });
}

result<chassis_state> autolabor::pm1::get_chassis_state() {
	return run<chassis_state>([] {
		auto temp = ptr()->get_state();
		return chassis_state{(node_state) temp._ecu0,
		                     (node_state) temp._ecu1,
		                     (node_state) temp._tcu};
	});
}
