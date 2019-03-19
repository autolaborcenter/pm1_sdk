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
#include "internal/time_extensions.h"
#include "internal/chassis.hh"
#include "internal/mechanical.h"

using namespace autolabor::pm1;

result::operator bool() const { return error_info.empty(); }

std::shared_ptr<chassis> _ptr;

/** 空安全检查 */
inline std::shared_ptr<chassis> ptr() {
	auto copy = _ptr;
	if (!copy) throw std::exception("chassis has not been initialized!");
	return copy;
}

/** 记录暂停状态 */
volatile bool paused = false;

/** 循环的间隔 */
inline void loop_delay() { delay(0.01); }

/** 检查并执行 */
inline result run(const std::function<void()> &code,
                  const std::function<void()> &recover = [] {}) {
	try { code(); } catch (std::exception &e) { return {e.what()}; }
	return {};
}

struct process_controlller {
	double x0, y0, x1, y1, k;
	
	constexpr process_controlller(
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

const auto max_v = default_config.max_wheel_speed,
           max_w = 2 * default_config.max_wheel_speed / default_config.width;

const process_controlller
		move_up(0, 0.05, 0.5, max_v),                  // NOLINT(cert-err58-cpp)
		move_down(0.05, 0.05, 3, max_v),               // NOLINT(cert-err58-cpp)
		rotate_up(0, pi_f / 18, pi_f / 4, max_w),      // NOLINT(cert-err58-cpp)
		rotate_down(pi_f / 9, pi_f / 36, pi_f, max_w); // NOLINT(cert-err58-cpp)

namespace block {
	/** 阻塞等待后轮转动 */
	inline void wait_or_drive(double v, double w) {
		if ((v == 0 && w == 0) || paused)
			ptr()->set_state(0, ptr()->rudder().position);
		else {
			auto rudder_target = v == 0
			                     ? w > 0
			                       ? -pi_f / 2
			                       : +pi_f / 2
			                     : -std::atan(w * default_config.length / v);
			
			if (std::abs(ptr()->rudder().position - rudder_target) > pi_f / 36)
				ptr()->set_state(0, rudder_target);
			else
				ptr()->set_target(v, w);
		}
	}
	
	/** 阻塞并抑制输出 */
	autolabor::seconds_floating inhibit() {
		return autolabor::measure_time([] {
			while (paused) {
				wait_or_drive(0, 0);
				loop_delay();
			}
		});
	}
	
	/** 按固定控制量运行指定时间 */
	void go_timing(double v, double w, double seconds) {
		auto ending = autolabor::now() + autolabor::seconds_duration(seconds);
		while (autolabor::now() < ending) {
			if (paused) ending += inhibit();
			else wait_or_drive(v, w);
			
			loop_delay();
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

result autolabor::pm1::initialize(const std::string &port) {
	if (port.empty()) {
		std::stringstream builder;
		for (const auto   &item : serial_ports()) {
			auto result = initialize(item);
			if (result) return {};
			builder << item << ": " << result.error_info << std::endl;
		}
		auto              msg = builder.str();
		return msg.empty() ? result{"no available port"}
		                   : result{msg};
	} else {
		try {
			_ptr = std::make_shared<chassis>(port);
			return {};
		}
		catch (std::exception &e) {
			_ptr = nullptr;
			return {e.what()};
		}
	}
}

result autolabor::pm1::shutdown() {
	return _ptr ? _ptr = nullptr, result{} : result{"chassis doesn't exist"};
}

result autolabor::pm1::go_straight(double speed, double distance) {
	if (speed == 0 && distance != 0) return {"this action will never complete"};
	return run([speed, distance] {
		const auto o = ptr()->odometry().s;
		
		while (true) {
			auto current = std::abs(ptr()->odometry().s - o),
			     rest    = distance - current;
			if (rest < 0) break;
			auto actual = std::min({std::abs(speed),
			                        move_up(current),
			                        move_down(rest)});
			block::wait_or_drive(speed > 0 ? actual : -actual, 0);
			loop_delay();
		}
	});
}

result autolabor::pm1::go_straight_timing(double speed, double time) {
	return run([speed, time] { block::go_timing(speed, 0, time); });
}

result autolabor::pm1::go_arc(double speed, double r, double rad) {
	if (speed == 0 && rad != 0) return {"this action will never complete"};
	return run([speed, r, rad] {
		const auto o = ptr()->odometry().s;
		const auto d = std::abs(r * rad);
		
		while (true) {
			auto current = std::abs(ptr()->odometry().s - o),
			     rest    = d - current;
			if (rest < 0) break;
			auto actual    = std::min({std::abs(speed),
			                           move_up(current),
			                           move_down(rest)}),
			     available = speed > 0 ? actual : -actual;
			block::wait_or_drive(available, available / r);
			loop_delay();
		}
	});
}

result autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	return run([speed, r, time] { block::go_timing(speed, speed / r, time); });
}

result autolabor::pm1::turn_around(double speed, double rad) {
	if (speed == 0 && rad != 0) return {"this action will never complete"};
	return run([speed, rad] {
		const auto o = ptr()->odometry().theta;
		while (true) {
			auto current = std::abs(ptr()->odometry().theta - o),
			     rest    = rad - current;
			if (rest < 0) break;
			auto actual = std::min({std::abs(speed),
			                        rotate_up(current),
			                        rotate_down(rest)});
			block::wait_or_drive(0, speed > 0 ? actual : -actual);
			loop_delay();
		}
	});
}

result autolabor::pm1::turn_around_timing(double speed, double time) {
	return run([speed, time] { block::go_timing(0, speed, time); });
}

result autolabor::pm1::pause() {
	return run([] {
		block::wait_or_drive(0, 0);
		paused = true;
	});
}

result autolabor::pm1::resume() {
	return run([] { paused = false; });
}

void autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(autolabor::seconds_duration(time));
}

autolabor::pm1::odometry autolabor::pm1::get_odometry() {
	try {
		auto odometry = ptr()->odometry();
		return {odometry.x,
		        odometry.y,
		        odometry.theta,
		        odometry.vx,
		        odometry.vy,
		        odometry.w};
	} catch (std::exception &) {
		return {NAN, NAN, NAN, NAN, NAN, NAN};
	}
}

result autolabor::pm1::drive(double v, double w) {
	return run([v, w] { ptr()->set_target(v, w); });
}

result autolabor::pm1::reset_odometry() {
	return run([] { ptr()->clear_odometry(); });
}
