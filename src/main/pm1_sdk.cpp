//
// Created by ydrml on 2019/2/22.
//

#include "pm1_sdk.h"

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <thread>
#include "internal/time_extensions.h"
#include "internal/chassis.hh"
#include "internal/mechanical.h"

using namespace autolabor::pm1;

result::operator bool() const { return error_info.empty(); }

using seconds = std::chrono::duration<double, std::ratio<1>>;

std::shared_ptr<chassis> _ptr;

/** 空安全检查 */
inline const std::shared_ptr<chassis> &ptr() {
	if (!_ptr) throw std::exception("chassis has not been initialized!");
	return _ptr;
}

/** 记录暂停状态 */
volatile bool paused = false;

/** 循环的间隔 */
inline void loop_delay() { delay(0.05); }

/** 检查并执行 */
inline result run(const std::function<void()> &code,
                  const std::function<void()> &recover = [] {}) {
	try { code(); } catch (std::exception &e) { return {e.what()}; }
	return {};
}

namespace wheels {
	/** 获取编码器均值 */
	inline double average() {
		return mechanical::radius * .5 * (ptr()->left().position + ptr()->right().position);
	}
	
	/** 获取编码器差 */
	inline double difference() {
		return mechanical::radius * std::abs(ptr()->left().position - ptr()->right().position);
	}
}

namespace block {
	/** 减速距离 */
	constexpr auto slow_down_begin = 1.0;
	constexpr auto slow_down_end   = .1;
	
	/** 最小线速度 */
	constexpr auto min_speed = 0.1;
	
	/** 最大线速度 */
	constexpr auto max_speed = 2.0;
	
	/** 求与目标特定距离时的最大速度 */
	inline double max_speed_when(double distance_difference) {
		constexpr static auto k = (max_speed - min_speed) / (slow_down_begin - slow_down_end);
		
		return distance_difference > slow_down_begin
		       ? max_speed
		       : distance_difference < slow_down_end
		         ? min_speed
		         : k * (distance_difference - slow_down_end) + min_speed;
	}
	
	/** 求目标速度下实际应有的速度 */
	inline double actual_speed(double target, double distance_difference) {
		auto actual = std::min(std::abs(target), max_speed_when(distance_difference));
		return target > 0 ? actual : -actual;
	}
	
	/** 直接设置控制量 */
	inline void set(double l, double r, double rudder) {
		ptr()->left(l / mechanical::radius);
		ptr()->right(r / mechanical::radius);
		ptr()->rudder(rudder);
	}
	
	/** 阻塞等待后轮转动 */
	inline void wait_or_drive(double v, double w) {
		if ((v == 0 && w == 0) || paused)
			set(0, 0, ptr()->rudder().position);
		else {
			auto target_rudder = v == 0
			                     ? w > 0 ? -mechanical::pi / 2 : +mechanical::pi / 2
			                     : std::atan(w * mechanical::length / v);
			
			if (std::abs(ptr()->rudder().position - target_rudder) > mechanical::pi / 36) {
				set(0, 0, target_rudder);
			} else {
				auto diff = mechanical::width / 2 * w;
				set(v - diff, v + diff, target_rudder);
			}
		}
	}
	
	/** 阻塞并抑制输出 */
	seconds inhibit() {
		const auto temp = mechdancer::common::now();
		while (paused) {
			wait_or_drive(0, 0);
			loop_delay();
		}
		return mechdancer::common::now() - temp;
	}
	
	/** 按固定控制量运行指定时间 */
	void go_timing(double v, double w, double seconds) {
		using namespace mechdancer::common;
		
		auto ending = now() + seconds_duration(seconds);
		while (now() < ending) {
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
	               [](const serial::PortInfo &it) -> std::string { return it.port; });
	return result;
}

result autolabor::pm1::initialize(const std::string &port) {
	if (port.empty()) {
		for (const auto &item:serial_ports())
			if (initialize(item))
				return delay(.2), result{};
		
		return {"no available port"};
	} else {
		try {
			_ptr = std::make_shared<chassis>(port);
			return delay(.2), result{};
		}
		catch (std::exception &e) {
			_ptr = nullptr;
			return {e.what()};
		}
	}
}

result autolabor::pm1::shutdown() {
	return _ptr = nullptr, result{};
}

result autolabor::pm1::go_straight(double speed, double distance) {
	if (speed == 0 && distance != 0) return {"this action will never complete"};
	return run([speed, distance] {
		const auto o = wheels::average();
		double     distance_difference;
		
		while ((distance_difference = distance - std::abs(wheels::average() - o)) > 0) {
			block::wait_or_drive(block::actual_speed(speed, distance_difference), 0);
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
		const auto o = wheels::average();
		const auto d = std::abs(r * rad);
		double     distance_difference;
		
		while ((distance_difference = d - std::abs(wheels::average() - o)) > 0) {
			auto actual_speed = block::actual_speed(speed, distance_difference);
			block::wait_or_drive(actual_speed, actual_speed / r);
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
		const auto o = wheels::difference();
		while (std::abs(wheels::difference() - o) < std::abs(mechanical::width * rad)) {
			block::wait_or_drive(0, speed);
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
	std::this_thread::sleep_for(mechdancer::common::seconds_duration(time));
}

autolabor::pm1::Odometry autolabor::pm1::get_odometry() {
	throw std::runtime_error("to do");
}

result autolabor::pm1::drive(double v, double w) {
	return result{"to do"};
}
