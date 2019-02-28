//
// Created by ydrml on 2019/2/22.
//

#include "pm1_sdk.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "internal/time_extensions.h"
#include "internal/chassis.hh"
#include "internal/chassis/pm1.h"

using namespace autolabor::pm1;
using namespace mechdancer::common;

result::operator bool() const { return error_info.empty(); }

using seconds = std::chrono::duration<double, std::ratio<1>>;

std::shared_ptr<chassis> ptr;

// 记录暂停状态
volatile bool paused = false;

// 循环的间隔
inline void loop_delay() { delay(0.05); }

inline result run(const std::function<void()> &code,
                  const std::function<void()> &recover) {
	try { code(); }
	catch (std::exception &e) {
		recover();
		return {e.what()};
	}
	return {};
}

namespace wheels {
	/** 获取编码器均值 */
	inline double average() {
		return mechanical::radius * .5 * (ptr->left().position + ptr->right().position);
	}
	
	/** 获取编码器差 */
	inline double difference() {
		return mechanical::radius * std::abs(ptr->left().position - ptr->right().position);
	}
}

namespace block {
	/** 直接设置控制量 */
	inline void set(double l, double r, double rudder) {
		ptr->left(l);
		ptr->right(r);
		ptr->rudder(rudder);
	}
	
	/** 阻塞等待后轮转动 */
	inline void wait_or_drive(double v, double w) {
		if ((v == 0 && w == 0) || paused)
			set(0, 0, ptr->rudder().position);
		else {
			auto target_rudder = v == 0
			                     ? w > 0 ? -mechanical::pi / 2 : +mechanical::pi / 2
			                     : std::atan(w * mechanical::length / v);
			
			if (std::abs(ptr->rudder().position - target_rudder) > mechanical::pi / 36) {
				set(0, 0, target_rudder);
			} else {
				auto diff = mechanical::width / 2 * w;
				set(v - diff, v + diff, target_rudder);
			}
		}
	}
	
	/** 阻塞并抑制输出 */
	seconds inhibit() {
		const auto temp = now();
		while (paused) {
			wait_or_drive(0, 0);
			loop_delay();
		}
		return now() - temp;
	}
	
	/** 按固定控制量运行指定时间 */
	void go_timing(double v, double w, double seconds) {
		auto ending = now() + seconds_duration(seconds);
		while (now() < ending) {
			if (paused) ending += inhibit();
			else wait_or_drive(v, w);
			
			loop_delay();
		}
	}
	
	
}

result autolabor::pm1::initialize(const std::string &port) {
	return run([&port] { ptr = std::make_shared<chassis>(port); },
	           [] { ptr = nullptr; });
}

result autolabor::pm1::initialize() {
	return {"to do"};
}

result autolabor::pm1::shutdown() {
	return ptr = nullptr, result{};
}

result autolabor::pm1::go_straight(double speed, double distance) {
	return run([speed, distance] {
		const auto o = wheels::average();
		while (std::abs(wheels::average() - o) < distance) {
			block::wait_or_drive(speed, 0);
			loop_delay();
		}
	}, [] {});
}

result autolabor::pm1::go_straight_timing(double speed, double time) {
	return run([speed, time] { block::go_timing(speed, speed, time); }, [] {});
	
}

result autolabor::pm1::go_arc(double speed, double r, double rad) {
	return run([speed, r, rad] {
		const auto o = wheels::average();
		while (std::abs(wheels::average() - o) < r * rad) {
			block::wait_or_drive(speed, speed / r);
			loop_delay();
		}
	}, [] {});
}

result autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	return run([speed, r, time] { block::go_timing(speed, speed / r, time); }, [] {});
}

result autolabor::pm1::turn_around(double speed, double rad) {
	return run([speed, rad] {
		const auto o = wheels::difference();
		while (std::abs(wheels::difference() - o) < mechanical::width * rad) {
			block::wait_or_drive(0, speed);
			loop_delay();
		}
	}, [] {});
}

result autolabor::pm1::turn_around_timing(double speed, double time) {
	return run([speed, time] { block::go_timing(0, speed, time); }, [] {});
}

result autolabor::pm1::pause() {
	return run([] {
		block::wait_or_drive(0, 0);
		paused = true;
	}, [] {});
}

result autolabor::pm1::resume() {
	return paused = false, result{};
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
