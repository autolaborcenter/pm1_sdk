//
// Created by ydrml on 2019/2/22.
//

#include "api.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "extensions.h"
#include "time_extensions.h"
#include "internal/chassis.hh"
#include "internal/chassis/pm1.h"

using namespace autolabor::pm1;
using namespace mechdancer::common;
using seconds = std::chrono::duration<double, std::ratio<1>>;

// 记录暂停状态
volatile bool paused = false;

// 循环的间隔
inline void loop_delay() { delay(0.05); }

std::shared_ptr<chassis> ptr;

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
		
		if (std::abs(ptr->rudder().position - target_rudder) > mechanical::pi / 36)
			set(0, 0, target_rudder);
		
		else {
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

/** 按固定控制量运行指定时间 */
void go_timing(double v, double w, double seconds) {
	auto ending = now() + seconds_duration(seconds);
	while (now() < ending) {
		if (paused) ending += inhibit();
		else wait_or_drive(v, w);
		
		loop_delay();
	}
}

bool autolabor::pm1::initialize(const std::string &port) {
	try {
		ptr = std::make_shared<chassis>(port);
		std::cout << "chassis initialized on " << port << std::endl;
	} catch (const std::exception &e) {
		ptr = nullptr;
		std::cerr << "initialize failed, because: " << e.what() << std::endl;
	}
	return ptr != nullptr;
}

bool autolabor::pm1::initialize() {
	throw std::runtime_error("to do");
}

void autolabor::pm1::shutdown() {
	ptr = nullptr;
}

void autolabor::pm1::go_straight(double speed, double distance) {
	const auto o = wheels::average();
	while (std::abs(wheels::average() - o) < distance) {
		wait_or_drive(speed, speed);
		loop_delay();
	}
}

void autolabor::pm1::go_straight_timing(double speed, double time) {
	go_timing(speed, speed, time);
}

void autolabor::pm1::go_arc(double speed, double r, double rad) {
	const auto o = wheels::average();
	while (std::abs(wheels::average() - o) < r * rad) {
		wait_or_drive(speed, speed / r);
		loop_delay();
	}
}

void autolabor::pm1::go_arc_timing(double speed, double r, double time) {
	go_timing(speed, speed / r, time);
}

void autolabor::pm1::turn_around(double speed, double rad) {
	const auto o = wheels::difference();
	while (std::abs(wheels::difference() - o) < mechanical::width * rad) {
		wait_or_drive(0, speed);
		loop_delay();
	}
}

void autolabor::pm1::turn_around_timing(double speed, double time) {
	go_timing(0, speed, time);
}

void autolabor::pm1::pause() {
	wait_or_drive(0, 0);
	paused = true;
}

void autolabor::pm1::resume() {
	paused = false;
}

void autolabor::pm1::delay(double time) {
	std::this_thread::sleep_for(mechdancer::common::seconds_duration(time));
}

autolabor::pm1::Odometry autolabor::pm1::get_odometry() {
	throw std::runtime_error("to do");
}

void autolabor::pm1::drive(double v, double w) {
	throw std::runtime_error("to do");
}
